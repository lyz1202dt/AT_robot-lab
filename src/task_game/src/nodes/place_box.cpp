#include "nodes/place_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <array>
#include <atomic>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

// 机械臂协议：单吸手上放置，一层/二层命令分别为 5/6。
constexpr int kArmPlaceHandFirstFloor = 5;
constexpr int kArmPlaceHandSecondFloor = 6;

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

int place_command(bool second_floor) {
    return second_floor ? kArmPlaceHandSecondFloor : kArmPlaceHandFirstFloor;
}

Pilot::TargetPoint make_retreat_target(const MoveBoxPlan& plan)
{
    Pilot::TargetPoint target;
    target.target_pos = Eigen::Vector2d(plan.dst2_pos[0], plan.dst2_pos[1]);
    target.target_yaw = plan.dst2_pos[2];
    target.constraint_target_yaw = plan.dst0_to_dst2.constraint_target_yaw;
    target.target_vel = plan.dst0_to_dst2.target_vel;
    target.max_velocity = plan.dst0_to_dst2.max_velocity;
    target.max_accelation = plan.dst0_to_dst2.max_accelation;
    target.max_omega = plan.dst0_to_dst2.max_omega;
    target.kp = plan.dst0_to_dst2.kp;
    target.allow_start_dir_error = plan.dst0_to_dst2.allow_start_dir_error;
    target.allow_final_dir_error = plan.dst0_to_dst2.allow_final_dir_error;
    target.allow_final_pos_allow = plan.dst0_to_dst2.allow_final_pos_allow;
    target.adjust_min_vel = plan.dst0_to_dst2.adjust_min_vel;
    target.adjust_min_omega = plan.dst0_to_dst2.adjust_min_omega;
    target.allow_y_vel = plan.dst0_to_dst2.allow_y_vel;
    target.trajectory_connection_radius = 0.0f;
    return target;
}

bool retreat_to_dst2(Robot* context, const MoveBoxPlan& plan, bool final_round)
{
    const auto target = make_retreat_target(plan);
    if (!context->pilot->set_target(target)) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 设置退让目标失败");
        return false;
    }

    bool finished = false;
    bool success = false;
    if (!context->pilot->start([&finished, &success](int result) {
            success = (result != 0);
            finished = true;
        }, false)) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 启动退让到 dst2 失败");
        return false;
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 本轮搬箱完成，退让到 dst2=(%.2f, %.2f, %.2f)",
        plan.dst2_pos[0],
        plan.dst2_pos[1],
        plan.dst2_pos[2]);

    while (rclcpp::ok() && context->auto_pilot_enabled.load() && !finished) {
        std::this_thread::sleep_for(50ms);
    }

    if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
        context->pilot->stop();
        return false;
    }

    if (!success) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 退让到 dst2 失败");
        context->pilot->stop();
        return false;
    }

    // 仅最后一轮全部搬箱完成后切入手动模式；中间轮退让后继续下一轮。
    if (final_round) {
        context->enter_manual_mode();
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 已退让到 dst2 并切入手动模式");
    }
    return true;
}

} // namespace

PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}

void PlaceBoxAction::arm_place_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) { arm_state_ = msg->data; }

BT::Status PlaceBoxAction::execute(BT& tree) {

    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    arm_state_ = 0;

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂放置完成");

    if (!subscriptions_ready_) {
        arm_cmd_pub_ = context->node_->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);

        arm_state_sub = context->node_->create_subscription<std_msgs::msg::Int32>(
            "arm_cmd_state", 10, std::bind(&PlaceBoxAction::arm_place_cmd_callback, this, std::placeholders::_1));

        subscriptions_ready_ = true;
    }

    if (!wait_for_stage(context, Robot::kTreePlaceBox)) {
        context->pilot->stop();
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;

    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }

    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    const auto& plan = move_plan[plan_index];
    const int box_id = plan.box_id;

    // 运行期二层判定：读黑板 placed_count，已放置过同 ID 的箱子则放到第二层。
    std::array<int, 4> placed_count{};
    const bool has_placed_count = tree.read_msg("placed_count", placed_count);
    bool second_floor = plan.place_at_second_floor;
    if (has_placed_count && box_id >= 0 && box_id < 4) {
        second_floor = placed_count[box_id] > 0;
    }
    place_at_second_floor_ = second_floor;

    std_msgs::msg::Int32 msg;
    msg.data = place_command(place_at_second_floor_);
    arm_cmd_pub_->publish(msg);

    RCLCPP_INFO(context->node_->get_logger(), "放置箱子，arm_cmd=%d，id=%d，second_floor=%s", msg.data, box_id, place_at_second_floor_ ? "true" : "false");

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        if (arm_state_ == 1) {
            RCLCPP_INFO(context->node_->get_logger(), "放置完成");
            arm_state_ = 0;
            break;
        }

        if (arm_state_ == -1) {
            RCLCPP_WARN(context->node_->get_logger(), "机械臂返回放置失败");
            arm_state_ = 0;
            break;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");
            break;
        }

        std::this_thread::sleep_for(10ms);
    }

    if (has_placed_count && box_id >= 0 && box_id < 4) {
        ++placed_count[box_id];
        tree.write_msg("placed_count", placed_count);
    }

    MoveBoxPlan retreat_plan = plan;
    std::array<std::array<float, 3>, 4> place_table{};
    if (box_id >= 0 && box_id < 4 && tree.read_msg("place_table", place_table)) {
        retreat_plan.dst2_pos = {place_table[box_id][0] - 0.05f, place_table[box_id][1], place_table[box_id][2]};
    }

    const bool final_round = plan_index + 1 >= static_cast<int>(move_plan.size());
    if (final_round) {
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 全部搬箱计划执行完成");
    }
    if (!retreat_to_dst2(context, retreat_plan, final_round)) {
        return BT::FAILED;
    }
    if (!final_round) {
        tree.write_msg("plan_index", plan_index + 1);
    }
    if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
        context->advance_tree_stage();
    }

    return BT::SUCCESS;
}
