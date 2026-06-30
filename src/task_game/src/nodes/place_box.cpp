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

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace {

// 激光传感器判断 box0 是否成功放置的距离阈值（<= 视为未放置成功，平板被卡死）。
constexpr int plane_limit_dst = 2;

// 机械臂协议：box1 从手上放，box0 从平板放；二层命令保留独立编号。
constexpr int kArmPlaceHandFirstFloor = 5;
constexpr int kArmPlaceHandSecondFloor = 6;
constexpr int kArmPlacePlateFirstFloor = 3;
constexpr int kArmPlacePlateSecondFloor = 4;

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

int32_t stage_for_slot(BoxSlot slot) {
    return slot == BoxSlot::Box1 ? Robot::kTreePlaceBox1 : Robot::kTreePlaceBox0;
}

const BoxMoveTask& task_for_slot(const MoveBoxPlan& plan, BoxSlot slot) {
    return slot == BoxSlot::Box1 ? plan.box1 : plan.box0;
}

int place_command_for_slot(BoxSlot slot, bool second_floor) {
    if (slot == BoxSlot::Box1) {
        return second_floor ? kArmPlaceHandSecondFloor : kArmPlaceHandFirstFloor;
    }
    return second_floor ? kArmPlacePlateSecondFloor : kArmPlacePlateFirstFloor;
}

const char* slot_name(BoxSlot slot) {
    return slot == BoxSlot::Box1 ? "box1" : "box0";
}

// 把某个剩余箱子任务改造成“单吸到手上、再从手上放置”的伪双箱计划：
// 真实箱子放在 box1 槽（走 box1 抓取/放置链），box0 槽留空被跳过。
// box1.to_dst 串联原 to_box + to_dst，使其先去抓箱再去放置点。
MoveBoxPlan make_single_hand_place_plan(const MoveBoxPlan& previous_plan, const BoxMoveTask& remaining_task)
{
    MoveBoxPlan plan;
    plan.box0 = remaining_task;
    plan.box1 = remaining_task;
    plan.hand_only_plan = true;
    plan.box0.to_box = {};
    plan.box0.to_dst = {};
    plan.box1.to_dst = remaining_task.to_box;
    plan.box1.to_dst.trajectory.insert(
        plan.box1.to_dst.trajectory.end(), remaining_task.to_dst.trajectory.begin(), remaining_task.to_dst.trajectory.end());
    plan.box1.to_dst.target_points.insert(
        plan.box1.to_dst.target_points.end(), remaining_task.to_dst.target_points.begin(), remaining_task.to_dst.target_points.end());
    plan.dst2_pos = previous_plan.dst2_pos;
    plan.dst0_to_dst2 = previous_plan.dst0_to_dst2;
    return plan;
}

// box0 平板放置失败后重建计划：
// 1) 失败轮之后的每个剩余箱子（box0、box1）都改为单吸手放计划；
// 2) 末尾追加一轮“回头重试卡住的平板箱”计划（plate_retry）。
// 注意：二层判定不在这里计算，统一由放置时读取黑板 placed_count 决定，
// 这样卡住未成功的箱子不会把同 ID 的后续箱子错误抬到第二层。
std::vector<MoveBoxPlan> make_replan_after_plate_blocked(const std::vector<MoveBoxPlan>& move_plan, int blocked_plan_index)
{
    std::vector<MoveBoxPlan> replan;
    replan.reserve((move_plan.size() - blocked_plan_index) * 2);

    const auto blocked_task = move_plan[blocked_plan_index].box0;

    for (int index = blocked_plan_index + 1; index < static_cast<int>(move_plan.size()); ++index) {
        const auto& old_plan = move_plan[index];
        replan.push_back(make_single_hand_place_plan(old_plan, old_plan.box0));
        replan.push_back(make_single_hand_place_plan(old_plan, old_plan.box1));
    }

    // 末轮重试：直接复用卡住轮计划，box0 用卡住的箱子；只走 box0 平板放置，放完结束。
    MoveBoxPlan retry_plan = move_plan[blocked_plan_index];
    retry_plan.box0 = blocked_task;
    retry_plan.plate_retry_plan = true;
    retry_plan.finish_after_box0_place = true;
    retry_plan.skip_box0_place_check = true;
    replan.push_back(retry_plan);
    return replan;
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
        }, true)) {
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


PlaceBoxAction::PlaceBoxAction(BoxSlot slot)
    : BT::ActionNode(slot == BoxSlot::Box1 ? "place_box1_action" : "place_box0_action"), slot_(slot) {}

void PlaceBoxAction::arm_place_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) { arm_state_ = msg->data; }


BT::Status PlaceBoxAction::execute(BT& tree) {

    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    arm_state_ = 0;

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂放置 %s 完成", slot_name(slot_));

    if (!subscriptions_ready_) {
        arm_cmd_pub_ = context->node_->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);

        arm_state_sub = context->node_->create_subscription<std_msgs::msg::Int32>(
            "arm_cmd_state", 10, std::bind(&PlaceBoxAction::arm_place_cmd_callback, this, std::placeholders::_1));

        subscriptions_ready_ = true;
    }

    if (!wait_for_stage(context, stage_for_slot(slot_))) {
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

    // 激光重规划阶段跳过：
    // - 单吸计划(hand_only 非 retry)的 box0 放置跳过（真实箱已由 box1 放置），推进到下一计划；
    // - 末轮平板重试(plate_retry)只走 box0 放置，box1 放置阶段跳过。
    if (slot_ == BoxSlot::Box0 && plan.hand_only_plan && !plan.plate_retry_plan) {
        tree.write_msg("plan_index", plan_index + 1);
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 单吸计划跳过 box0 放置阶段");
        return BT::SUCCESS;
    }
    if (slot_ == BoxSlot::Box1 && plan.plate_retry_plan) {
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 平板重试计划跳过 box1 放置阶段");
        return BT::SUCCESS;
    }

    const auto& task = task_for_slot(plan, slot_);
    const int box_id = task.box_id;

    // 运行期二层判定：读黑板 placed_count，已放置过同 ID 的箱子则放到第二层。
    std::array<int, 4> placed_count{};
    const bool has_placed_count = tree.read_msg("placed_count", placed_count);
    bool second_floor = task.place_at_second_floor;
    if (has_placed_count && box_id >= 0 && box_id < 4) {
        second_floor = placed_count[box_id] > 0;
    }
    place_at_second_floor_ = second_floor;

    std_msgs::msg::Int32 msg;
    msg.data = place_command_for_slot(slot_, place_at_second_floor_);
    arm_cmd_pub_->publish(msg);

    RCLCPP_INFO(context->node_->get_logger(), "放置 %s，arm_cmd=%d，id=%d，second_floor=%s", slot_name(slot_), msg.data, box_id, place_at_second_floor_ ? "true" : "false");

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {

        if (arm_state_ == 1) {
            RCLCPP_INFO(context->node_->get_logger(), "放置 %s 完成", slot_name(slot_));
            arm_state_ = 0;
            break;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");
            break;
        }

        std::this_thread::sleep_for(10ms);
    }

    // box1 从手上放置：视为成功，更新二层计数后推进。
    if (slot_ == BoxSlot::Box1) {
        if (has_placed_count && box_id >= 0 && box_id < 4) {
            ++placed_count[box_id];
            tree.write_msg("placed_count", placed_count);
        }
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        return BT::SUCCESS;
    }

    // box0 从平板放置：用激光判断是否真正放置成功。
    if (slot_ == BoxSlot::Box0) {
        bool place_failed = false;
        if (!plan.skip_box0_place_check) {
            const bool enable_plane_dst_replan = context->enable_plane_dst_replan_.load();
            const bool debug_force_replan = context->debug_force_replan_.load();
            const bool plane_dst_received = context->plane_dst_received_.load();

            if (debug_force_replan) {
                // 调试用：无激光时一次性强制触发重规划，触发后复位。
                place_failed = true;
                context->debug_force_replan_.store(false);
                RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 调试参数触发重规划");
            } else if (enable_plane_dst_replan) {
                if (!plane_dst_received) {
                    RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 尚未收到 plane_dst，跳过激光重规划判断");
                } else {
                    place_failed = context->plane_dst_buffer_.load() <= plane_limit_dst;
                }
            }
        }

        if (place_failed) {
            // 平板被卡死：重建剩余箱子为单吸手放计划 + 末轮平板重试，从下一轮重新开始。
            // 注意：失败的箱子未计入 placed_count，不影响同 ID 后续箱子的楼层判定。
            RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: box0 平板放置失败(卡死)，开始重新规划轨迹");
            const auto replan = make_replan_after_plate_blocked(move_plan, plan_index);
            tree.write_msg("move_plan", replan);
            tree.write_msg<int>("plan_index", 0);
            // 从 ArriveToBox0 阶段重新进入，单吸计划会在 box0 各阶段自动跳过。
            context->tree_start_key = Robot::kTreeArriveToBox0;
            RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 已生成 %zu 轮重规划计划", replan.size());
            return BT::SUCCESS;
        }

        // 放置成功，更新二层计数。
        if (has_placed_count && box_id >= 0 && box_id < 4) {
            ++placed_count[box_id];
            tree.write_msg("placed_count", placed_count);
        }

        // 每轮放完 box0 都退让到 dst2；dst2 用本轮真实 box0 id 重算。
        MoveBoxPlan retreat_plan = plan;
        std::array<std::array<float, 3>, 4> place_table{};
        if (box_id >= 0 && box_id < 4 && tree.read_msg("place_table", place_table)) {
            retreat_plan.dst2_pos = {place_table[box_id][0] - 0.1f, place_table[box_id][1], 3.14f};
        }

        const bool final_round = plan.finish_after_box0_place || (plan_index + 1 >= static_cast<int>(move_plan.size()));
        if (final_round) {
            RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 全部搬箱计划执行完成");
        }
        if (!retreat_to_dst2(context, retreat_plan, final_round)) {
            return BT::FAILED;
        }
        if (!final_round) {
            tree.write_msg("plan_index", plan_index + 1);
        }
    }

    if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
        context->advance_tree_stage();
    }

    return BT::SUCCESS;
}
