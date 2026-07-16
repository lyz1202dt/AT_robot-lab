#include "nodes/place_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
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

// 机械臂协议：box1 从手上放，box0 从平板放；二层命令保留独立编号。
constexpr int kArmPlaceHandFirstFloor = 5;
constexpr int kArmPlaceHandSecondFloor = 6;
constexpr int kArmPlacePlateFirstFloor = 3;
constexpr int kArmPlacePlateSecondFloor = 4;

constexpr float dst2_pos_x_offset = 0.15f;

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
    plan.replan_after_box1_place = false;
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

struct ReplanTask {
    const MoveBoxPlan* previous_plan{};
    BoxMoveTask task{};
};

float pick_y(const BoxMoveTask& task)
{
    if (!task.to_box.trajectory.empty()) {
        return task.to_box.trajectory.back()[1];
    }
    return 0.0f;
}

float place_y(const BoxMoveTask& task, float fallback_y)
{
    if (!task.to_dst.trajectory.empty()) {
        return task.to_dst.trajectory.back()[1];
    }
    return fallback_y;
}

int choose_nearest_replan_task(const std::vector<ReplanTask>& tasks, float current_dst2_y)
{
    int best_index = 0;
    float best_error = std::numeric_limits<float>::max();
    for (int index = 0; index < static_cast<int>(tasks.size()); ++index) {
        const float y_error = std::abs(pick_y(tasks[index].task) - current_dst2_y);
        if (y_error < best_error) {
            best_index = index;
            best_error = y_error;
        }
    }
    return best_index;
}

int choose_nearest_replan_task_on_line(const std::vector<ReplanTask>& tasks, int line, float current_dst2_y)
{
    int best_index = -1;
    float best_error = std::numeric_limits<float>::max();
    for (int index = 0; index < static_cast<int>(tasks.size()); ++index) {
        if (tasks[index].task.line != line) {
            continue;
        }
        const float y_error = std::abs(pick_y(tasks[index].task) - current_dst2_y);
        if (y_error < best_error) {
            best_index = index;
            best_error = y_error;
        }
    }
    return best_index;
}

int choose_pair_replan_task(const std::vector<ReplanTask>& tasks, const BoxMoveTask& first_task)
{
    const int pair_line = 1 - first_task.line;
    for (int index = 0; index < static_cast<int>(tasks.size()); ++index) {
        const auto& task = tasks[index].task;
        if (task.line == pair_line && task.col == first_task.col) {
            return index;
        }
    }
    return choose_nearest_replan_task(tasks, pick_y(first_task));
}

bool is_valid_task(const BoxMoveTask& task)
{
    return task.box_id >= 0 && task.box_id < 4;
}

void append_remaining_tasks(std::vector<ReplanTask>& remaining_tasks, const MoveBoxPlan& old_plan)
{
    if (old_plan.plate_retry_plan) {
        if (is_valid_task(old_plan.box0)) {
            remaining_tasks.push_back({&old_plan, old_plan.box0});
        }
        return;
    }

    if (old_plan.hand_only_plan) {
        if (is_valid_task(old_plan.box1)) {
            remaining_tasks.push_back({&old_plan, old_plan.box1});
        }
        return;
    }

    if (is_valid_task(old_plan.box0)) {
        remaining_tasks.push_back({&old_plan, old_plan.box0});
    }
    if (is_valid_task(old_plan.box1)) {
        remaining_tasks.push_back({&old_plan, old_plan.box1});
    }
}

// box0 平板放置失败后重建计划：
// 1) 失败轮之后的剩余箱子改为单吸手放计划，按 pick_line_0 -> pick_line_1 成对排序；
// 2) 末尾追加一轮“回头重试卡住的平板箱”计划（plate_retry）。
// 注意：二层判定不在这里计算，统一由放置时读取黑板 placed_count 决定，
// 这样卡住未成功的箱子不会把同 ID 的后续箱子错误抬到第二层。
std::vector<MoveBoxPlan> make_replan_after_plate_blocked(const std::vector<MoveBoxPlan>& move_plan, int blocked_plan_index)
{
    std::vector<MoveBoxPlan> replan;
    replan.reserve((move_plan.size() - blocked_plan_index) * 2);

    const auto blocked_task = move_plan[blocked_plan_index].box0;
    std::vector<ReplanTask> remaining_tasks;
    remaining_tasks.reserve((move_plan.size() - blocked_plan_index - 1) * 2);
    for (int index = blocked_plan_index + 1; index < static_cast<int>(move_plan.size()); ++index) {
        append_remaining_tasks(remaining_tasks, move_plan[index]);
    }

    float current_dst2_y = move_plan[blocked_plan_index].dst2_pos[1];
    while (!remaining_tasks.empty()) {
        int first_index = choose_nearest_replan_task_on_line(remaining_tasks, 0, current_dst2_y);
        if (first_index < 0) {
            first_index = choose_nearest_replan_task(remaining_tasks, current_dst2_y);
        }
        const auto first_task = remaining_tasks[first_index];
        replan.push_back(make_single_hand_place_plan(*first_task.previous_plan, first_task.task));
        current_dst2_y = place_y(first_task.task, current_dst2_y);
        remaining_tasks.erase(remaining_tasks.begin() + first_index);

        if (remaining_tasks.empty()) {
            break;
        }

        const int pair_index = choose_pair_replan_task(remaining_tasks, first_task.task);
        const auto pair_task = remaining_tasks[pair_index];
        replan.push_back(make_single_hand_place_plan(*pair_task.previous_plan, pair_task.task));
        current_dst2_y = place_y(pair_task.task, current_dst2_y);
        remaining_tasks.erase(remaining_tasks.begin() + pair_index);
    }

    // 末轮重试：直接复用卡住轮计划，box0 用卡住的箱子；只走 box0 平板放置，放完结束。
    MoveBoxPlan retry_plan = move_plan[blocked_plan_index];
    retry_plan.box0 = blocked_task;
    retry_plan.plate_retry_plan = true;
    retry_plan.replan_after_box1_place = false;
    retry_plan.finish_after_box0_place = true;
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
        MoveBoxPlan retreat_plan = plan;
        std::array<std::array<float, 3>, 4> place_table{};
        const int box_id = plan.box1.box_id;
        if (box_id >= 0 && box_id < 4 && tree.read_msg("place_table", place_table)) {
            retreat_plan.dst2_pos = {place_table[box_id][0] - dst2_pos_x_offset, place_table[box_id][1], place_table[box_id][2]};
        }

        const bool final_round = plan_index + 1 >= static_cast<int>(move_plan.size());
        if (!retreat_to_dst2(context, retreat_plan, final_round)) {
            return BT::FAILED;
        }
        if (!final_round) {
            tree.write_msg("plan_index", plan_index + 1);
        }
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 单吸计划跳过 box0 放置阶段，已退让到 dst2");
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

        if (arm_state_ == -1) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂返回 %s 放置失败", slot_name(slot_));
            arm_state_ = 0;
            break;
            //return BT::FAILED;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂放置 %s 超时", slot_name(slot_));
            //return BT::FAILED;
            break;
        }

        std::this_thread::sleep_for(10ms);
    }

    if (!rclcpp::ok()) {
        return BT::FAILED;
    }

    // box1 从手上放置：视为成功，更新二层计数后推进。
    if (slot_ == BoxSlot::Box1) {
        if (has_placed_count && box_id >= 0 && box_id < 4) {
            ++placed_count[box_id];
            tree.write_msg("placed_count", placed_count);
        }
        if (plan.replan_after_box1_place) {
            RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 第一轮 box1 放置结束，跳过 dst1 到 dst0 并提前重规划");
            const auto replan = make_replan_after_plate_blocked(move_plan, plan_index);
            tree.write_msg("move_plan", replan);
            tree.write_msg<int>("plan_index", 0);
            context->tree_start_key = Robot::kTreeArriveToBox0;
            tree.request_restart_from_root();
            RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 已生成 %zu 轮提前重规划计划", replan.size());
            return BT::SUCCESS;
        }
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        return BT::SUCCESS;
    }

    if (slot_ == BoxSlot::Box0) {
        // 放置成功，更新二层计数。
        if (has_placed_count && box_id >= 0 && box_id < 4) {
            ++placed_count[box_id];
            tree.write_msg("placed_count", placed_count);
        }

        // 每轮放完 box0 都退让到 dst2；dst2 用本轮真实 box0 id 重算。
        MoveBoxPlan retreat_plan = plan;
        std::array<std::array<float, 3>, 4> place_table{};
        if (box_id >= 0 && box_id < 4 && tree.read_msg("place_table", place_table)) {
            retreat_plan.dst2_pos = {place_table[box_id][0] - dst2_pos_x_offset, place_table[box_id][1], place_table[box_id][2]};
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
