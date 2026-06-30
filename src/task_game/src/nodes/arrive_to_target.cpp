#include "nodes/arrive_to_target.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>
using namespace std::chrono_literals;
namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

int32_t stage_for_slot(BoxSlot slot) {
    return slot == BoxSlot::Box1 ? Robot::kTreeArriveToDst1 : Robot::kTreeArriveToDst0;
}

const BoxMoveTask& task_for_slot(const MoveBoxPlan& plan, BoxSlot slot) {
    return slot == BoxSlot::Box1 ? plan.box1 : plan.box0;
}

const char* slot_name(BoxSlot slot) {
    return slot == BoxSlot::Box1 ? "box1" : "box0";
}

}  // namespace

ArriveToTargetAction::ArriveToTargetAction(BoxSlot slot)
    : BT::ActionNode(slot == BoxSlot::Box1 ? "arrive_to_dst1_action" : "arrive_to_dst0_action"), slot_(slot) {}

void ArriveToTargetAction::ensure_stop_timer(Robot* context) {
    if (stop_timer_) {
        return;
    }

    stop_timer_ = context->node_->create_wall_timer(500ms, [this, context]() {
        context->pilot->enable_stop_when_finished_if_generation_matches(stop_timer_generation_);
        stop_timer_->cancel();
    });
    stop_timer_->cancel();
}

void ArriveToTargetAction::arm_stop_timer(Robot* context) {
    ensure_stop_timer(context);
    stop_timer_generation_ = context->pilot->generation();
    if (!stop_timer_->is_canceled()) {
        stop_timer_->cancel();
    }
    stop_timer_->reset();
}

BT::Status ArriveToTargetAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (!wait_for_stage(context, stage_for_slot(slot_))) {
        context->pilot->stop();
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;
    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: 缺少 plan_index");
        return BT::FAILED;
    }
    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: 缺少 move_plan");
        return BT::FAILED;
    }

    if (move_plan.empty()) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: move_plan 为空");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    const auto& current_plan = move_plan[plan_index];
    // 激光重规划计划的阶段跳过：
    // - 单吸计划(hand_only)的 box0 到放置点阶段跳过（真实箱走 box1 链）；
    // - 末轮平板重试(plate_retry)只走 box0，box1 到放置点阶段跳过。
    if ((slot_ == BoxSlot::Box0 && current_plan.hand_only_plan && !current_plan.plate_retry_plan) ||
        (slot_ == BoxSlot::Box1 && current_plan.plate_retry_plan)) {
        if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
            context->advance_tree_stage();
        }
        RCLCPP_INFO(context->node_->get_logger(), "ArriveToTargetAction: 重规划计划跳过 %s 放置点阶段", slot_name(slot_));
        return BT::SUCCESS;
    }

    const auto& current_task = task_for_slot(current_plan, slot_);
    const auto& trajectory_plan = current_task.to_dst;

    if (trajectory_plan.trajectory.size() != trajectory_plan.target_points.size()) {
        RCLCPP_ERROR(
            context->node_->get_logger(),
            "ArriveToTargetAction: %s 轨迹点数量=%zu 与参数数量=%zu 不一致",
            slot_name(slot_),
            trajectory_plan.trajectory.size(),
            trajectory_plan.target_points.size());
        return BT::FAILED;
    }

    if (trajectory_plan.trajectory.empty()) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: %s to_dst 为空", slot_name(slot_));
        return BT::FAILED;
    }

    std::vector<Pilot::TargetPoint> pilot_targets;
    pilot_targets.reserve(trajectory_plan.trajectory.size());
    for (size_t point_index = 0; point_index < trajectory_plan.trajectory.size(); ++point_index) {
        const auto& point = trajectory_plan.trajectory[point_index];
        const auto& plan_target_point = trajectory_plan.target_points[point_index];

        Pilot::TargetPoint target_point;
        target_point.target_pos = Eigen::Vector2d(point[0], point[1]);
        target_point.target_vel = plan_target_point.target_vel;
        target_point.target_yaw = point[2];
        target_point.constraint_target_yaw = plan_target_point.constraint_target_yaw;
        target_point.max_velocity = plan_target_point.max_velocity;
        target_point.max_accelation = plan_target_point.max_accelation;
        target_point.max_omega = plan_target_point.max_omega;
        target_point.kp = plan_target_point.kp;
        target_point.allow_start_dir_error = plan_target_point.allow_start_dir_error;
        target_point.allow_final_dir_error = plan_target_point.allow_final_dir_error;
        target_point.allow_final_pos_allow = plan_target_point.allow_final_pos_allow;
        target_point.adjust_min_vel = plan_target_point.adjust_min_vel;
        target_point.adjust_min_omega = plan_target_point.adjust_min_omega;
        target_point.allow_y_vel = plan_target_point.allow_y_vel;
        target_point.trajectory_connection_radius = plan_target_point.trajectory_connection_radius;
        pilot_targets.push_back(target_point);
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToTargetAction: 开始执行第 %d 轮 %s 到放置点计划, 目标放置位置=(%.2f, %.2f), 连续轨迹点数量=%zu, second_floor=%s",
        plan_index,
        slot_name(slot_),
        current_task.place_box_pos[0],
        current_task.place_box_pos[1],
        pilot_targets.size(),
        current_task.place_at_second_floor ? "true" : "false");

    if (!context->pilot->set_target(pilot_targets)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: 设置连续轨迹失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    bool finished = false;
    bool success = false;
    if (!context->pilot->start([&finished, &success](int result) {
            success = (result != 0);
            finished = true;
        }, false)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: 启动连续轨迹失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    for (size_t point_index = 0; point_index < trajectory_plan.trajectory.size(); ++point_index) {
        const auto& point = trajectory_plan.trajectory[point_index];
        RCLCPP_INFO(
            context->node_->get_logger(),
            "ArriveToTargetAction: %s 连续轨迹点 %zu -> (%.2f, %.2f, %.2f)",
            slot_name(slot_),
            point_index,
            point[0],
            point[1],
            point[2]);
    }

    while (rclcpp::ok() && context->auto_pilot_enabled.load() && !finished) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
        context->pilot->stop();
        return BT::FAILED;
    }

    if (!success) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToTargetAction: 连续轨迹执行失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToTargetAction: 第 %d 轮 %s 到放置点计划执行完成，等待后续放置动作",
        plan_index,
        slot_name(slot_));

    return BT::SUCCESS;
}
