#include "nodes/arrive_to_box.hpp"
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

}  // namespace

ArriveToBoxAction::ArriveToBoxAction()
    : BT::ActionNode("arrive_to_box_action") {}

void ArriveToBoxAction::ensure_stop_timer(Robot* context) {
    if (stop_timer_) {
        return;
    }

    stop_timer_ = context->node_->create_wall_timer(500ms, [this, context]() {
        context->pilot->stop_if_generation_matches(stop_timer_generation_);
        stop_timer_->cancel();
    });
    stop_timer_->cancel();
}

void ArriveToBoxAction::arm_stop_timer(Robot* context) {
    ensure_stop_timer(context);
    stop_timer_generation_ = context->pilot->generation();
    if (!stop_timer_->is_canceled()) {
        stop_timer_->cancel();
    }
    stop_timer_->reset();
}

BT::Status ArriveToBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (!wait_for_stage(context, Robot::kTreeArriveToBox)) {
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
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: move_plan 为空");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    const auto& current_plan = move_plan[plan_index];
    const size_t expected_target_point_count =
        current_plan.catch_trajectory.size() + current_plan.place_trajectory.size();
    if (current_plan.target_point.size() != expected_target_point_count) {
        RCLCPP_ERROR(
            context->node_->get_logger(),
            "ArriveToBoxAction: target_point 数量=%zu 与轨迹点总数=%zu 不一致",
            current_plan.target_point.size(),
            expected_target_point_count);
        return BT::FAILED;
    }

    if (current_plan.catch_trajectory.empty()) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: catch_trajectory 为空");
        return BT::FAILED;
    }

    std::vector<Pilot::TargetPoint> pilot_targets;
    pilot_targets.reserve(current_plan.catch_trajectory.size());
    for (size_t point_index = 0; point_index < current_plan.catch_trajectory.size(); ++point_index) {
        if (point_index >= current_plan.target_point.size()) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToBoxAction: 第 %zu 个轨迹点缺少 target_point 参数",
                point_index);
            context->pilot->stop();
            return BT::FAILED;
        }

        const auto& point = current_plan.catch_trajectory[point_index];
        const auto& plan_target_point = current_plan.target_point[point_index];

        Pilot::TargetPoint target_point;
        target_point.target_pos = Eigen::Vector2d(point[0], point[1]);
        target_point.target_yaw = point[2];
        target_point.constraint_target_yaw = plan_target_point.constraint_target_yaw;
        target_point.target_vel = plan_target_point.target_vel;
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
        "ArriveToBoxAction: 开始执行第 %d 个到达计划, 目标箱子位置=(%.2f, %.2f), 连续轨迹点数量=%zu",
        plan_index,
        current_plan.src_box_pos[0],
        current_plan.src_box_pos[1],
        pilot_targets.size());

    if (!context->pilot->set_target(pilot_targets)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: 设置连续轨迹失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    bool finished = false;
    bool success = false;
    if (!context->pilot->start([&finished, &success](int result) {
            success = (result != 0);
            finished = true;
        }, false)) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: 启动连续轨迹失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    for (size_t point_index = 0; point_index < current_plan.catch_trajectory.size(); ++point_index) {
        const auto& point = current_plan.catch_trajectory[point_index];
        RCLCPP_INFO(
            context->node_->get_logger(),
            "ArriveToBoxAction: 连续轨迹点 %zu -> (%.2f, %.2f, %.2f)",
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
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: 连续轨迹执行失败");
        context->pilot->stop();
        return BT::FAILED;
    }

    arm_stop_timer(context);

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToBoxAction: 第 %d 个到达计划执行完成，等待后续抓取动作",
        plan_index);

    return BT::SUCCESS;
}
