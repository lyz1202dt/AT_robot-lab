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

}  // namespace

ArriveToTargetAction::ArriveToTargetAction()
    : BT::ActionNode("arrive_to_target_action") {}

BT::Status ArriveToTargetAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (!wait_for_stage(context, Robot::kTreeArriveToTarget)) {
        context->pilot->stop();
        return BT::FAILED;
    }

    if (context->auto_pilot_enabled.load()) {
        context->cmd.mode = 2;
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

    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToTargetAction: 开始执行第 %d 个到达计划, 目标放置位置=(%.2f, %.2f), 轨迹点数量=%zu, second_floor=%s",
        plan_index,
        current_plan.dst_box_pos[0],
        current_plan.dst_box_pos[1],
        current_plan.place_trajectory.size(),
        current_plan.place_at_second_floor ? "true" : "false");

    for (size_t point_index = 0; point_index < current_plan.place_trajectory.size(); ++point_index) {
        const auto& point = current_plan.place_trajectory[point_index];
        const bool is_last = (point_index + 1 == current_plan.place_trajectory.size());

        Pilot::TargetPoint target_point;
        target_point.target_pos = Eigen::Vector2d(point[0], point[1]);
        target_point.target_vel = is_last ? 0.0f : 0.25f;

        // 最后一个轨迹点要求机器人面向放置区，方便后续放箱动作衔接。
        if (is_last) {
            target_point.target_yaw = 0.0f;
            target_point.constraint_target_yaw = true;
        }

        if (!context->pilot->set_target(target_point)) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToTargetAction: 设置第 %zu 个轨迹点失败",
                point_index);
            context->pilot->stop();
            //return BT::FAILED;
        }

        bool finished = false;
        bool success = false;
        if (!context->pilot->start([&finished, &success](int result) {
                success = (result != 0);
                finished = true;
            })) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToTargetAction: 启动第 %zu 个轨迹点失败",
                point_index);
            context->pilot->stop();
            //return BT::FAILED;
        }

        RCLCPP_INFO(
            context->node_->get_logger(),
            "ArriveToTargetAction: 前往轨迹点 %zu -> (%.2f, %.2f, %.2f)",
            point_index,
            point[0],
            point[1],
            point[2]);

        // 等待 Pilot 执行完当前目标点；控制指令由 Robot 的定时器持续调用 get_command 输出。
        while (rclcpp::ok() && context->auto_pilot_enabled.load() && !finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
            context->pilot->stop();
            //return BT::FAILED;
        }

        if (!success) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToTargetAction: 第 %zu 个轨迹点执行失败",
                point_index);
            context->pilot->stop();
            //return BT::FAILED;
        }
    }

    context->pilot->stop();

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    // 到达放置位后，交给后续放箱动作继续处理。
    // 注意：这里不推进 plan_index，索引应由完整完成一次“抓取+放置”后再更新。
    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToTargetAction: 第 %d 个到达计划执行完成，等待后续放置动作",
        plan_index);

    return BT::SUCCESS;
}
