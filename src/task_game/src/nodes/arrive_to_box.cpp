#include "nodes/arrive_to_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"

#include <cmath>
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

ArriveToBoxAction::ArriveToBoxAction()
    : BT::ActionNode("arrive_to_box_action") {}

BT::Status ArriveToBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
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

    // 计划为空时直接失败，说明上游规划节点还未正确生成搬运任务。
    if (move_plan.empty()) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: move_plan 为空");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_ERROR(context->node_->get_logger(), "ArriveToBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    const auto& current_plan = move_plan[plan_index];

    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToBoxAction: 开始执行第 %d 个到达计划, 目标箱子位置=(%.2f, %.2f), 轨迹点数量=%zu",
        plan_index,
        current_plan.src_box_pos[0],
        current_plan.src_box_pos[1],
        current_plan.catch_trajectory.size());

    for (size_t point_index = 0; point_index < current_plan.catch_trajectory.size(); ++point_index) {
        const auto& point = current_plan.catch_trajectory[point_index];
        const bool is_last = (point_index + 1 == current_plan.catch_trajectory.size());

        Pilot::TargetPoint target_point;
        target_point.target_pos = Eigen::Vector2d(point[0], point[1]);
        target_point.target_vel = is_last ? 0.0f : 0.25f;

        // 最后一个轨迹点要求机器人面向箱子，方便后续抓取动作衔接。
        if (is_last) {
            target_point.target_yaw = -static_cast<float>(M_PI);
            target_point.constraint_target_yaw = true;
        }

        if (!context->pilot->set_target(target_point)) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToBoxAction: 设置第 %zu 个轨迹点失败",
                point_index);
            context->pilot->stop();
            return BT::FAILED;
        }

        bool finished = false;
        bool success = false;
        if (!context->pilot->start([&finished, &success](int result) {
                success = (result != 0);
                finished = true;
            })) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToBoxAction: 启动第 %zu 个轨迹点失败",
                point_index);
            context->pilot->stop();
            return BT::FAILED;
        }

        RCLCPP_INFO(
            context->node_->get_logger(),
            "ArriveToBoxAction: 前往轨迹点 %zu -> (%.2f, %.2f, %.2f)",
            point_index,
            point[0],
            point[1],
            point[2]);

        // 等待 Pilot 执行完当前目标点；控制指令由 Robot 的定时器持续调用 get_command 输出。
        while (rclcpp::ok() && !finished) {
            std::this_thread::sleep_for(50ms);
        }

        if (!rclcpp::ok()) {
            context->pilot->stop();
            return BT::FAILED;
        }

        if (!success) {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "ArriveToBoxAction: 第 %zu 个轨迹点执行失败",
                point_index);
            context->pilot->stop();
            return BT::FAILED;
        }
    }

    context->pilot->stop();

    // 到达抓取位后，交给后续抓箱动作继续处理。
    // 注意：这里不推进 plan_index，索引应由完整完成一次“抓取+放置”后再更新，
    // 否则会导致后续节点读取到错误的当前计划。
    RCLCPP_INFO(
        context->node_->get_logger(),
        "ArriveToBoxAction: 第 %d 个到达计划执行完成，等待后续抓取动作",
        plan_index);

    return BT::SUCCESS;
}
