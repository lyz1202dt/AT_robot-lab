#include "nodes/catch_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action") {}

BT::Status CatchBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;
    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }
    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }
    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    const auto& plan = move_plan[plan_index];
    RCLCPP_INFO(
        context->node_->get_logger(),
        "CatchBoxAction: 执行抓取轨迹，轨迹点数量=%zu",
        plan.catch_trajectory.size());
    std::this_thread::sleep_for(100ms);
    std::this_thread::sleep_for(10s);
    return BT::SUCCESS;
}
