#include "nodes/handle_plan.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>

using namespace std::chrono_literals;

HandlePlaneAction::HandlePlaneAction()
    : BT::ActionNode("move_box_action") {}

BT::Status HandlePlaneAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> plan;
    std::size_t current_plan_index = 0;
    if (!tree.read_msg("move_plan", plan) || plan.empty()) {
        RCLCPP_WARN(context->node_->get_logger(), "HandlePlaneAction: move_plan 不存在或为空");
        return BT::FAILED;
    }
    tree.read_msg("current_plan_index", current_plan_index);

    if (current_plan_index >= plan.size()) {
        RCLCPP_WARN(
            context->node_->get_logger(),
            "HandlePlaneAction: current_plan_index=%zu 超出计划范围 %zu",
            current_plan_index,
            plan.size());
        return BT::FAILED;
    }

    tree.write_msg("active_move_plan", plan[current_plan_index]);

    RCLCPP_INFO(context->node_->get_logger(), "HandlePlaneAction: 已装载第 %zu 条计划", current_plan_index);
    std::this_thread::sleep_for(100ms);
    return BT::SUCCESS;
}
