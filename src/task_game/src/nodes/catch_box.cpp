#include "nodes/catch_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
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

bool wait_with_interrupt(Robot* context, const std::chrono::milliseconds duration) {
    auto remaining = duration;
    while (remaining.count() > 0) {
        if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
            return false;
        }
        const auto step = std::min(remaining, std::chrono::milliseconds(50));
        std::this_thread::sleep_for(step);
        remaining -= step;
    }

    return true;
}

}  // namespace

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action") {}

BT::Status CatchBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (!wait_for_stage(context, Robot::kTreeCatchBox)) {
        return BT::FAILED;
    }

    if (context->auto_pilot_enabled.load()) {
        context->cmd.mode = 1;
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
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!wait_with_interrupt(context, std::chrono::seconds(10))) {
        return BT::FAILED;
    }
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }
    return BT::SUCCESS;
}
