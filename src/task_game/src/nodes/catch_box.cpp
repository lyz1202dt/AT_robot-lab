#include "nodes/catch_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>

using namespace std::chrono_literals;

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action") {}

BT::Status CatchBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    MoveBoxPlan plan;
    if (!tree.read_msg("active_move_plan", plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 active_move_plan");
        return BT::FAILED;
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "CatchBoxAction: 执行抓取轨迹，轨迹点数量=%zu",
        plan.catch_trajectory.size());
    std::this_thread::sleep_for(100ms);
    return BT::SUCCESS;
}
