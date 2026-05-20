#include "nodes/place_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>

using namespace std::chrono_literals;

PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}

BT::Status PlaceBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    MoveBoxPlan plan;
    std::size_t current_plan_index = 0;
    if (!tree.read_msg("active_move_plan", plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 active_move_plan");
        return BT::FAILED;
    }

    tree.read_msg("current_plan_index", current_plan_index);
    tree.write_msg("current_plan_index", current_plan_index + 1);

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 执行放置轨迹，轨迹点数量=%zu，目标位置=(%.2f, %.2f)",
        plan.place_trajectory.size(),
        plan.dst_box_pos[0],
        plan.dst_box_pos[1]);
    std::this_thread::sleep_for(100ms);
    return BT::SUCCESS;
}
