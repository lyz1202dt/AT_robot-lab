#include "nodes/place_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}

BT::Status PlaceBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
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

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 执行放置轨迹，轨迹点数量=%zu，目标位置=(%.2f, %.2f)",
        plan.place_trajectory.size(),
        plan.dst_box_pos[0],
        plan.dst_box_pos[1]);
    std::this_thread::sleep_for(100ms);
    std::this_thread::sleep_for(10s);
    if (plan_index + 1 < static_cast<int>(move_plan.size())) {
        tree.write_msg("plan_index", plan_index + 1);
    } else {
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 全部搬箱计划执行完成");
    }
    return BT::SUCCESS;
}
