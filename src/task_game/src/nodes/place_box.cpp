#include "nodes/place_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() &&
           context->auto_pilot_enabled.load() &&
           context->tree_start_key.load() != expected_stage) {
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


PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}


BT::Status PlaceBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (!wait_for_stage(context, Robot::kTreePlaceBox)) {
        return BT::FAILED;
    }

    if (context->auto_pilot_enabled.load()) {
        context->cmd.mode = 1;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;

    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(),
                    "PlaceBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }

    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(),
                    "PlaceBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }

    if (plan_index < 0 ||
        plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(),
                    "PlaceBoxAction: plan_index=%d 越界",
                    plan_index);
        return BT::FAILED;
    }

    const auto& plan = move_plan[plan_index];

    // 提取保存到当前类成员变量
    dst_box_pos_ = plan.dst_box_pos;
    place_at_second_floor_ = plan.place_at_second_floor;

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 提取目标位置=(%.2f, %.2f), second_floor=%s",
        dst_box_pos_[0],
        dst_box_pos_[1],
        place_at_second_floor_ ? "true" : "false"
    );

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 执行放置轨迹，轨迹点数量=%zu，目标位置=(%.2f, %.2f)",
        plan.place_trajectory.size(),
        plan.dst_box_pos[0],
        plan.dst_box_pos[1]
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!wait_with_interrupt(context, std::chrono::seconds(10))) {
        return BT::FAILED;
    }

    if (plan_index + 1 < static_cast<int>(move_plan.size())) {
        tree.write_msg("plan_index", plan_index + 1);
    } else {
        RCLCPP_INFO(
            context->node_->get_logger(),
            "PlaceBoxAction: 全部搬箱计划执行完成"
        );
    }

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    return BT::SUCCESS;
}