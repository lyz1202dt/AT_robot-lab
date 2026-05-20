#include "nodes/generate_plan.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

GeneratePlaneAction::GeneratePlaneAction()
    : BT::ActionNode("generate_plan_action") {}

BT::Status GeneratePlaneAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    move_plan.reserve(8);

    MoveBoxPlan box_plan;
    box_plan.catch_trajectory.push_back({0.0F, 0.0F, 0.0F});
    box_plan.place_trajectory.push_back({1.0F, 0.0F, 0.0F});
    box_plan.src_box_pos = {0.0F, 0.0F};
    box_plan.dst_box_pos = {1.0F, 0.0F};
    box_plan.place_at_second_floor = false;
    move_plan.push_back(box_plan);

    tree.write_msg("move_plan", move_plan);
    tree.write_msg("current_plan_index", std::size_t{0});

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 已生成 %zu 条移动计划", move_plan.size());
    std::this_thread::sleep_for(100ms);
    return BT::SUCCESS;
}
