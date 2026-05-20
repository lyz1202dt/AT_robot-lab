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

    if(generated)
        return BT::SUCCESS;

    //计移动箱子计划数容器
    std::vector<MoveBoxPlan> move_plan;
    //移动8个箱子，所以应该有8个计划
    move_plan.reserve(8);

    //示例：填写一个计划
    MoveBoxPlan box_plan;
    box_plan.catch_trajectory.push_back({0.0F, 0.0F, 0.0F});
    box_plan.place_trajectory.push_back({1.0F, 0.0F, 0.0F});
    box_plan.src_box_pos = {0.0F, 0.0F};
    box_plan.dst_box_pos = {1.0F, 0.0F};
    box_plan.place_at_second_floor = false;
    move_plan.push_back(box_plan);

    tree.write_msg("move_plan", move_plan);
    tree.write_msg<int>("plan_index", 0);

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 已生成 %zu 条移动计划", move_plan.size());

    generated=true;
    return BT::SUCCESS;
}
