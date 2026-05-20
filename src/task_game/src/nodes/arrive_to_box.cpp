#include "nodes/arrive_to_box.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <rclcpp/logging.hpp>
#include <thread>

using namespace std::chrono_literals;

ArriveToBoxAction::ArriveToBoxAction()
    : BT::ActionNode("catch_box_action") {}

BT::Status ArriveToBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    std::vector<MoveBoxPlan> move_plan;
    int plan_index;
    tree.read_msg("plan_index",plan_index);
    tree.read_msg("move_plan", move_plan);

    //TODO:填写并顺次执行所有轨迹点，等待轨迹点完成完成
    //context->pilot->set_target(const TargetPoint &target)

    return BT::SUCCESS;
}
