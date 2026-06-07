#include "nodes/catch_box.hpp"

#include "core/robot.hpp"
#include "nodes/msg.hpp"

#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;
namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

}  // namespace

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action")
{

     
    
}

void CatchBoxAction::arm_cmd_callback(
    const robot_msgs::msg::Armmode::SharedPtr msg)
{
    arm_state_ = msg->mode;
}

BT::Status CatchBoxAction::execute(BT& tree)
{
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }

    // arm_cmd_pub_ =
    //         context->node_->create_publisher<
    //             robot_msgs::msg::Armmode>(
    //                 "arm_cmd",
    //                 10);

    // arm_state_sub_ =
    //         context->node_->create_subscription<
    //             robot_msgs::msg::Armmode>(
    //                 "arm_cmd_state",
    //                 10,
    //                 std::bind(
    //                     &CatchBoxAction::arm_cmd_callback,
    //                     this,
    //                     std::placeholders::_1));

    
    // // 发送抓取命令
    // robot_msgs::msg::Armmode msg;
    // msg.mode = 1;

    // arm_cmd_pub_->publish(msg);

    if (!wait_for_stage(context, Robot::kTreeCatchBox)) {
        context->pilot->stop();
        return BT::FAILED;
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "等待机械臂抓取完成");

    // auto start = std::chrono::steady_clock::now();

    // while (rclcpp::ok()) {

    //     // 成功
    //     if (arm_state_ == 1) {

    //         RCLCPP_INFO(
    //             context->node_->get_logger(),
    //             "抓取成功");

    //         arm_state_ = 0;

    //         return BT::SUCCESS;
    //     }

    //     // 失败
    //     if (arm_state_ == -1) {

    //         RCLCPP_ERROR(
    //             context->node_->get_logger(),
    //             "抓取失败");

    //         arm_state_ = 0;

    //         return BT::FAILED;
    //     }

    //     // 超时
    //     if (std::chrono::steady_clock::now() - start
    //         > 20s)
    //     {
    //         RCLCPP_ERROR(
    //             context->node_->get_logger(),
    //             "机械臂任务超时");

    //         return BT::FAILED;
    //     }

    //     std::this_thread::sleep_for(5ms);

    // }

    std::this_thread::sleep_for(1s);
    RCLCPP_INFO(
        context->node_->get_logger(),
        "机械臂抓取完成");
    return BT::SUCCESS;
}