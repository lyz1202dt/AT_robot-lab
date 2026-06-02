#include "nodes/lift_search.hpp"

#include "core/robot.hpp"
#include "nodes/msg.hpp"

#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;

LiftSearchAction::LiftSearchAction()
    : BT::ActionNode("lift_search_action")
{

     
    
}

void LiftSearchAction::arm_cmd_callback(
    const robot_msgs::msg::Armmode::SharedPtr msg)
{
    arm_search_state_ = msg->mode;
}

BT::Status LiftSearchAction::execute(BT& tree)
{
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }
    arm_cmd_pub_ =
            context->node_->create_publisher<
                robot_msgs::msg::Armmode>(
                    "arm_cmd",
                    10);

    arm_state_sub_ =
            context->node_->create_subscription<
                robot_msgs::msg::Armmode>(
                    "arm_search_state",
                    10,
                    std::bind(
                        &LiftSearchAction::arm_cmd_callback,
                        this,
                        std::placeholders::_1));

    
    // 发送抓取命令
    robot_msgs::msg::Armmode msg;
    msg.mode = 4;

    arm_cmd_pub_->publish(msg);

    RCLCPP_INFO(
        context->node_->get_logger(),
        "等待机械臂抓取完成");

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {

        // 成功
        if (arm_search_state_ == 1) {

            RCLCPP_INFO(
                context->node_->get_logger(),
                "抓取成功");

            arm_search_state_ = 0;

            return BT::SUCCESS;
        }

        // 失败
        if (arm_search_state_ == -1) {

            RCLCPP_ERROR(
                context->node_->get_logger(),
                "抓取失败");

            arm_search_state_ = 0;

            return BT::FAILED;
        }

        // 超时
        if (std::chrono::steady_clock::now() - start
            > 20s)
        {
            RCLCPP_ERROR(
                context->node_->get_logger(),
                "机械臂任务超时");

            return BT::FAILED;
        }

        std::this_thread::sleep_for(5ms);
    }

    return BT::FAILED;
}