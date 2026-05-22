#include "nodes/catch_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <robot_interfaces/msg/armmode.hpp>

using namespace std::chrono_literals;

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action") {}

BT::Status CatchBoxAction::execute(BT& tree)
{
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }

    // 先清状态
    context->arm_state = 0;

    // 发送抓取命令
    robot_interfaces::msg::Armmode msg;
    msg.mode = 1;
    context->arm_cmd_pub->publish(msg);

    std::this_thread::sleep_for(50ms);

    //robot_interfaces::msg::Armmode msg;
    msg.mode = 0;
    context->arm_cmd_pub->publish(msg);


    RCLCPP_INFO(
        context->node_->get_logger(),
        "等待机械臂抓取完成");

    // 最多等待20秒
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {

        // 成功
        if (context->arm_state == 1) {

            RCLCPP_INFO(
                context->node_->get_logger(),
                "抓取成功");

            break;
        }

        // 失败
        if (context->arm_state == -1) {

            RCLCPP_ERROR(
                context->node_->get_logger(),
                "抓取失败");

            return BT::FAILED;
        }

        // 超时
        auto now = std::chrono::steady_clock::now();

        if (now - start > 20s) {

            RCLCPP_ERROR(
                context->node_->get_logger(),
                "机械臂任务超时");

            return BT::FAILED;
        }

        std::this_thread::sleep_for(50ms);
    }

    return BT::SUCCESS;
}
