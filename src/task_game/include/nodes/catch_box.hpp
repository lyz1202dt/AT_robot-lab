#pragma once

#include "core/behavior_tree.hpp"
#include "nodes/msg.hpp"

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
class Robot;

class CatchBoxAction : public BT::ActionNode {
public:
    // box0 抓取后放到平板，box1 抓取后保持吸在机械臂上。
    explicit CatchBoxAction(BoxSlot slot);

    void arm_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg);

protected:
    BT::Status execute(BT& tree) override;

private:
    BoxSlot slot_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_state_sub_;
    std::atomic<int> arm_state_{0};
    bool subscriptions_ready_ = false;
    bool ros_initialized_ = false;
};
