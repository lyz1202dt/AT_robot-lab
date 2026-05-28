#pragma once

#include "core/behavior_tree.hpp"

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/armmode.hpp>

class Robot;

class CatchBoxAction : public BT::ActionNode {
public:

    CatchBoxAction();

    void arm_cmd_callback(
        const robot_msgs::msg::Armmode::SharedPtr msg);

protected:

    BT::Status execute(BT& tree) override;

private:

    rclcpp::Publisher<
        robot_msgs::msg::Armmode>::SharedPtr arm_cmd_pub_;

    rclcpp::Subscription<
        robot_msgs::msg::Armmode>::SharedPtr arm_state_sub_;

    std::atomic<int> arm_search_state_{0};

    float x{0.0f}, y{0.0f}, z{0.0f};

    bool ros_initialized_ = false;
};