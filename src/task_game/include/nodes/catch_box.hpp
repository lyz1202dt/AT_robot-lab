#pragma once

#include "core/behavior_tree.hpp"

#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <robot_interfaces/msg/armmode.hpp>

class Robot;

class CatchBoxAction : public BT::ActionNode {
public:

    CatchBoxAction();

    void arm_cmd_callback(
        const robot_interfaces::msg::Armmode::SharedPtr msg);

protected:

    BT::Status execute(BT& tree) override;

private:

    rclcpp::Publisher<
        robot_interfaces::msg::Armmode>::SharedPtr arm_cmd_pub_;

    rclcpp::Subscription<
        robot_interfaces::msg::Armmode>::SharedPtr arm_state_sub_;

    std::atomic<int> arm_state_{0};

    bool ros_initialized_ = false;
};