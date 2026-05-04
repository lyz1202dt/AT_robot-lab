#pragma once

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <robot_msgs/msg/remote.hpp>

class Robot{
public:
    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Subscription<robot_msgs::msg::Remote>::SharedPtr remote_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;

};
