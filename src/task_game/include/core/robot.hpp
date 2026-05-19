#pragma once

#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <robot_msgs/msg/remote.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "core/pilot.hpp"
#include "actions/action.hpp"

class Robot{
public:
    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();
    bool check_key_trigger(uint32_t current_key,int index);
    bool check_key_pressed(uint32_t current_key,int index);
    void record_key(uint32_t current_key);
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Subscription<robot_msgs::msg::Remote>::SharedPtr remote_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    //TODO:操作机械臂的话题，相机数据采集的话题等等

    //TF  获取机器人位置
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped robot_pos_transfer;

    std::shared_ptr<Pilot> pilot;
    robot_msgs::msg::Cmd cmd;
    Action<Robot> action;
    std::shared_ptr<std::thread> action_thread;

    uint32_t last_key{0};
    int current_control_mode{0};
    bool autopilot_available{true};
};
