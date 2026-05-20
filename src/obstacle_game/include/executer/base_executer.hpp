#pragma once

#include <chrono>
#include <functional>
#include <string>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>

class BaseExecuter {
public:
   
    explicit BaseExecuter(rclcpp::Node::SharedPtr node)
    {
        (void)node;
    }
    ~BaseExecuter()=default;

    // 开始执行，让机器人行走到目标(到达目标后执行一次finished_cb)
    virtual bool start(std::function<void(int success)> finished_cb)=0;

    // 进入位控站立
    virtual bool stop()=0;

    // 设置机器人目标位姿
    virtual bool set_target(const Eigen::Vector3d &target,std::string &params)=0;

    // 设置机器人当前状态
    virtual void set_state(const Eigen::Vector2d &pos, const float &yaw)=0;

    // 获取当前机器人的速度指令输出
    virtual robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time)=0;

    virtual std::string get_executer_name()=0;
};
