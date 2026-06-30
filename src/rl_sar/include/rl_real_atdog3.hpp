/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_ATDOG3_HPP
#define RL_REAL_ATDOG3_HPP

// atdog3（dog3）实机控制入口。
// 与 dog2 的主要区别：
//   - dog3 不使用独立 IMUDriver，姿态直接来自 leg_driver 的 IMU（pack0 含 IMU）。
//   - dog3 为轮足构型：dof>=12 的自由度对应 4 个轮子（SetCommand 单独处理）。
//   - dog3 上层速度限幅 vx/vz 为 ±1.5；dog2 为 ±2.5。
// RL_Real 继承自 RL，按 loop_control/loop_rl/loop_command 三个定时循环运行：
//   读取状态(GetState) -> 状态机推理(StateController) -> 下发电机指令(SetCommand)。

// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "loop.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "leg_driver/leg_driver.hpp"

#include <csignal>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>


class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv,const rclcpp::Node::SharedPtr node);
    ~RL_Real();

private:
    rclcpp::Node::SharedPtr node_;      //用于ROS2订阅通信的节点
    rclcpp::Subscription<robot_msgs::msg::Cmd>::SharedPtr cmd_sub;
    robot_msgs::msg::Cmd remote_cmd;
    // rl functions
    std::vector<float> Forward() override;            // 策略网络前向推理，输出动作
    void GetState(RobotState<float> *state) override; // 读取 IMU/关节状态，并发布 plane_dst
    void SetCommand(const RobotCommand<float> *command) override; // 把策略输出下发到电机/轮子
    void RunModel();      // RL 推理循环体（loop_rl）
    void RobotControl();  // 底层控制循环体（loop_control）：取状态->状态机->下发指令

    // loop
    std::shared_ptr<LoopFunc> loop_command;  // 键盘输入循环
    std::shared_ptr<LoopFunc> loop_control;  // 底层控制循环
    std::shared_ptr<LoopFunc> loop_rl;       // 策略推理循环

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    //real_port
    std::unique_ptr<LegDriver> leg_driver;  // 腿部电机 USB 驱动（含 IMU 与 plane_dst）
    
    // others
    std::vector<float> mapped_joint_positions;
    std::vector<float> mapped_joint_velocities;
};

#endif // RL_REAL_ATDOG3_HPP
