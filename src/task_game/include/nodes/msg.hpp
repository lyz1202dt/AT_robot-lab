#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>

struct TargetPoint {
        bool constraint_target_yaw{false};    // 是否约束机器人轨迹执行完毕后指向特定的方向
        float target_vel{0.0f};               // 到达目标点后应具有的速度
        float max_velocity{0.7f};             // 路线最大速度
        float max_accelation{0.25f};           // 路线最大加速度
        float max_omega{1.0f};                // 自旋最大角速度
        Eigen::Vector3d kp{0.2,0.2,0.5};  // 用于闭环控制的Kp参数
        float allow_start_dir_error{0.2f};    // 允许开始行走时的角度误差
        float allow_final_dir_error{0.2f};    // 允许最终角度误差
        float allow_final_pos_allow{0.2f};    // 允许最终位置误差
        float adjust_min_vel{0.25f};           // 微调时的最小速度
        float adjust_min_omega{0.15f};         // 微调时的最小角速度
        bool allow_y_vel{false};              // 是否允许较大的y向速度
        float trajectory_connection_radius{0.0f};  // 轨迹衔接半径，0 表示不使用曲线连接
    };

struct MoveBoxPlan{
    std::vector<std::array<float,3>> catch_trajectory;    //轨迹点序列（抓）
    std::vector<std::array<float,3>> place_trajectory;    //轨迹点序列（放）
    std::array<float,2> src_box_pos;  //抓取的箱子位置
    std::array<float,2> dst_box_pos;  //放置的箱子位置
    bool place_at_second_floor;  //是不是要放置在第二层
    std::vector<TargetPoint> target_point;    //目标点参数
};