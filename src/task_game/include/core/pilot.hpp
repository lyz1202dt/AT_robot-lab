#pragma once

#include <chrono>
#include <cstddef>
#include <functional>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>

class Pilot {
public:
    struct TargetPoint {
        Eigen::Vector2d target_pos{Eigen::Vector2d::Zero()};
        float target_yaw{0.0f};                // 机器人轨迹执行完毕后应该指向的方向
        bool constraint_target_yaw{false};    // 是否约束机器人轨迹执行完毕后指向特定的方向
        float target_vel{0.0f};               // 到达目标点后应具有的速度
        float max_velocity{0.7f};             // 路线最大速度
        float max_accelation{0.25f};           // 路线最大加速度
        float max_omega{1.0f};                // 自旋最大角速度
        Eigen::Vector3d kp{0.2,0.2,0.5};  // 用于闭环控制的Kp参数
        float allow_start_dir_error{0.2f};    // 允许开始行走时的角度误差
        float allow_final_dir_error{0.2f};    // 允许最终停止运动时的角度误差
        float allow_final_pos_allow{0.2f};    // 允许最终停止时的位置误差
        float adjust_min_vel{0.25f};           // 微调时的最小速度
        float adjust_min_omega{0.15f};         // 微调时的最小角速度
        bool allow_y_vel{false};              // 是否允许较大的y向速度（如果为true，则机器人以边平移边旋转的方式运动到目标点，否则就是先瞄准在运动到目标点）
        float trajectory_connection_radius{0.0};    //三次多项式轨迹衔接半径
    };

    explicit Pilot(rclcpp::Node::SharedPtr node);
    ~Pilot();

    // 开始执行，让机器人行走到目标(到达目标后执行一次finished_cb)
    bool start(std::function<void(int success)> finished_cb);

    // 进入位控站立
    bool stop();

    // 设置机器人系列轨迹点
    bool set_target(const std::vector<TargetPoint> &target);

    //设置机器人单轨迹点
    bool set_target(const TargetPoint &target);

    // 设置机器人当前状态
    void set_state(const Eigen::Vector2d &pos, const float &yaw);

    // 获取当前机器人的速度指令输出
    robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time);

private:
    // Pilot 的内部执行状态：Idle/Paused/Finished 都输出站立零速。
    enum class PilotState {
        Idle,
        Running,
        Adjusting,
        Paused,
        Finished
    };

    // 最终微调顺序：先到点，再原地对准最终方向，最后复检/修正位置漂移。
    enum class AdjustPhase {
        Position,
        Yaw,
        FinalPosition
    };

    // 多段轨迹连接处的三次多项式过渡数据，用于保证位置和速度连续。
    struct CubicTransition {
        bool active{false};
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time{};
        double duration{0.0};
        Eigen::Vector2d start_pos{Eigen::Vector2d::Zero()};
        Eigen::Vector2d end_pos{Eigen::Vector2d::Zero()};
        Eigen::Vector2d start_vel{Eigen::Vector2d::Zero()};
        Eigen::Vector2d end_vel{Eigen::Vector2d::Zero()};
        float start_yaw{0.0f};
        float end_yaw{0.0f};
    };

    static double normalize_angle(double angle);
    static double clamp_abs(double value, double limit);
    static Eigen::Vector2d world_to_body(const Eigen::Vector2d& vector, double yaw);

    robot_msgs::msg::Cmd stand_command() const;
    void reset_execution();
    void begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed);
    void finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time);

    rclcpp::Node::SharedPtr node_;
    // set_state 由控制定时器更新，get_command/start/stop 可能由行为树线程调用。
    mutable std::mutex mutex_;

    // targets_ 保存整条任务轨迹；current_index_ 指向当前正在执行的目标点。
    std::vector<TargetPoint> targets_;
    std::size_t current_index_{0};
    PilotState state_{PilotState::Idle};
    AdjustPhase adjust_phase_{AdjustPhase::Position};
    // stop 后不清空轨迹，resume_state_ 用于再次 start 时恢复到暂停前阶段。
    PilotState resume_state_{PilotState::Running};

    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    float current_yaw_{0.0f};

    // 当前直线段的起点状态，用于按时间采样规划位置/速度。
    Eigen::Vector2d segment_start_pos_{Eigen::Vector2d::Zero()};
    float segment_start_yaw_{0.0f};
    double segment_start_speed_{0.0};
    // allow_y_vel=false 时先瞄准，置 true 后本段不再回到纯瞄准状态。
    bool aiming_done_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> segment_start_time_{};

    CubicTransition transition_;
    std::function<void(int success)> finished_cb_;

};
