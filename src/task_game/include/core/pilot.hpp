#pragma once

#include <chrono>
#include <functional>

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
        float allow_final_dir_error{0.2f};    // 允许最终角度误差
        float allow_final_pos_allow{0.2f};    // 允许最终位置误差
        float adjust_min_vel{0.25f};           // 微调时的最小速度
        float adjust_min_omega{0.15f};         // 微调时的最小角速度
        bool allow_y_vel{false};              // 是否允许较大的y向速度
    };

    explicit Pilot(rclcpp::Node::SharedPtr node);
    ~Pilot();

    // 开始执行，让机器人行走到目标(到达目标后执行一次finished_cb)
    bool start(std::function<void(int success)> finished_cb);

    // 进入位控站立
    bool stop();

    // 设置机器人目标位姿
    bool set_target(const TargetPoint &target);

    // 设置机器人当前状态
    void set_state(const Eigen::Vector2d &pos, const float &yaw);

    // 获取当前机器人的速度指令输出
    robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time);

private:
    enum class ControlPhase {
        kIdle,
        kMoveToPosition,
        kAlignFinalYaw
    };

    bool target_available() const;
    bool target_config_valid(const TargetPoint &target) const;
    bool is_position_reached() const;
    bool is_yaw_reached() const;
    void finish_current_task(int success);
    double compute_limited_linear_speed(double distance, double dt);
    double compute_limited_omega(double yaw_error) const;
    bool in_position_adjust_window(double distance) const;
    void limit_body_velocity(Eigen::Vector2d &body_vel) const;
    void apply_min_adjust_linear_speed(Eigen::Vector2d &body_vel, double distance) const;
    void apply_output_axis_limits(robot_msgs::msg::Cmd &cmd) const;
    robot_msgs::msg::Cmd make_zero_command() const;
    static double normalize_angle(double angle);
    static double clamp_abs(double value, double limit);

    rclcpp::Node::SharedPtr node_;

    TargetPoint target_{};
    bool has_target_{false};

    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    double current_yaw_{0.0};
    bool has_state_{false};

    bool is_running_{false};
    bool first_run_{false};
    ControlPhase phase_{ControlPhase::kIdle};

    double current_linear_speed_{0.0};
    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_time_{};

    std::function<void(int success)> finished_cb_{};
};
