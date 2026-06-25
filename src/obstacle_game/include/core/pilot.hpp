#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>

class Pilot {
public:
    explicit Pilot(rclcpp::Node::SharedPtr node, const std::string yaml_path);
    ~Pilot();

    //开始执行轨迹
    bool start();

    //暂停执行轨迹
    bool stop();

    //复位轨迹
    bool reset();

    //填写机器人当前状态
    void set_state(const Eigen::Vector2d &pos,const double &yaw);

    //得到当前轨迹的执行状态
    bool get_current_path_info(uint32_t &path_num,float &time_rate);

    //获取当前机器人的速度指令输出
    robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time);

private:
    struct PathPoint {
        int32_t policy_id{0};
        Eigen::Vector2d target_pos{Eigen::Vector2d::Zero()};
        double target_yaw{0.0}; // 机器人轨迹执行完毕后应该指向的方向
        bool constraint_target_yaw{false}; // 是否约束机器人轨迹执行完毕后指向特定的方向
        double target_vel{0.0};  // 到达目标点后应具有的速度
        double max_velocity{0.7};  // 路线最大速度
        double max_accelation{0.25};  // 路线最大加速度
        double max_omega{1.0};  // 自旋最大角速度
        Eigen::Vector3d kp{0.2, 0.2, 0.5};  // 用于闭环控制的Kp参数
        double allow_start_dir_error{0.2};   // 允许起始方向误差
        double allow_final_dir_error{0.2};    // 允许最终方向误差
        double allow_final_pos_allow{0.2};    // 允许最终位置误差
        double adjust_min_vel{0.25};    // 微调时的最小角速度
        double adjust_min_omega{0.15};
        bool allow_y_vel{false};   // 是否允许较大的y向速度（如果为true，则机器人以边平移边旋转的方式运动到目标点，否则就是先瞄准在运动到目标点）
        double trajectory_connection_radius{0.0};   //三次多项式轨迹衔接半径
        bool stand_at_target{false};   // 到达该点后是否进入位控站立
        double stand_duration{0.0};   // 到达该点后的位控站立时间，单位秒
        bool mode_switch_only{false};   // 只切换控制模式，不按目标位姿生成运动轨迹
    };

    enum class PilotState {
        Idle,
        Running,
        Standing,
        Adjusting,
        Paused,
        Finished
    };

    enum class AdjustPhase {
        Position,
        Yaw,
        FinalPosition
    };

    struct CubicTransition {
        bool active{false};
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time{};
        double duration{0.0};
        Eigen::Vector2d start_pos{Eigen::Vector2d::Zero()};
        Eigen::Vector2d end_pos{Eigen::Vector2d::Zero()};
        Eigen::Vector2d start_vel{Eigen::Vector2d::Zero()};
        Eigen::Vector2d end_vel{Eigen::Vector2d::Zero()};
        double start_yaw{0.0};
        double end_yaw{0.0};
    };

    bool load_paths(const std::string &yaml_path);
    void reset_execution();
    void begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed);
    void finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time);
    void advance_after_stand(std::chrono::time_point<std::chrono::high_resolution_clock> time);
    bool should_stand_at_current_target() const;
    robot_msgs::msg::Cmd stand_command() const;
    robot_msgs::msg::Cmd walk_zero_command() const;
    static int32_t policy_id_to_cmd_mode(int32_t policy_id);
    static double normalize_angle(double angle);
    static double clamp_abs(double value, double limit);
    static Eigen::Vector2d world_to_body(const Eigen::Vector2d& vector, double yaw);

    rclcpp::Node::SharedPtr node_;
    mutable std::mutex mutex_;
    std::vector<PathPoint> paths_;

    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    double current_yaw_{0.0};
    bool has_state_{false};

    std::size_t current_path_index_{0};
    PilotState state_{PilotState::Idle};
    PilotState resume_state_{PilotState::Running};
    AdjustPhase adjust_phase_{AdjustPhase::Position};

    Eigen::Vector2d segment_start_pos_{Eigen::Vector2d::Zero()};
    double segment_start_yaw_{0.0};
    double segment_start_speed_{0.0};
    bool aiming_done_{false};
    std::chrono::time_point<std::chrono::high_resolution_clock> segment_start_time_{};
    std::chrono::time_point<std::chrono::high_resolution_clock> stand_start_time_{};

    CubicTransition transition_;
};
