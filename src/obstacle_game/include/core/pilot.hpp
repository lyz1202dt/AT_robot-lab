#pragma once

#include <robot_msgs/msg/detail/cmd__struct.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <string>
#include <vector>

class Pilot {
public:
    explicit Pilot(rclcpp::Node::SharedPtr node, const std::string yaml_path);   //需要一个描述多个路径点的yaml文件比如src/obstacle_game/config/path1.yaml
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
        double target_vel{0.0};
        double max_velocity{0.0};
        double max_accelation{0.0};
        Eigen::Vector3d kp{Eigen::Vector3d::Zero()};
        double allow_start_dir_error{0.0};
        double err_allow{0.0};
        double adjust_min_vel{0.0};
    };

    bool load_paths(const std::string &yaml_path);
    void reset_segment_progress();
    bool advance_if_current_target_reached();
    Eigen::Vector2d get_segment_direction(const PathPoint &path) const;
    double compute_feedforward_speed(const PathPoint &path, double distance, double dt);
    robot_msgs::msg::Cmd make_zero_command() const;
    static double normalize_angle(double angle);
    static double clamp_abs(double value, double limit);

    rclcpp::Node::SharedPtr node_;
    std::vector<PathPoint> paths_;

    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    double current_yaw_{0.0};
    bool has_state_{false};

    std::size_t current_path_index_{0};
    bool is_running_{false};
    bool is_finished_{false};

    Eigen::Vector2d segment_start_pos_{Eigen::Vector2d::Zero()};
    double segment_start_distance_{0.0};
    bool segment_initialized_{false};

    double current_linear_speed_{0.0};
    std::chrono::time_point<std::chrono::high_resolution_clock> last_command_time_{};
    bool first_run{false};
};
