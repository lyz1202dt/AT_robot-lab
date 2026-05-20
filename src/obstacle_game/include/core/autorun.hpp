#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include<unordered_map>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <yaml-cpp/yaml.h>

#include "executer/base_executer.hpp"

class AutoRun {
public:
    AutoRun(rclcpp::Node::SharedPtr node, const std::string yaml_path);
    ~AutoRun() = default;

    //开始执行轨迹
    bool start();

    //暂停执行轨迹
    bool stop();

    //复位并暂停轨迹执行，下次执行从头开始
    bool reset();

    //注册执行器
    void register_executer(std::shared_ptr<BaseExecuter> executer);  //注册执行器

    //在控制循环中调用
    void set_robot_state(const Eigen::Vector2d &pos, const double &yaw);  //识别到的场景ID
    robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time); //得到机器人指令

private:
    struct PathPoint {
        std::string executer_name;
        std::string params;
        Eigen::Vector3d target_pose{Eigen::Vector3d::Zero()};
    };

    bool load_path(const std::string &yaml_path);
    bool load_path_point(const YAML::Node &path_node, std::size_t index);
    bool start_current_target();
    bool prepare_current_target();
    void advance_to_next_target(int success);
    robot_msgs::msg::Cmd make_zero_command() const;
    std::shared_ptr<BaseExecuter> find_executer(const std::string &executer_name) const;

    rclcpp::Node::SharedPtr node_;
    std::unordered_map<std::string, std::shared_ptr<BaseExecuter>> executer_map_;
    std::vector<PathPoint> path_points_;
    std::shared_ptr<BaseExecuter> active_executer_;
    std::size_t current_index_{0};
    bool is_running_{false};
    bool path_loaded_{false};
    bool has_robot_state_{false};
    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    double current_yaw_{0.0};
    int current_scene_id_{0};
};
