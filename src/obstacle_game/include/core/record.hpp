#pragma once


#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class Record {
public:

    Record(rclcpp::Node::SharedPtr node);

    // 设置接下来要记录路径的文件
    bool set_output_yaml(const std::string file_path);

    // 记录位置
    bool record_pos(const Eigen::Vector3d &target_info);

    // 完成记录并关闭当前文件
    int finishe_record();

private:
    void close_current_file();
    void write_yaml_header();
    void write_path_point(const Eigen::Vector3d &target_info);

    rclcpp::Node::SharedPtr node_;
    std::ofstream yaml_out_;
    std::string output_file_path_;
    int recorded_count_{0};
};
