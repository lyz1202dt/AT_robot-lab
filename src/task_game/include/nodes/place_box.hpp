/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，将箱子位置和高分区域入栈，切入
*/


#pragma once

#include "core/behavior_tree.hpp"
#include <robot_msgs/msg/vis.hpp>

class Robot;

class PlaceBoxAction : public BT::ActionNode {
public:
    PlaceBoxAction();
    void arm_place_cmd_callback(const robot_msgs::msg::Armmode::SharedPtr msg);

protected:
    BT::Status execute(BT& tree) override;
    std::array<float, 2> dst_box_pos_;
    bool place_at_second_floor_ = false;

    std::atomic<int> arm_state_{0};

    rclcpp::Publisher<robot_msgs::msg::Vis>::SharedPtr place_pos_up_pub;
    rclcpp::Publisher<robot_msgs::msg::Vis>::SharedPtr place_pos_down_pub;
    rclcpp::Publisher<robot_msgs::msg::Armmode>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<robot_msgs::msg::Armmode>::SharedPtr arm_state_sub;

    const std::array<float, 2> kBox1Pos = {1.0f, 2.0f};
    const std::array<float, 2> kBox2Pos = {2.0f, 2.0f};
    const std::array<float, 2> kBox3Pos = {3.0f, 2.0f};
    const std::array<float, 2> kBox4Pos = {4.0f, 2.0f};
};
