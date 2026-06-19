/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，将箱子位置和高分区域入栈，切入
*/


#pragma once

#include "core/behavior_tree.hpp"
#include <robot_msgs/msg/vis.hpp>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
class Robot;

class PlaceBoxAction : public BT::ActionNode {
public:
    PlaceBoxAction();
    void arm_place_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg);

protected:
    BT::Status execute(BT& tree) override;
    std::array<float, 2> dst_box_pos_;
    bool place_at_second_floor_ = false;

    std::atomic<int> arm_state_{0};

    
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_state_sub;
    bool subscriptions_ready_ = false;
   
};
