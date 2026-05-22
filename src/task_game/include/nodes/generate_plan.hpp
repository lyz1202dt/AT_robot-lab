/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，将箱子位置和高分区域入栈，切入
*/


#pragma once

#include <array>
#include <memory>
#include <semaphore.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_msgs/msg/box_id_grid.hpp>
#include <robot_msgs/msg/int.hpp>

#include "core/behavior_tree.hpp"

class Robot;

using BoxIdGrid = std::array<std::array<int, 4>, 2>;

class GeneratePlaneAction : public BT::ActionNode {
public:
    GeneratePlaneAction();
    ~GeneratePlaneAction() override;

protected:
    BT::Status execute(BT& tree) override;

private:
    void init_subscriptions(const rclcpp::Node::SharedPtr& node);

private:
    bool generated{false};
    bool subscriptions_ready_{false};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_msgs::msg::Int>::SharedPtr vip_box_id_sub_;
    rclcpp::Subscription<robot_msgs::msg::BoxIdGrid>::SharedPtr box_id_grid_sub_;
    int vip_box_id_{-1};
    BoxIdGrid box_id_grid_{};
    sem_t vip_box_id_sem_;
    sem_t box_id_grid_sem_;
};
