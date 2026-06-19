/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，将箱子位置和高分区域入栈，切入
*/


#pragma once

#include <array>
#include <memory>
#include <semaphore.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <robot_msgs/msg/int.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include "core/behavior_tree.hpp"

class Robot;

using BoxIdGrid = std::array<std::array<int, 4>, 2>;

class GeneratePlaneAction : public BT::ActionNode {
public:
    GeneratePlaneAction();
    ~GeneratePlaneAction() override;
    void reset_generated();

protected:
    BT::Status execute(BT& tree) override;

private:
    void init_subscriptions(const rclcpp::Node::SharedPtr& node);
    void init_publishers(const rclcpp::Node::SharedPtr& node);

private:
    bool generated{false};
    bool subscriptions_ready_{false};
    bool publishers_ready_{false};
    bool first_run_{true};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<robot_msgs::msg::Int>::SharedPtr vip_box_id_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr box_grid_sub_;

    rclcpp::Publisher<
        std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    int vip_box_id_{-1};
    BoxIdGrid box_id_grid_{};
    sem_t vip_box_id_sem_;
    sem_t box_id_grid_sem_;
};
