#pragma once

#include "core/behavior_tree.hpp"
#include "nodes/msg.hpp"

#include <atomic>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class Robot;

class CatchBoxAction : public BT::ActionNode {
public:
    CatchBoxAction();

    void arm_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg);
    // pnp_box_index 回调：抓取窗口内收到被抓箱子的放置区 ID（first-wins 锁存）。
    void pnp_box_index_callback(const std_msgs::msg::Int32::SharedPtr msg);

protected:
    BT::Status execute(BT& tree) override;

private:
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pnp_box_index_sub_;
    std::atomic<int> arm_state_{0};
    // pnp_box_index 锁存：每次抓取开始前清零，回调首条消息写入。
    std::mutex pnp_mutex_;
    int pnp_value_{-1};
    bool pnp_latched_{false};
    bool subscriptions_ready_ = false;
    bool ros_initialized_ = false;
};
