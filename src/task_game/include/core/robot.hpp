#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <array>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <robot_msgs/msg/remote.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "core/pilot.hpp"
#include "core/behavior_tree.hpp"

class Robot{
public:
    static constexpr int32_t kTreeIdle = -2;
    static constexpr int32_t kTreeGeneratePlan = -1;
    static constexpr int32_t kTreeArriveToBox = 0;
    static constexpr int32_t kTreeCatchBox = 1;
    static constexpr int32_t kTreeArriveToTarget = 2;
    static constexpr int32_t kTreePlaceBox = 3;

    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();
    bool check_key_trigger(uint32_t current_key,int index);
    bool check_key_pressed(uint32_t current_key,int index);
    void record_key(uint32_t current_key);
    void advance_tree_stage();
    void set_tree_debug_mode(bool enabled);
    bool is_tree_debug_mode() const;

    std::shared_ptr<Pilot> pilot;
    robot_msgs::msg::Cmd cmd;
    rclcpp::Node::SharedPtr node_;
    BT bt;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::atomic_bool auto_pilot_enabled{false};
    std::atomic_int32_t tree_start_key{kTreeIdle};
    std::atomic_bool tree_debug_mode{true};
private:
    bool is_position_out_of_bounds(const geometry_msgs::msg::TransformStamped& transfer) const;
    void stop_rl_real_nodes();

    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Subscription<robot_msgs::msg::Remote>::SharedPtr remote_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    //TODO:操作机械臂的话题，相机数据采集的话题等等

    //TF  获取机器人位置
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped robot_pos_transfer;
    std::array<double, 2> transfer_x_limits_{{-0.5, 6.5}};
    std::array<double, 2> transfer_y_limits_{{-4.2, 0.2}};
    std::array<double, 2> transfer_z_limits_{{-0.5, 0.5}};
    std::atomic_bool rl_real_stop_requested_{false};



    std::shared_ptr<std::thread> action_thread;

    uint32_t last_key{0};
    int current_control_mode{0};
    bool autopilot_available{true};
    int reconnect_ignore_frames_{0};
    static constexpr int kReconnectIgnoreFrames = 10;
};
