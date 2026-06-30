#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <array>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <robot_msgs/msg/int.hpp>
#include <robot_msgs/msg/remote.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include "core/pilot.hpp"
#include "core/behavior_tree.hpp"
#include "nodes/msg.hpp"

class Robot{
public:
    static constexpr int32_t kTreeIdle = -2;
    static constexpr int32_t kTreeGeneratePlan = -1;
    static constexpr int32_t kTreeArriveToBox0 = 0;
    static constexpr int32_t kTreeCatchBox0 = 1;
    static constexpr int32_t kTreeArriveToBox1 = 2;
    static constexpr int32_t kTreeCatchBox1 = 3;
    static constexpr int32_t kTreeArriveToDst1 = 4;
    static constexpr int32_t kTreePlaceBox1 = 5;
    static constexpr int32_t kTreeArriveToDst0 = 6;
    static constexpr int32_t kTreePlaceBox0 = 7;

    Robot(const std::shared_ptr<rclcpp::Node> node);
    ~Robot();
    bool check_key_trigger(uint32_t current_key,int index);
    bool check_key_pressed(uint32_t current_key,int index);
    void record_key(uint32_t current_key);
    void advance_tree_stage();
    void enter_manual_mode();
    void set_tree_debug_mode(bool enabled);
    bool is_tree_debug_mode() const;

    bool start_game{false};
    std::shared_ptr<Pilot> pilot;
    robot_msgs::msg::Cmd cmd;
    rclcpp::Node::SharedPtr node_;
    BT bt;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::atomic_bool auto_pilot_enabled{false};
    std::atomic_int32_t tree_start_key{kTreeIdle};
    std::atomic_bool tree_debug_mode{false};
    // 激光测距结果缓存：plane_dst_buffer_ 为最近一次 plane_dst 话题值，
    // plane_dst_received_ 表示是否至少收到过一次。
    std::atomic_int32_t plane_dst_buffer_{0};
    std::atomic_bool plane_dst_received_{false};
    // enable_plane_dst_replan_：是否启用基于激光测距的失败重规划（真机用）。
    std::atomic_bool enable_plane_dst_replan_{true};
    // debug_force_replan_：调试用，一次性强制触发一次重规划，触发后自动复位。
    std::atomic_bool debug_force_replan_{false};
private:
    bool is_position_out_of_bounds(const geometry_msgs::msg::TransformStamped& transfer) const;
    void stop_rl_real_nodes();
    void publish_active_box_target_tf();

    rclcpp::TimerBase::SharedPtr control_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> target_box_tf_broadcaster_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Subscription<robot_msgs::msg::Remote>::SharedPtr remote_sub_;
    rclcpp::Subscription<robot_msgs::msg::Int>::SharedPtr plane_dst_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped robot_pos_transfer;
    std::array<double, 2> transfer_x_limits_{{-0.5, 6.5}};
    std::array<double, 2> transfer_y_limits_{{-4.2, 0.2}};
    std::array<double, 2> transfer_z_limits_{{-1.0, 1.0}};
    int out_of_bounds_frames_{0};
    static constexpr int kOutOfBoundsStopFrames = 2;
    std::atomic_bool rl_real_stop_requested_{false};

    std::shared_ptr<std::thread> action_thread;

    uint32_t last_key{0};
    int current_control_mode{0};
    bool autopilot_available{true};
    int reconnect_ignore_frames_{0};
    static constexpr int kReconnectIgnoreFrames = 10;
};
