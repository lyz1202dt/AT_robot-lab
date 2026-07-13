#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>
#include <robot_msgs/msg/int.hpp>
#include <robot_msgs/msg/remote.hpp>
#include <vector>
#include <core/pilot.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Robot{
public:
    Robot(const std::shared_ptr<rclcpp::Node> node);
    bool check_key_trigger(uint32_t current_key,int index);
    bool check_key_pressed(uint32_t current_key,int index);
private:
    bool refresh_robot_pose();
    bool sync_pilot_state_from_transform(const geometry_msgs::msg::TransformStamped& transfer);
    std::shared_ptr<Pilot> active_pilot() const;
    bool switch_to_path(int path_id);
    void save_key_state(uint32_t current_key);

    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_server_;
    rclcpp::Subscription<robot_msgs::msg::Remote>::SharedPtr remote_sub_;
    rclcpp::Subscription<robot_msgs::msg::Int>::SharedPtr policy_done_sub_;
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr cmd_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::vector<std::shared_ptr<Pilot>> pilots_;
    int active_path_id_{0};
    robot_msgs::msg::Cmd cmd;

    uint32_t last_key{0};
    int current_control_mode{0};
    int manual_switch_request_count_{0};
    static constexpr int kManualSwitchDebounceFrames = 3;
    bool autopilot_available{true};
    bool robot_pose_valid_{false};
    std::atomic_bool stop_target_clear_pending_{false};
    std::atomic_bool start_target_clear_pending_{false};
    std::atomic_bool stand_clear_pending_{false};
    std::atomic_bool begin_game_clear_pending_{false};

    geometry_msgs::msg::TransformStamped robot_pos_transfer;
};
