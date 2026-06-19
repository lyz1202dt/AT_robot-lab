#ifndef __SERIALNODE_HPP__
#define __SERIALNODE_HPP__

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "cdc_trans.hpp"
#include <robot_msgs/msg/arm.hpp>
#include <robot_msgs/msg/arm4.hpp>
#include <robot_msgs/msg/int.hpp>
#include "data_pack.h"
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"





class ArmNode : public rclcpp::Node
{
public:
    ArmNode();
    ~ArmNode();
private:
    bool exit_thread;
    bool first_update{true};
    int state_log_print_cnt{0};
    int target_log_print_cnt{0};
    int state_log_update_cnt{50};
    int target_log_update_cnt{50};
    bool enable_control{false};
    void armSubscribCb(const robot_msgs::msg::Arm& msg);
    void airSubscribCb(const std_msgs::msg::Int32& msg);
    void publishredState(const state_pack_t *arm_state);
    
    int air_pump;
    std::unique_ptr<CDCTrans> cdc_trans;
    std::unique_ptr<std::thread> usb_event_handle_thread;
    target_pack_t arm_target;
    state_pack_t arm_state;
    rclcpp::Publisher<robot_msgs::msg::Int>::SharedPtr red_pub;
    rclcpp::Subscription<robot_msgs::msg::Arm>::SharedPtr arm_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr air_sub;
    
    OnSetParametersCallbackHandle::SharedPtr param_server_;
    

    rclcpp::Time base_time;
    bool runned{false};
};

#endif
