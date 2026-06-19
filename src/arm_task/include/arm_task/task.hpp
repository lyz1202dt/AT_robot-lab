#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/vis.hpp>
#include <robot_msgs/msg/int.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <vector>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>


namespace arm_task {

class ArmTaskNode : public rclcpp::Node {
public:
    explicit ArmTaskNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ArmTaskNode();

private:

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Task execution thread
    void task_execution_thread();

    // State machine for task execution
    void execute_task_state_machine();
    void execute_grasp_flow();
    void execute_place_flow_1();
    void execute_place_flow_2();
    void execut_pos_record();
    void execute_look_for();

    void execute_move_to_position(int position_index);
    void execute_lift_search();


    // Arm control operations (private methods)
    void stop_arm_motion();
    void execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration);
    void execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration);
    

    // Helper methods
    bool get_object_pose_in_base_frame(geometry_msgs::msg::PoseStamped& pose_out,float z);
    void set_parameter_on_remote_node(const std::string& node_name, const std::string& param_name, const rclcpp::Parameter& param);
    void load_arm_positions_from_yaml();

    // Callbacks
    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter>& params);

    // TF2 for transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr visual_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_space_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr air_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vision_command_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr box_grid_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arm_finished_pub;
    // rclcpp::Publisher<robot_msgs::msg::Armmode>::SharedPtr arm_state_pub_2;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_cmd_sub_;


    // Parameters
    std::atomic<int32_t> arm_task_mode_{0}; // 0: standby, 1: grasp, 2: place, 1x: move to position x
    std::atomic<bool> task_running_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> visual_servo_active_{false};
    int current_mode{0};

    // Thread safety
    std::mutex task_mutex_;
    std::mutex pose_mutex_;
    std::mutex visual_servo_state_mutex_;
    std::condition_variable visual_servo_state_cv_;
    std::thread task_thread_;
    std::thread visual_servo_thread_;

    bool visual_servo_result_ready_{false};
    bool visual_servo_succeeded_{false};

    // Task data
    geometry_msgs::msg::PoseStamped target_object_pose_;
    geometry_msgs::msg::PoseStamped place_target_pose_;
    bool has_object_pose_{false};
    bool has_place_target_{false};

    // Configuration
    std::string base_frame_{"arm_base_link"};
    std::string camera_frame_{"camera_link"};
    std::string object_frame_{"target_object"};
    std::string tip_frame_{"link5"};
    std::string arm_calc_node_name_{"arm_calc_node"};
    double approach_distance_{0.1};    // meters above target
    double trajectory_duration_{3.0};  // seconds
    double visual_servo_kp_{0.1};
    double visual_servo_max_linear_acc_{0.1};
    int air_pump_pin_{0};              // Parameter service index for air pump control
    int start_scan{0};
    std::atomic<int> scan_finished_{0}; // 0: not started, 1: finished

    std::mutex arm_cmd_mutex_;
    std::atomic<int> arm_up_cmd_{0};


    // Joint positions from YAML
    std::map<int, std::vector<double>> arm_positions_;
    std::vector<double> ready_position{0.0, 2.1, 2.0, 3.0}; //机械臂抓块时的预设位置
    std::vector<double> home_position_{0.0, 0.1, 0.0, 0.0};//机械臂初始0位置
    std::vector<double> grasp_position{0.0, 3.14159, 2.45, 2.48};
    std::vector<double> grasp_position_two{0.0, 3.14159, 2.4, 2.55};
    std::vector<double> place_position{0.0, 2.1, 2.0, 3.0};
    std::vector<double> place_position_2{0.0, 1.57, 2.0, 2.3};
    std::vector<double> look_for_position_{0.0, 1.2, 2.3, 2.8};   //这是全场扫描时机械臂合适的位置    //0.0 1.0 2.45 3.1
    
    //下面三个位置是机械臂当抓取过程识别不到物块时，会抬高机械臂去寻找物块时预设的三个位置，分别是向左看、向中间看、向右看
    std::vector<double> look_left_position_{-0.8, 1.5, 2.45, 3.1};
    std::vector<double> look_middle_position_{0.0, 1.5, 2.45, 3.1};
    std::vector<double> look_right_position_{0.8, 1.5, 2.45, 3.1};
    
    std::vector<double> grasp_finish_position{0.0, 0.1, 0.1, -0.8};
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    // Remote node clients for parameter setting
    rclcpp::AsyncParametersClient::SharedPtr arm_calc_param_client_;
};

} // namespace arm_task
