#pragma once

#include <atomic>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
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
    // Task execution thread
    void task_execution_thread();

    // State machine for task execution
    void execute_task_state_machine();
    
    void execute_grasp_flow_on_hand();
    void execute_grasp_flow_on_box();
    void execute_place_flow_1_on_hand();
    void execute_place_flow_2_on_hand();
    void execute_place_flow_1_on_box();
    void execute_place_flow_2_on_box();
    void execut_pos_record();
    void execute_look_for();

    void execute_lift_search();


    // Arm control operations (private methods)
    void execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration);
    void execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration);
    void set_initial_arm_state(const std::vector<double>& joint_angles);
    bool sample_target_xy_from_tf(double tf_timeout_sec, double& x, double& y);
    geometry_msgs::msg::PoseStamped make_fixed_pitch_pose(double x, double y, double z, double pitch_offset) const;
    bool wait_for_stable_vision_target(geometry_msgs::msg::Point& vision_box_pos, double& vision_variance);
    bool wait_for_stable_place_target(geometry_msgs::msg::Point& vision_box_pos, double& vision_variance);
    void set_air_pump(bool enabled);
    

    // Helper methods
    void declare_config_parameters();
    void load_config_parameters();

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

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr arm_cmd_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr box_pos_by_vision_sub;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr place_pos_by_vision_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr box_to_hand_dis_sub;


    // Parameters
    std::atomic<int32_t> arm_task_mode_{0}; // 0: standby, 1: grasp, 2: place, 1x: move to position x
    std::atomic<bool> task_running_{false};
    std::atomic<bool> shutdown_requested_{false};
    int current_mode{0}; 
    float current_box_hand_dis{0.0};

    std::thread task_thread_;

    // Configuration
    std::string base_frame_{"arm_base_link"};
    std::string camera_frame_{"camera_link"};
    std::string object_frame_{"object_fame"};
    std::string arm_calc_node_name_{"arm_calc_node"};
    std::string vision_model_node_name_{"arm_node"};
    bool air_pump_{false};              // Parameter service index for air pump control
    std::atomic<bool> use_vision_grasp_{true};
    std::atomic<int> scan_finished_{0}; // 0: not started, 1: finished

    // Joint positions from task_config.yaml
    std::vector<double> ready_position{0.0, 2.1, 2.0, 3.0}; //机械臂抓块时的预设位置
    std::vector<double> home_position_{0.0, 0.1, 0.1, 0.0};//机械臂初始0位置
    std::vector<double> place_position{0.0, 1.7, 2.8, 3.3};
    std::vector<double> place_position_2{0.0, 1.7, 2.8, 3.3};
    std::vector<double> look_for_position_{0.0, 1.2, 2.3, 2.8};   //这是全场扫描时机械臂合适的位置    //0.0 1.0 2.45 3.1
    std::vector<double> grasp_finish_position{0.0, 0.1, 0.1, -0.8};
    std::vector<double> release_box_position{0.0, 1.8, 3.8, 0.6};   //将箱子放到狗背上的关节位置
    std::vector<double> re_graspe_box_position{0.0, 0.8, 3.3, -1.2};
    std::vector<double> place_box_collision_avoid_position{0.0, 0.1, 1.6, 0.2};
    std::vector<double> finished_release_box_position{0.0, 1.8, 2.0, 0.6};  //放置碰撞的中间轨迹点
    

    double grasp_z_{-0.16};
    double rady_grasp_z_{0.07};
    double place_level_1_z_{0.10};
    double place_level_2_z_{0.15};
    std::atomic<double> pitch_offset_{-0.25};

    double grasp_vision_threshold_variance_{0.1};
    int pump_on_wait_ms_{600};
    double record_ready_duration_{0.7};
    double grasp_ready_duration_{0.2};
    double grasp_pregrasp_duration_{0.3};
    double grasp_descend_duration_{0.3};
    double grasp_lift_duration_{0.4};
    double grasp_finish_duration_{1.7};
    double release_box_duration_{2.1};
    double release_collision_avoid_duration_{0.2};
    double release_home_duration_{0.35};
    double place_hand_level_1_prepare_duration_{2.4};
    double place_hand_level_1_cartesian_duration_{0.6};
    double place_hand_level_1_home_duration_{0.3};
    double place_hand_level_2_prepare_duration_{3.0};
    double place_hand_level_2_cartesian_duration_{0.6};
    double place_hand_level_2_retract_duration_{0.2};
    double place_hand_level_2_home_duration_{0.5};
    double place_box_re_grasp_duration_{1.0};
    double place_box_collision_avoid_duration_{0.1};
    double place_box_level_1_prepare_duration_{3.5};
    double place_box_level_1_cartesian_duration_{0.6};
    double place_box_level_1_home_duration_{0.3};
    double place_box_level_2_prepare_duration_{3.5};
    double place_box_level_2_cartesian_duration_{1.0};
    double place_box_level_2_retract_duration_{0.2};
    double place_box_level_2_home_duration_{0.5};
    double look_for_prepare_duration_{1.5};
    double scan_start_duration_{0.2};
    double scan_home_duration_{0.5};

    double scan_start_joint_0_{-0.4};
    double scan_stop_joint_0_{0.4};
    double scan_initial_wait_sec_{5.0};
    double scan_sweep_duration_{8.0};
    
    // Parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

    // Remote node clients for parameter setting
    rclcpp::AsyncParametersClient::SharedPtr arm_calc_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr vision_model_param_client_;

    std::mutex vision_pose_mutex_;
    geometry_msgs::msg::Point latest_vision_box_pos_;
    rclcpp::Time latest_vision_box_pos_time_;
    bool has_vision_box_pos_{false};
};

} // namespace arm_task
