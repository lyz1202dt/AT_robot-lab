#include "arm_task/task.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <limits>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/detail/int32__struct.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace arm_task {

ArmTaskNode::ArmTaskNode(const rclcpp::NodeOptions& options)
    : Node("arm_task", options) {

    RCLCPP_INFO(this->get_logger(), "Initializing ArmTaskNode...");

    // Initialize TF2
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare parameters
    this->declare_parameter<int32_t>("arm_task", 0);
    this->declare_parameter<bool>("air_pump", false);
    this->declare_parameter<bool>("use_vision_grasp", false);    //开启使用机械臂进行视觉抓取
    this->declare_parameter<bool>("enable_retry", true);
    this->declare_parameter<std::string>("base_frame", "arm_base_link");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("object_frame", "object_frame");
    this->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");
    this->declare_parameter<std::string>("vision_model_node_name", "arm_node");
    declare_config_parameters();

    // Get parameters
    this->get_parameter("air_pump", air_pump_);
    bool use_vision_grasp = true;
    this->get_parameter("use_vision_grasp", use_vision_grasp);
    use_vision_grasp_ = use_vision_grasp;
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("object_frame", object_frame_);
    this->get_parameter("arm_calc_node_name", arm_calc_node_name_);
    this->get_parameter("vision_model_node_name", vision_model_node_name_);
    load_config_parameters();

    // Create publishers
    visual_target_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);

    box_to_hand_dis_sub = this->create_subscription<std_msgs::msg::Float32>(
        "plane_dst", 10, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(box_to_hand_dis_mutex_);
            current_box_hand_dis = msg->data;
            box_to_hand_dis_msg_count_++;
        });
    // 当发1时通知视觉可以开始全场扫描，当发2时通知视觉可以开始寻找并发布物块坐标使机械臂能够去抓取物块
    vision_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("arm_command", 10);

    place_pos_by_vision_sub =
        this->create_subscription<geometry_msgs::msg::Point>("color_pnp_move", 10, [this](geometry_msgs::msg::Point::ConstSharedPtr pose) {
            std::lock_guard<std::mutex> lock(vision_pose_mutex_);
            latest_vision_box_pos_      = *pose;
            latest_vision_box_pos_time_ = this->now();
            has_vision_box_pos_         = true;
        });


    // 结束扫描，机械臂需要回到初始位置
    box_grid_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "box_id_grid", 10,
        [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr /*msg*/) { scan_finished_ = 1; }); // 该消息发出说明扫描结束

    // // 跟上层控制反馈当前机械臂状态，是否抓到物块了
    arm_finished_pub = this->create_publisher<std_msgs::msg::Int32>("arm_cmd_state", 10);

    // 上层控制命令订阅，告诉机械臂执行哪个任务
    arm_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>("arm_cmd", 10, [this](const std_msgs::msg::Int32& msg) {
        RCLCPP_INFO(get_logger(), "arm_task接收到命令");
        arm_task_mode_ = msg.data;
    });

    // 气泵的控制话题，发布机械臂需要的吸取和放置命令
    air_pub_ = this->create_publisher<std_msgs::msg::Int32>("air_pump_target", 10);

    box_pos_by_vision_sub =
        this->create_subscription<geometry_msgs::msg::Point>("pnp_move", 10, [this](geometry_msgs::msg::Point::ConstSharedPtr pose) {
            std::lock_guard<std::mutex> lock(vision_pose_mutex_);
            latest_vision_box_pos_      = *pose;
            latest_vision_box_pos_time_ = this->now();
            has_vision_box_pos_         = true;
        });

    // 参数服务的回调
    param_callback_ = this->add_on_set_parameters_callback(std::bind(&ArmTaskNode::on_parameters_changed, this, std::placeholders::_1));

    arm_calc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, arm_calc_node_name_);
    RCLCPP_INFO(this->get_logger(), "Using arm_calc parameter client target: %s", arm_calc_node_name_.c_str());
    set_initial_arm_state(home_position_);

    vision_model_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, vision_model_node_name_);
    RCLCPP_INFO(this->get_logger(), "Using vision parameter client target: %s", vision_model_node_name_.c_str());

    // Start task execution thread
    task_thread_ = std::thread(&ArmTaskNode::task_execution_thread, this);

    RCLCPP_INFO(this->get_logger(), "ArmTaskNode initialized successfully");
}

ArmTaskNode::~ArmTaskNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down ArmTaskNode...");
    shutdown_requested_ = true;

    if (task_thread_.joinable()) {
        task_thread_.join();
    }
}

void ArmTaskNode::declare_config_parameters() {
    this->declare_parameter<std::vector<double>>("positions.ready", ready_position);
    this->declare_parameter<std::vector<double>>("positions.home", home_position_);
    this->declare_parameter<std::vector<double>>("positions.place_level_1", place_position);
    this->declare_parameter<std::vector<double>>("positions.place_level_2", place_position_2);
    this->declare_parameter<std::vector<double>>("positions.look_for", look_for_position_);
    this->declare_parameter<std::vector<double>>("positions.grasp_finish", grasp_finish_position);
    this->declare_parameter<std::vector<double>>("positions.release_box", release_box_position);
    this->declare_parameter<std::vector<double>>("positions.re_grasp_box", re_graspe_box_position);
    this->declare_parameter<std::vector<double>>("positions.re_grasp_box_collision_avoid", re_graspe_box_collision_avoid_position);
    this->declare_parameter<std::vector<double>>("positions.place_box_collision_avoid", place_box_collision_avoid_position);
    this->declare_parameter<std::vector<double>>("positions.finished_release_box", finished_release_box_position);

    this->declare_parameter<double>("poses.grasp_z", grasp_z_);
    this->declare_parameter<double>("poses.rady_grasp_z", rady_grasp_z_);
    this->declare_parameter<double>("poses.place_level_1_z", place_level_1_z_);
    this->declare_parameter<double>("poses.place_level_2_z", place_level_2_z_);
    this->declare_parameter<double>("poses.pitch_offset", pitch_offset_);

    this->declare_parameter<double>("vision.grasp_threshold_variance", grasp_vision_threshold_variance_);

    this->declare_parameter<int>("timing.pump_on_wait_ms", pump_on_wait_ms_);
    this->declare_parameter<double>("timing.record_ready_duration", record_ready_duration_);
    this->declare_parameter<double>("timing.grasp_ready_duration", grasp_ready_duration_);
    this->declare_parameter<double>("timing.grasp_pregrasp_duration", grasp_pregrasp_duration_);
    this->declare_parameter<double>("timing.grasp_descend_duration", grasp_descend_duration_);
    this->declare_parameter<double>("timing.grasp_lift_duration", grasp_lift_duration_);
    this->declare_parameter<double>("timing.grasp_finish_duration", grasp_finish_duration_);
    this->declare_parameter<double>("timing.release_box_duration", release_box_duration_);
    this->declare_parameter<double>("timing.release_collision_avoid_duration", release_collision_avoid_duration_);
    this->declare_parameter<double>("timing.release_home_duration", release_home_duration_);
    this->declare_parameter<double>("timing.place_hand_level_1_prepare_duration", place_hand_level_1_prepare_duration_);
    this->declare_parameter<double>("timing.place_hand_level_1_cartesian_duration", place_hand_level_1_cartesian_duration_);
    this->declare_parameter<double>("timing.place_hand_level_1_home_duration", place_hand_level_1_home_duration_);
    this->declare_parameter<double>("timing.place_hand_level_2_prepare_duration", place_hand_level_2_prepare_duration_);
    this->declare_parameter<double>("timing.place_hand_level_2_cartesian_duration", place_hand_level_2_cartesian_duration_);
    this->declare_parameter<double>("timing.place_hand_level_2_retract_duration", place_hand_level_2_retract_duration_);
    this->declare_parameter<double>("timing.place_hand_level_2_home_duration", place_hand_level_2_home_duration_);
    this->declare_parameter<double>("timing.place_box_re_grasp_duration", place_box_re_grasp_duration_);
    this->declare_parameter<double>("timing.place_box_re_grasp_collision_avoid_duration", place_box_re_grasp_collision_avoid_duration_);
    this->declare_parameter<double>("timing.place_box_collision_avoid_duration", place_box_collision_avoid_duration_);
    this->declare_parameter<double>("timing.place_box_level_1_prepare_duration", place_box_level_1_prepare_duration_);
    this->declare_parameter<double>("timing.place_box_level_1_cartesian_duration", place_box_level_1_cartesian_duration_);
    this->declare_parameter<double>("timing.place_box_level_1_home_duration", place_box_level_1_home_duration_);
    this->declare_parameter<double>("timing.place_box_level_2_prepare_duration", place_box_level_2_prepare_duration_);
    this->declare_parameter<double>("timing.place_box_level_2_cartesian_duration", place_box_level_2_cartesian_duration_);
    this->declare_parameter<double>("timing.place_box_level_2_retract_duration", place_box_level_2_retract_duration_);
    this->declare_parameter<double>("timing.place_box_level_2_home_duration", place_box_level_2_home_duration_);
    this->declare_parameter<double>("timing.look_for_prepare_duration", look_for_prepare_duration_);
    this->declare_parameter<double>("timing.scan_start_duration", scan_start_duration_);
    this->declare_parameter<double>("timing.scan_home_duration", scan_home_duration_);

    this->declare_parameter<double>("scan.start_joint_0", scan_start_joint_0_);
    this->declare_parameter<double>("scan.stop_joint_0", scan_stop_joint_0_);
    this->declare_parameter<double>("scan.initial_wait_sec", scan_initial_wait_sec_);
    this->declare_parameter<double>("scan.sweep_duration", scan_sweep_duration_);
}

void ArmTaskNode::load_config_parameters() {
    ready_position                         = this->get_parameter("positions.ready").as_double_array();
    home_position_                         = this->get_parameter("positions.home").as_double_array();
    place_position                         = this->get_parameter("positions.place_level_1").as_double_array();
    place_position_2                       = this->get_parameter("positions.place_level_2").as_double_array();
    look_for_position_                     = this->get_parameter("positions.look_for").as_double_array();
    grasp_finish_position                  = this->get_parameter("positions.grasp_finish").as_double_array();
    release_box_position                   = this->get_parameter("positions.release_box").as_double_array();
    re_graspe_box_position                 = this->get_parameter("positions.re_grasp_box").as_double_array();
    re_graspe_box_collision_avoid_position = this->get_parameter("positions.re_grasp_box_collision_avoid").as_double_array();
    place_box_collision_avoid_position     = this->get_parameter("positions.place_box_collision_avoid").as_double_array();
    finished_release_box_position          = this->get_parameter("positions.finished_release_box").as_double_array();

    grasp_z_         = this->get_parameter("poses.grasp_z").as_double();
    rady_grasp_z_    = this->get_parameter("poses.rady_grasp_z").as_double();
    place_level_1_z_ = this->get_parameter("poses.place_level_1_z").as_double();
    place_level_2_z_ = this->get_parameter("poses.place_level_2_z").as_double();
    pitch_offset_    = this->get_parameter("poses.pitch_offset").as_double();

    grasp_vision_threshold_variance_ = this->get_parameter("vision.grasp_threshold_variance").as_double();

    pump_on_wait_ms_                            = this->get_parameter("timing.pump_on_wait_ms").as_int();
    record_ready_duration_                      = this->get_parameter("timing.record_ready_duration").as_double();
    grasp_ready_duration_                       = this->get_parameter("timing.grasp_ready_duration").as_double();
    grasp_pregrasp_duration_                    = this->get_parameter("timing.grasp_pregrasp_duration").as_double();
    grasp_descend_duration_                     = this->get_parameter("timing.grasp_descend_duration").as_double();
    grasp_lift_duration_                        = this->get_parameter("timing.grasp_lift_duration").as_double();
    grasp_finish_duration_                      = this->get_parameter("timing.grasp_finish_duration").as_double();
    release_box_duration_                       = this->get_parameter("timing.release_box_duration").as_double();
    release_collision_avoid_duration_           = this->get_parameter("timing.release_collision_avoid_duration").as_double();
    release_home_duration_                      = this->get_parameter("timing.release_home_duration").as_double();
    place_hand_level_1_prepare_duration_        = this->get_parameter("timing.place_hand_level_1_prepare_duration").as_double();
    place_hand_level_1_cartesian_duration_      = this->get_parameter("timing.place_hand_level_1_cartesian_duration").as_double();
    place_hand_level_1_home_duration_           = this->get_parameter("timing.place_hand_level_1_home_duration").as_double();
    place_hand_level_2_prepare_duration_        = this->get_parameter("timing.place_hand_level_2_prepare_duration").as_double();
    place_hand_level_2_cartesian_duration_      = this->get_parameter("timing.place_hand_level_2_cartesian_duration").as_double();
    place_hand_level_2_retract_duration_        = this->get_parameter("timing.place_hand_level_2_retract_duration").as_double();
    place_hand_level_2_home_duration_           = this->get_parameter("timing.place_hand_level_2_home_duration").as_double();
    place_box_re_grasp_duration_                = this->get_parameter("timing.place_box_re_grasp_duration").as_double();
    place_box_re_grasp_collision_avoid_duration_ = this->get_parameter("timing.place_box_re_grasp_collision_avoid_duration").as_double();
    place_box_collision_avoid_duration_         = this->get_parameter("timing.place_box_collision_avoid_duration").as_double();
    place_box_level_1_prepare_duration_         = this->get_parameter("timing.place_box_level_1_prepare_duration").as_double();
    place_box_level_1_cartesian_duration_       = this->get_parameter("timing.place_box_level_1_cartesian_duration").as_double();
    place_box_level_1_home_duration_            = this->get_parameter("timing.place_box_level_1_home_duration").as_double();
    place_box_level_2_prepare_duration_         = this->get_parameter("timing.place_box_level_2_prepare_duration").as_double();
    place_box_level_2_cartesian_duration_       = this->get_parameter("timing.place_box_level_2_cartesian_duration").as_double();
    place_box_level_2_retract_duration_         = this->get_parameter("timing.place_box_level_2_retract_duration").as_double();
    place_box_level_2_home_duration_            = this->get_parameter("timing.place_box_level_2_home_duration").as_double();
    look_for_prepare_duration_                  = this->get_parameter("timing.look_for_prepare_duration").as_double();
    scan_start_duration_                        = this->get_parameter("timing.scan_start_duration").as_double();
    scan_home_duration_                         = this->get_parameter("timing.scan_home_duration").as_double();

    scan_start_joint_0_    = this->get_parameter("scan.start_joint_0").as_double();
    scan_stop_joint_0_     = this->get_parameter("scan.stop_joint_0").as_double();
    scan_initial_wait_sec_ = this->get_parameter("scan.initial_wait_sec").as_double();
    scan_sweep_duration_   = this->get_parameter("scan.sweep_duration").as_double();
}

rcl_interfaces::msg::SetParametersResult ArmTaskNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "arm_task") {
            int32_t new_mode = param.as_int();
            arm_task_mode_   = new_mode;
            RCLCPP_INFO(this->get_logger(), "Task mode changed to: %d", new_mode);
        } else if (param.get_name() == "air_pump") {
            set_air_pump(param.as_bool());
        } else if (param.get_name() == "use_vision_grasp") {
            use_vision_grasp_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Use vision grasp changed to: %s", use_vision_grasp_.load() ? "true" : "false");
        } else if (param.get_name() == "poses.pitch_offset") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                result.successful = false;
                result.reason     = "poses.pitch_offset must be a double";
                return result;
            }

            pitch_offset_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Pitch offset changed to: %.6f", pitch_offset_);
        }
    }

    return result;
}

void ArmTaskNode::task_execution_thread() {
    RCLCPP_INFO(this->get_logger(), "线程启动成功");

    while (!shutdown_requested_) {
        execute_task_state_machine();
        std::this_thread::sleep_for(100ms);
    }

    RCLCPP_INFO(this->get_logger(), "线程结束");
}

void ArmTaskNode::execut_pos_record() {
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position, record_ready_duration_);
    std::this_thread::sleep_for(750ms);
}

// 这个函数是整个机械臂任务的核心，它根据当前的任务模式（arm_task_mode_）来决定执行哪个具体的任务流程（抓取、放置、搜索等）。它还负责管理任务的状态，
// 确保在一个任务执行过程中不会被新的任务打断，并且在任务完成后重置状态以准备下一次任务。
// 该函数为线性执行函数
void ArmTaskNode::execute_task_state_machine() {

    current_mode = arm_task_mode_.load(); // 调试时用的，实际跑时应该注释掉，current_mode直接接受回调里的赋值（在最下面）


    // 已经在运行
    if (task_running_) {
        return;
    }

    // 进入BUSY状态
    task_running_ = true;

    try {
        if (current_mode == 1) {
            // 执行抓块流程
            RCLCPP_INFO(this->get_logger(), "开始抓取到框中的任务");
            execute_grasp_flow_on_box();
        }
        if (current_mode == 2) {
            // 执行抓块流程
            RCLCPP_INFO(this->get_logger(), "开始抓取到手上任务");
            execute_grasp_flow_on_hand();

        } else if (current_mode == 3) {
            // 执行第一层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第一层从框里的放置任务");
            execute_place_flow_1_on_box();
        } else if (current_mode == 4) {
            // 执行第二层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第二层从框里的放置任务");
            execute_place_flow_2_on_box();
        } else if (current_mode == 5) {
            // 执行第一层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第一层从手上的放置任务");
            execute_place_flow_1_on_hand();
        } else if (current_mode == 6) {
            // 执行第二层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第二层从手上的放置任务");
            execute_place_flow_2_on_hand();
        } else if (current_mode == 7) {
            // 在比赛开始时，机械臂需要先巡视扫描场地上的物块，确定狗的巡线流程
            RCLCPP_INFO(this->get_logger(), "巡视扫描物块");
            execute_look_for();
        } else if (current_mode == 10) {
            RCLCPP_INFO(this->get_logger(), "调试录点模式");
            execut_pos_record();
        }
        if (current_mode) {
            current_mode   = 0;
            arm_task_mode_ = 0;
            this->set_parameter(rclcpp::Parameter("arm_task", 0));
        } else {
            std::this_thread::sleep_for(100ms);
        }
        // Reset mode to standby after completion
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Task execution failed: %s", e.what());
        current_mode   = 0;
        arm_task_mode_ = 0;
        this->set_parameter(rclcpp::Parameter("arm_task", 0));
    }

    task_running_ = false;
}

// 抓块函数
void ArmTaskNode::execute_grasp_flow_on_hand() {
    constexpr const int kRetrycnt = 2;
    int max_ryretry               = kRetrycnt;
    do {
        // 机械臂先预摆到一个合适的位置，方便相机观察和后续运动
        RCLCPP_INFO(this->get_logger(), "移动到准备位置");
        execute_joint_space_trajectory(ready_position, grasp_ready_duration_);
        std::this_thread::sleep_for(250ms);

        double x = 0.0;
        double y = 0.0;
        if (!sample_target_xy_from_tf(0.02, x, y)) {
            return;
        }

        // 移动到预抓取位置，等待相机识别
        
        geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, rady_grasp_z_, pitch_offset_);

        //视觉识别时，需要抬得高点
        //geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, 0.3, pitch_offset_);


        // 3.笛卡尔轨迹规划使机械臂运动到开启视觉识别的位置
        RCLCPP_INFO(this->get_logger(), "移动到块的预抓取位置");
        execute_cartesian_space_trajectory(object_pose, grasp_pregrasp_duration_);
        std::this_thread::sleep_for(350ms);

        double vision_weight = 0.0;
        geometry_msgs::msg::Point vision_box_pos;
        if (use_vision_grasp_.load()) {
            // 4.触发视觉识别动作
            RCLCPP_INFO(this->get_logger(), "启动视觉识别动作");
            double vision_variance = std::numeric_limits<double>::infinity();
            bool vision_ready      = wait_for_stable_vision_target(vision_box_pos, vision_variance);

            if (vision_ready) {
                geometry_msgs::msg::PointStamped vision_point_camera;
                vision_point_camera.header.stamp.sec     = 0;
                vision_point_camera.header.stamp.nanosec = 0;
                vision_point_camera.header.frame_id      = camera_frame_;
                vision_point_camera.point                = vision_box_pos;

                try {
                    const auto vision_point_base = tf_buffer_->transform(vision_point_camera, base_frame_, tf2::durationFromSec(0.1));
                    vision_box_pos               = vision_point_base.point;
                    // vision_weight = grasp_vision_threshold_variance_ / (grasp_vision_threshold_variance_ + vision_variance);
                    // vision_weight = std::max(0.0, std::min(1.0, vision_weight));
                    vision_weight = 0.5;      // 先写死平均数加权

                    double dx=vision_box_pos.x-object_pose.pose.position.x;
                    double dy=vision_box_pos.y-object_pose.pose.position.y;
                    double dz=vision_box_pos.z-object_pose.pose.position.z;
                    if(std::sqrt(dx*dx+dy*dy)>0.2||std::abs(dz)>0.1)
                    {
                        vision_weight = 0.0;
                        RCLCPP_WARN(this->get_logger(), "视觉坐标和相机坐标差距过大，水平为%f,竖直为%f,不信任视觉坐标",std::sqrt(dx*dx+dy*dy),dz);
                    }
                    else {
                    RCLCPP_INFO(
                        this->get_logger(), "视觉坐标已从 %s 转到 %s: x=%.4f, y=%.4f, z=%.4f", camera_frame_.c_str(), base_frame_.c_str(),
                        vision_box_pos.x, vision_box_pos.y, vision_box_pos.z);
                    }
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(
                        this->get_logger(), "视觉坐标从 %s 转到 %s 失败，使用雷达坐标抓取: %s", camera_frame_.c_str(), base_frame_.c_str(),
                        ex.what());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "视觉识别未在超时时间内稳定，使用雷达坐标抓取");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "未启用视觉辅助抓取，使用雷达坐标抓取");
        }

        sample_target_xy_from_tf(0.02, x, y); // 拿到最新的稳定TF
        object_pose.pose.position.x = x;
        object_pose.pose.position.y = y;

        if (vision_weight > 0.0) {
            object_pose.pose.position.x = object_pose.pose.position.x * (1.0 - vision_weight) + vision_box_pos.x * vision_weight;
            object_pose.pose.position.y = object_pose.pose.position.y * (1.0 - vision_weight) + vision_box_pos.y * vision_weight;
        }
        object_pose.pose.position.z = grasp_z_;

        // if (max_ryretry != kRetrycnt)         // 重试时抓得更靠下一点
        //     object_pose.pose.position.z -= 0.03;

        RCLCPP_INFO(
            this->get_logger(), "抓取坐标: x=%.4f, y=%.4f, z=%.4f, vision_weight=%.3f", object_pose.pose.position.x,
            object_pose.pose.position.y, object_pose.pose.position.z, vision_weight);

        execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
        std::this_thread::sleep_for(200ms);

        // 通知气泵开始吸了
        RCLCPP_INFO(this->get_logger(), "启动气泵");
        set_air_pump(true);

        std::this_thread::sleep_for(500ms);
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试正前方
        {
            set_air_pump(false);
            object_pose.pose.position.x+=0.12f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试左侧
        {
            set_air_pump(false);
            object_pose.pose.position.x-=0.12f;
            object_pose.pose.position.y+=0.12f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试右侧
        {
            set_air_pump(false);
            object_pose.pose.position.y-=0.24f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }
        // if (max_ryretry != kRetrycnt) // 重试时抓的时间长一些
        //     std::this_thread::sleep_for(500ms);

        object_pose.pose.position.z = rady_grasp_z_;
        execute_cartesian_space_trajectory(object_pose, grasp_lift_duration_);
        std::this_thread::sleep_for(std::chrono::milliseconds(pump_on_wait_ms_));

        RCLCPP_INFO(this->get_logger(), "移动到准备位置");
        execute_joint_space_trajectory(grasp_finish_position, grasp_finish_duration_);

        std::this_thread::sleep_for(800ms);

        max_ryretry--;
        RCLCPP_INFO(get_logger(),"测距模块读数%f", get_current_box_hand_dis());
        
    } while ((get_current_box_hand_dis()>0.1f )&& max_ryretry);

    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(900ms);

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_grasp_flow_on_box() {
    constexpr const int kRetrycnt = 2;
    int max_ryretry               = kRetrycnt;

    bool re_grasp=false;
    do {
        RCLCPP_INFO(this->get_logger(), "移动到准备位置");
        execute_joint_space_trajectory(ready_position, grasp_ready_duration_);
        std::this_thread::sleep_for(250ms);


        double x = 0.0;
        double y = 0.0;
        if (!sample_target_xy_from_tf(0.02, x, y)) {
            return;
        }

        // 移动到预抓取位置，等待相机识别
        RCLCPP_INFO(this->get_logger(), "移动到预抓取位置");


        geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, rady_grasp_z_, pitch_offset_);

        //视觉识别时，需要抬得高点
        //geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, 0.3, pitch_offset_);

        
        execute_cartesian_space_trajectory(object_pose, grasp_pregrasp_duration_);
        std::this_thread::sleep_for(350ms);

        double vision_weight = 0.0;
        geometry_msgs::msg::Point vision_box_pos;
        if (use_vision_grasp_.load()) {
            // 4.触发视觉识别动作
            RCLCPP_INFO(this->get_logger(), "启动视觉识别");
            double vision_variance = std::numeric_limits<double>::infinity();
            bool vision_ready      = wait_for_stable_vision_target(vision_box_pos, vision_variance);

            if (vision_ready) {
                geometry_msgs::msg::PointStamped vision_point_camera;
                vision_point_camera.header.stamp.sec     = 0;
                vision_point_camera.header.stamp.nanosec = 0;
                vision_point_camera.header.frame_id      = camera_frame_;
                vision_point_camera.point                = vision_box_pos;

                try {
                    const auto vision_point_base = tf_buffer_->transform(vision_point_camera, base_frame_, tf2::durationFromSec(0.1));
                    vision_box_pos               = vision_point_base.point;
                    // vision_weight = grasp_vision_threshold_variance_ / (grasp_vision_threshold_variance_ + vision_variance);
                    // vision_weight = std::max(0.0, std::min(1.0, vision_weight));
                    vision_weight = 0.5;      // 先写死平均数加权
                    RCLCPP_INFO(get_logger(),"从背上抓取失败，触发重规划");
                    double dx=vision_box_pos.x-object_pose.pose.position.x;
                    double dy=vision_box_pos.y-object_pose.pose.position.y;
                    double dz=vision_box_pos.z-object_pose.pose.position.z;
                    if(std::sqrt(dx*dx+dy*dy)>0.2||std::abs(dz)>0.1)
                    {
                        vision_weight = 0.0;
                        RCLCPP_WARN(this->get_logger(), "视觉坐标和相机坐标差距过大，水平为%f,竖直为%f,不信任视觉坐标",std::sqrt(dx*dx+dy*dy),dz);
                    }
                    else {
                    RCLCPP_INFO(
                        this->get_logger(), "视觉坐标已从 %s 转到 %s: x=%.4f, y=%.4f, z=%.4f", camera_frame_.c_str(), base_frame_.c_str(),
                        vision_box_pos.x, vision_box_pos.y, vision_box_pos.z);
                    }
                    
                } catch (const tf2::TransformException& ex) {
                    RCLCPP_WARN(
                        this->get_logger(), "视觉坐标从 %s 转到 %s 失败，使用雷达坐标抓取: %s", camera_frame_.c_str(), base_frame_.c_str(),
                        ex.what());
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "视觉识别未在超时时间内稳定，使用雷达坐标抓取");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "未启用视觉辅助抓取，使用雷达坐标抓取");
        }

        sample_target_xy_from_tf(0.02, x, y); // 拿到最新的稳定TF
        object_pose.pose.position.x = x;
        object_pose.pose.position.y = y;

        if (vision_weight > 0.0) {
            object_pose.pose.position.x = object_pose.pose.position.x * (1.0 - vision_weight) + vision_box_pos.x * vision_weight;
            object_pose.pose.position.y = object_pose.pose.position.y * (1.0 - vision_weight) + vision_box_pos.y * vision_weight;
        }
        object_pose.pose.position.z = grasp_z_;

        // if (max_ryretry != kRetrycnt)
        //     object_pose.pose.position.z -= 0.05f;

        RCLCPP_INFO(
            this->get_logger(), "抓取坐标: x=%.4f, y=%.4f, z=%.4f, vision_weight=%.3f", object_pose.pose.position.x,
            object_pose.pose.position.y, object_pose.pose.position.z, vision_weight);

        execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
        std::this_thread::sleep_for(200ms);

        // 通知气泵开始吸了
        RCLCPP_INFO(this->get_logger(), "启动气泵");
        set_air_pump(true);

        // if (max_ryretry != kRetrycnt)
        //     std::this_thread::sleep_for(500ms);

        std::this_thread::sleep_for(500ms);
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试正前方
        {
            set_air_pump(false);
            object_pose.pose.position.x+=0.12f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试左侧
        {
            set_air_pump(false);
            object_pose.pose.position.x-=0.12f;
            object_pose.pose.position.y+=0.12f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }
        if(get_current_box_hand_dis()>0.1f) //正下方抓取失败，尝试右侧
        {
            set_air_pump(false);
            object_pose.pose.position.y-=0.24f;
            object_pose.pose.position.z=rady_grasp_z_;
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_*2);
            std::this_thread::sleep_for(650ms);
            object_pose.pose.position.z=grasp_z_;
            set_air_pump(true);
            execute_cartesian_space_trajectory(object_pose, grasp_descend_duration_);
            std::this_thread::sleep_for(800ms);
        }

        object_pose.pose.position.z = rady_grasp_z_;
        execute_cartesian_space_trajectory(object_pose, grasp_lift_duration_);
        std::this_thread::sleep_for(500ms);

        RCLCPP_INFO(this->get_logger(), "移动到框的位置");
        execute_joint_space_trajectory(release_box_position, release_box_duration_);

        std::this_thread::sleep_for(1000ms);
        re_grasp=get_current_box_hand_dis()>0.1f;
        RCLCPP_INFO(get_logger(),"测距模块读数%f", get_current_box_hand_dis());

        std::this_thread::sleep_for(1400ms);

        max_ryretry--;

        
    } while (re_grasp&& max_ryretry);


    // 设置气泵松开，
    set_air_pump(false);

    std::this_thread::sleep_for(100ms); // 等待箱子落下


    // 回退到中间位置防止动作干涉
    execute_joint_space_trajectory(finished_release_box_position, release_collision_avoid_duration_);
    std::this_thread::sleep_for(250ms);

    // 通知抓取流程完成
    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    execute_joint_space_trajectory(home_position_, release_home_duration_);
    std::this_thread::sleep_for(300ms);

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_place_flow_1_on_hand() {

    // auto temp1=std::vector{-1.3,1.57,1.57,0.0};
    // execute_joint_space_trajectory(temp1, 4.0);
    // std::this_thread::sleep_for(4100ms);

    // std::this_thread::sleep_for(2000ms);


    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position, place_hand_level_1_prepare_duration_);
    std::this_thread::sleep_for(2500ms);

    double x = 0.0;
    double y = 0.0;
    if (!sample_target_xy_from_tf(0.08, x, y)) {
        RCLCPP_INFO(get_logger(), "找不到要放置的目标");
        return;
    }

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, place_level_1_z_, pitch_offset_);

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, 0.6);

    auto now=get_clock()->now();
    while(get_clock()->now()-now<rclcpp::Duration(600ms)){  //不断更新位置
        sample_target_xy_from_tf(0.08, x, y);
        object_pose.pose.position.x=x;
        object_pose.pose.position.y=y;
        visual_target_pub_->publish(object_pose);
    }

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    set_air_pump(false);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(home_position_, place_hand_level_1_home_duration_);

    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(300ms);

    RCLCPP_INFO(this->get_logger(), "第一层放块任务结束");
}

void ArmTaskNode::execute_place_flow_2_on_hand() {

    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position_2, place_hand_level_2_prepare_duration_);
    std::this_thread::sleep_for(3100ms);

    double x = 0.0;
    double y = 0.0;
    if (!sample_target_xy_from_tf(0.08, x, y)) {
        RCLCPP_INFO(get_logger(), "找不到要放置的目标");
        return;
    }

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, place_level_2_z_, pitch_offset_);

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, place_hand_level_2_cartesian_duration_);

    auto now=get_clock()->now();
    while(get_clock()->now()-now<rclcpp::Duration(600ms)){  //不断更新位置
        sample_target_xy_from_tf(0.08, x, y);
        object_pose.pose.position.x=x;
        object_pose.pose.position.y=y;
        visual_target_pub_->publish(object_pose);
    }

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    set_air_pump(false);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(place_position_2, place_hand_level_2_retract_duration_);

    std::this_thread::sleep_for(200ms);

    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(500ms); // 等待狗子离开

    execute_joint_space_trajectory(home_position_, place_hand_level_2_home_duration_);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "第二层放块任务结束");
}

void ArmTaskNode::execute_place_flow_1_on_box() {

    constexpr const int kRetrycnt = 2;
    int max_ryretry               = kRetrycnt;

    // auto temp1=std::vector{-1.3,1.57,1.57,0.0};
    // execute_joint_space_trajectory(temp1, 4.0);
    // std::this_thread::sleep_for(4100ms);

    // std::this_thread::sleep_for(2000ms);

    // execute_joint_space_trajectory(grasp_finish_position, 4.0);
    // std::this_thread::sleep_for(4100ms);

    do {
        RCLCPP_INFO(this->get_logger(), "移动到框中抓取箱子");
        set_air_pump(true);
        execute_joint_space_trajectory(re_graspe_box_collision_avoid_position, place_box_re_grasp_collision_avoid_duration_);
        std::this_thread::sleep_for(850ms);
        execute_joint_space_trajectory(re_graspe_box_position, place_box_re_grasp_duration_);
        std::this_thread::sleep_for(450ms);

        execute_joint_space_trajectory(place_box_collision_avoid_position, place_box_collision_avoid_duration_);
        std::this_thread::sleep_for(200ms);

        execute_joint_space_trajectory(place_position, place_box_level_1_prepare_duration_);
        std::this_thread::sleep_for(2000ms);

        max_ryretry--;

        if(get_current_box_hand_dis() > 0.1f)
        {
            std::this_thread::sleep_for(1600ms);
            execute_joint_space_trajectory(home_position_, 0.2);
        }
        else
            std::this_thread::sleep_for(1500ms);
        
        RCLCPP_INFO(get_logger(),"测距模块读数%f", get_current_box_hand_dis());

    } while (max_ryretry && get_current_box_hand_dis() > 0.1f);
    if (get_current_box_hand_dis() > 0.1f) {
        std_msgs::msg::Int32 ret;
        ret.data = -1;
        arm_finished_pub->publish(ret);
        RCLCPP_INFO(get_logger(),"从背上抓取失败，触发重规划");
        execute_joint_space_trajectory(home_position_, 0.3);
        std::this_thread::sleep_for(300ms);
        return;
    }

    double x = 0.0;
    double y = 0.0;
    if (!sample_target_xy_from_tf(0.08, x, y)) {
        RCLCPP_INFO(get_logger(), "找不到要放置的目标");
        return;
    }

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, place_level_1_z_, pitch_offset_);

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, place_box_level_1_cartesian_duration_);

    auto now=get_clock()->now();
    while(get_clock()->now()-now<rclcpp::Duration(600ms)){  //不断更新位置
        sample_target_xy_from_tf(0.08, x, y);
        object_pose.pose.position.x=x;
        object_pose.pose.position.y=y;
        visual_target_pub_->publish(object_pose);
    }

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    set_air_pump(false);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(home_position_, place_box_level_1_home_duration_);

    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(300ms);

    RCLCPP_INFO(this->get_logger(), "第一层放块任务结束");
}

void ArmTaskNode::execute_place_flow_2_on_box() {

    constexpr const int kRetrycnt = 2;
    int max_ryretry               = kRetrycnt;

    do {
        RCLCPP_INFO(this->get_logger(), "移动到框中抓取箱子");
        set_air_pump(true);
        execute_joint_space_trajectory(re_graspe_box_collision_avoid_position, place_box_re_grasp_collision_avoid_duration_);
        std::this_thread::sleep_for(850ms);
        execute_joint_space_trajectory(re_graspe_box_position, place_box_re_grasp_duration_);
        std::this_thread::sleep_for(450ms);

        execute_joint_space_trajectory(place_box_collision_avoid_position, place_box_collision_avoid_duration_);
        std::this_thread::sleep_for(200ms);

        execute_joint_space_trajectory(place_position_2, place_box_level_2_prepare_duration_);
        std::this_thread::sleep_for(3600ms);

        max_ryretry--;

        if(get_current_box_hand_dis() > 0.1f)
        {
            execute_joint_space_trajectory(home_position_, 0.2);
            std::this_thread::sleep_for(300ms);
        }
        
        RCLCPP_INFO(get_logger(),"测距模块读数%f", get_current_box_hand_dis());

    } while (max_ryretry && get_current_box_hand_dis() > 0.1f);
    if (get_current_box_hand_dis() > 0.1f) {
        std_msgs::msg::Int32 ret;
        ret.data = -1;
        arm_finished_pub->publish(ret);
        RCLCPP_INFO(get_logger(),"从背上抓取失败，触发重规划");
        execute_joint_space_trajectory(home_position_, 0.3);
        std::this_thread::sleep_for(300ms);
        return;
    }

    double x = 0.0;
    double y = 0.0;
    if (!sample_target_xy_from_tf(0.08, x, y)) {
        RCLCPP_INFO(get_logger(), "找不到要放置的目标");
        return;
    }

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose = make_fixed_pitch_pose(x, y, place_level_2_z_, pitch_offset_);

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, place_box_level_2_cartesian_duration_);

    auto now=get_clock()->now();
    while(get_clock()->now()-now<rclcpp::Duration(1000ms)){  //不断更新位置
        sample_target_xy_from_tf(0.08, x, y);
        object_pose.pose.position.x=x;
        object_pose.pose.position.y=y;
        visual_target_pub_->publish(object_pose);
    }

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    set_air_pump(false);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(place_position_2, place_box_level_2_retract_duration_);

    std::this_thread::sleep_for(200ms);

    std_msgs::msg::Int32 ret;
    ret.data = 1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(500ms); // 等待狗子离开

    execute_joint_space_trajectory(home_position_, place_box_level_2_home_duration_);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "第二层放块任务结束");
}


void ArmTaskNode::execute_look_for() {

    // 告知视觉开始全场扫描了
    std_msgs::msg::Int32 scan_msg;
    scan_msg.data = 1;
    vision_command_pub_->publish(scan_msg);

    // 机械臂先预摆到一个合适的位置，方便相机识别全场箱子
    execute_joint_space_trajectory(look_for_position_, look_for_prepare_duration_);
    std::this_thread::sleep_for(1500ms);

    auto start_time = std::chrono::steady_clock::now();

    auto start_joint_pos = look_for_position_;
    auto stop_joint_pos  = look_for_position_;
    start_joint_pos[0]   = scan_start_joint_0_;
    stop_joint_pos[0]    = scan_stop_joint_0_;

    // 在这里会阻塞等待视觉发布会扫描完成的消息（scan_finished_被置1），告诉机械臂可以结束等待了
    while (scan_finished_ == 0) {
        if (std::chrono::steady_clock::now() - start_time > std::chrono::duration<double>(scan_initial_wait_sec_)) {
            execute_joint_space_trajectory(start_joint_pos, scan_start_duration_); // 机械臂旋转，执行扫描
            std::this_thread::sleep_for(200ms);
            execute_joint_space_trajectory(stop_joint_pos, scan_sweep_duration_);
            std::this_thread::sleep_for(std::chrono::duration<double>(scan_sweep_duration_));
            RCLCPP_INFO(get_logger(), "搜索箱子...");
        }

        std::this_thread::sleep_for(100ms);
    }

    scan_finished_ = 0;                                                            // 清状态

    // 机械臂回到初始位置，准备接受后续的抓取指令
    execute_joint_space_trajectory(home_position_, scan_home_duration_);
    std::this_thread::sleep_for(500ms);
}

void ArmTaskNode::execute_lift_search() {
    RCLCPP_INFO(this->get_logger(), "开始搜索任务");

    RCLCPP_INFO(this->get_logger(), "搜索任务结束");
    std::this_thread::sleep_for(1500ms);
    current_mode = 0;
}

bool ArmTaskNode::sample_target_xy_from_tf(double tf_timeout_sec, double& x, double& y) {
    constexpr int sample_count = 4;
    int successful_samples     = 0;
    int remaining_retries      = 100;
    std::array<geometry_msgs::msg::TransformStamped, sample_count> transfer_array;

    while (remaining_retries != 0 && successful_samples < sample_count) {
        try {
            transfer_array[successful_samples] =
                tf_buffer_->lookupTransform(base_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_sec));
            successful_samples++;
            std::this_thread::sleep_for(20ms);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "当前找不到目标物体的TF: %s", ex.what());
            std::this_thread::sleep_for(100ms);
            remaining_retries--;
        }
    }

    if (remaining_retries == 0) {
        return false;
    }

    x = 0.0;
    y = 0.0;
    for (int i = 0; i < sample_count; i++) {
        x += transfer_array[i].transform.translation.x;
        y += transfer_array[i].transform.translation.y;
    }

    x /= static_cast<double>(sample_count);
    y /= static_cast<double>(sample_count);
    return true;
}

geometry_msgs::msg::PoseStamped ArmTaskNode::make_fixed_pitch_pose(double x, double y, double z, double pitch_offset) const {
    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.pose.position.x = x;
    object_pose.pose.position.y = y;
    object_pose.pose.position.z = z;

    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.0 + pitch_offset, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    return object_pose;
}

bool ArmTaskNode::wait_for_stable_vision_target(geometry_msgs::msg::Point& vision_box_pos, double& vision_variance) {
    {
        std::lock_guard<std::mutex> lock(vision_pose_mutex_);
        has_vision_box_pos_ = false;
    }

    vision_variance = std::numeric_limits<double>::infinity();

    if (!vision_model_param_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", vision_model_node_name_.c_str());
        return false;
    }

    bool vision_ready = false;
    vision_model_param_client_->set_parameters({rclcpp::Parameter("start_pnp", true)});

    const auto vision_start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - vision_start_time < 5s) {
        auto variance_future = vision_model_param_client_->get_parameters({"vision_variance"});
        if (variance_future.wait_for(300ms) == std::future_status::ready) {
            const auto params = variance_future.get();
            if (!params.empty()) {
                vision_variance = params.front().as_double();
                RCLCPP_INFO(this->get_logger(), "当前视觉方差: %.6f", vision_variance);

                if (vision_variance < grasp_vision_threshold_variance_) {
                    std::lock_guard<std::mutex> lock(vision_pose_mutex_);
                    if (has_vision_box_pos_) {
                        vision_box_pos = latest_vision_box_pos_;
                        vision_ready   = true;
                        break;
                    }
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "读取视觉方差超时，继续等待");
        }

        std::this_thread::sleep_for(400ms);
    }

    vision_model_param_client_->set_parameters({rclcpp::Parameter("start_pnp", false)});
    return vision_ready;
}

bool ArmTaskNode::wait_for_stable_place_target(geometry_msgs::msg::Point& vision_box_pos, double& vision_variance) {
    {
        std::lock_guard<std::mutex> lock(vision_pose_mutex_);
        has_vision_box_pos_ = false;
    }
    vision_variance = std::numeric_limits<double>::infinity();

    std_msgs::msg::Int32 msg;
    msg.data = 5; // 触发一次放置位置识别
    vision_command_pub_->publish(msg);

    const auto vision_start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - vision_start_time < 3s) {
        {
            std::lock_guard<std::mutex> lock(vision_pose_mutex_);
            if (has_vision_box_pos_) {
                vision_box_pos  = latest_vision_box_pos_;
                vision_variance = 0.0;
                RCLCPP_INFO(
                    this->get_logger(), "获取到放置视觉坐标: x=%.4f, y=%.4f, z=%.4f", vision_box_pos.x, vision_box_pos.y, vision_box_pos.z);
                return true;
            }
        }
        std::this_thread::sleep_for(50ms);
    }

    RCLCPP_WARN(this->get_logger(), "放置位置识别3秒内未获取到结果");
    return false;
}

float ArmTaskNode::get_current_box_hand_dis() {
    if(!get_parameter("enable_retry").as_bool())
        return 0.03f;
    std::lock_guard<std::mutex> lock(box_to_hand_dis_mutex_);
    return current_box_hand_dis;
}

// bool ArmTaskNode::check_success_grasp(std::chrono::steady_clock::duration time_out) {
//     uint64_t start_msg_count = 0;
//     {
//         std::lock_guard<std::mutex> lock(box_to_hand_dis_mutex_);
//         start_msg_count = box_to_hand_dis_msg_count_;
//     }

//     if (!vision_model_param_client_->wait_for_service(1s)) {
//         RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", vision_model_node_name_.c_str());
//         return false;
//     }

//     bool judge_started = false;
//     auto set_judge = [this](bool enabled) {
//         auto future = vision_model_param_client_->set_parameters({rclcpp::Parameter("judge", enabled)});
//         return future.wait_for(500ms) == std::future_status::ready;
//     };

//     if (!set_judge(true)) {
//         RCLCPP_ERROR(this->get_logger(), "Set %s judge=true timeout", vision_model_node_name_.c_str());
//     } else {
//         judge_started = true;
//     }

//     bool success = false;
//     if (judge_started) {
//         const auto start_time = std::chrono::steady_clock::now();
//         while (std::chrono::steady_clock::now() - start_time < time_out && !shutdown_requested_) {
//             bool has_new_msg = false;
//             float box_hand_dis = 0.0f;
//             {
//                 std::lock_guard<std::mutex> lock(box_to_hand_dis_mutex_);
//                 has_new_msg = box_to_hand_dis_msg_count_ > start_msg_count;
//                 box_hand_dis = current_box_hand_dis;
//             }

//             if (has_new_msg) {
//                 success = box_hand_dis < 0.1f;
//                 RCLCPP_INFO(this->get_logger(), "抓取检测距离: %.4f, result=%s", box_hand_dis, success ? "true" : "false");
//                 break;
//             }

//             std::this_thread::sleep_for(20ms);
//         }
//     }

//     if (!set_judge(false)) {
//         RCLCPP_WARN(this->get_logger(), "Set %s judge=false timeout", vision_model_node_name_.c_str());
//     }

//     return success;
// }



void ArmTaskNode::set_air_pump(bool enabled) {
    std_msgs::msg::Int32 msg;
    msg.data = enabled ? 1 : 0;
    air_pub_->publish(msg);
}

void ArmTaskNode::set_initial_arm_state(const std::vector<double>& joint_angles) {
    if (!arm_calc_param_client_->wait_for_service(5s)) {
        RCLCPP_ERROR(
            this->get_logger(), "Parameter service for %s not available, failed to set initial arm state", arm_calc_node_name_.c_str());
        return;
    }

    arm_calc_param_client_->set_parameters({rclcpp::Parameter("current_joint_state", joint_angles)});
    RCLCPP_INFO(this->get_logger(), "Set arm initial state to home_position_");
}


// 关节轨迹规划控制
void ArmTaskNode::execute_joint_space_trajectory(const std::vector<double>& joint_angles, double duration) {
    RCLCPP_INFO(this->get_logger(), "执行关节轨迹规划");

    // Publish joint target
    std_msgs::msg::Float64MultiArray msg;
    msg.data = joint_angles;
    joint_space_target_pub_->publish(msg);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 1)});

        std::this_thread::sleep_for(100ms);

        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

void ArmTaskNode::execute_cartesian_space_trajectory(const geometry_msgs::msg::PoseStamped& target_pose, double duration) {

    RCLCPP_INFO(this->get_logger(), "执行笛卡尔空间轨迹规划");

    // Publish visual target
    visual_target_pub_->publish(target_pose);

    std::this_thread::sleep_for(100ms);

    // Set parameters on arm_calc
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("trajectory_duration", duration), rclcpp::Parameter("motion_mode", 2)});

        std::this_thread::sleep_for(100ms);

        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}

} // namespace arm_task
