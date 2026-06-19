#include "arm_task/task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <robot_msgs/msg/vis.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace arm_task {

ArmTaskNode::ArmTaskNode(const rclcpp::NodeOptions& options)
    : Node("arm_task", options) {

    RCLCPP_INFO(this->get_logger(), "Initializing ArmTaskNode...");

    // Initialize TF2
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Declare parameters
    this->declare_parameter<int32_t>("arm_task", 0);
    this->declare_parameter<bool>("stop_visual_servo", false);
    this->declare_parameter<double>("trajectory_duration", 3.0);
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("visual_servo_kp", 1.6);
    this->declare_parameter<double>("visual_servo_max_linear_acc", 0.5);
    this->declare_parameter<int>("air_pump_pin", 0);
    this->declare_parameter<std::string>("base_frame", "arm_base_link");
    this->declare_parameter<std::string>("camera_frame", "camera_link");
    this->declare_parameter<std::string>("object_frame", "target_object");
    this->declare_parameter<std::string>("tip_frame", "link5");
    this->declare_parameter<std::string>("arm_calc_node_name", "arm_calc_node");

    // Get parameters
    this->get_parameter("trajectory_duration", trajectory_duration_);
    this->get_parameter("approach_distance", approach_distance_);
    // this->get_parameter("visual_servo_kp", visual_servo_kp_);
    this->get_parameter("visual_servo_max_linear_acc", visual_servo_max_linear_acc_);
    this->get_parameter("air_pump_pin", air_pump_pin_);
    this->get_parameter("base_frame", base_frame_);
    this->get_parameter("camera_frame", camera_frame_);
    this->get_parameter("object_frame", object_frame_);
    this->get_parameter("tip_frame", tip_frame_);
    this->get_parameter("arm_calc_node_name", arm_calc_node_name_);

    // Load arm positions from YAML
    //load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);
    
    // 当发1时通知视觉可以开始全场扫描，当发2时通知视觉可以开始寻找并发布物块坐标使机械臂能够去抓取物块
    vision_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("arm_command", 10);

    // 结束扫描，机械臂需要回到初始位置
    box_grid_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "box_id_grid", 10, [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr /*msg*/) { scan_finished_ = 1; });     //该消息发出说明扫描结束

    // // 跟上层控制反馈当前机械臂状态，是否抓到物块了
    arm_finished_pub = this->create_publisher<std_msgs::msg::Int32>("arm_cmd_state", 10);

    // // 跟上层控制反馈当前机械臂搜索状态，是否找到物块了
    // arm_state_pub_2 = this->create_publisher<robot_msgs::msg::Armmode>("arm_search_state", 10);

    // 上层控制命令订阅，告诉机械臂执行哪个任务
    arm_cmd_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "arm_cmd", 10, [this](const std_msgs::msg::Int32& msg){current_mode=msg.data;});

    // 气泵的控制话题，发布机械臂需要的吸取和放置命令
    air_pub_ = this->create_publisher<std_msgs::msg::Int32>("air_pump_target", 10);

    // 参数服务的回调
    param_callback_ = this->add_on_set_parameters_callback(std::bind(&ArmTaskNode::on_parameters_changed, this, std::placeholders::_1));

    arm_calc_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, arm_calc_node_name_);
    RCLCPP_INFO(this->get_logger(), "Using arm_calc parameter client target: %s", arm_calc_node_name_.c_str());

    // Start task execution thread
    task_thread_ = std::thread(&ArmTaskNode::task_execution_thread, this);

    RCLCPP_INFO(this->get_logger(), "ArmTaskNode initialized successfully");
}

ArmTaskNode::~ArmTaskNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down ArmTaskNode...");
    shutdown_requested_  = true;
    visual_servo_active_ = false;

    if (task_thread_.joinable()) {
        task_thread_.join();
    }

    if (visual_servo_thread_.joinable()) {
        visual_servo_thread_.join();
    }
}

void ArmTaskNode::load_arm_positions_from_yaml() {
    try {
        std::string package_share = ament_index_cpp::get_package_share_directory("arm_task");
        std::string yaml_path     = package_share + "/config/arm_position.yaml";

        YAML::Node config = YAML::LoadFile(yaml_path);

        if (config["arm_positions"]) {
            for (const auto& pos : config["arm_positions"]) {
                int index                  = pos["index"].as<int>();
                std::vector<double> joints = pos["joints"].as<std::vector<double>>();
                arm_positions_[index]      = joints;
                RCLCPP_INFO(this->get_logger(), "Loaded position %d with %zu joints", index, joints.size());
            }
        }


        if (config["place_position"]) {
            place_position_2 = config["place_position"].as<std::vector<double>>();
            RCLCPP_INFO(this->get_logger(), "Loaded place position with %zu joints", place_position_2.size());
        }

        if (config["home_position"]) {
            home_position_ = config["home_position"].as<std::vector<double>>();
            RCLCPP_INFO(this->get_logger(), "Loaded place position with %zu joints", home_position_.size());
        }


    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load arm positions: %s", e.what());
    }
}

rcl_interfaces::msg::SetParametersResult ArmTaskNode::on_parameters_changed(const std::vector<rclcpp::Parameter>& params) {

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto& param : params) {
        if (param.get_name() == "arm_task") {
            int32_t new_mode = param.as_int();
            arm_task_mode_   = new_mode;
            RCLCPP_INFO(this->get_logger(), "Task mode changed to: %d", new_mode);
        } else if (param.get_name() == "stop_visual_servo") {
            bool stop = param.as_bool();
            if (stop) {
                visual_servo_active_ = false;
                RCLCPP_INFO(this->get_logger(), "Visual servo stopped via parameter");
                // Reset parameter back to false
                this->set_parameter(rclcpp::Parameter("stop_visual_servo", false));
            }
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

void ArmTaskNode::execut_pos_record()
{
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position, 0.7);
    std::this_thread::sleep_for(750ms);
}

// 这个函数是整个机械臂任务的核心，它根据当前的任务模式（arm_task_mode_）来决定执行哪个具体的任务流程（抓取、放置、搜索等）。它还负责管理任务的状态，
// 确保在一个任务执行过程中不会被新的任务打断，并且在任务完成后重置状态以准备下一次任务。
// 该函数为线性执行函数
void ArmTaskNode::execute_task_state_machine() {

    //current_mode = arm_task_mode_.load(); // 调试时用的，实际跑时应该注释掉，current_mode直接接受回调里的赋值（在最下面）


    // 已经在运行
    if (task_running_) {
        return;
    }

    // 进入BUSY状态
    task_running_ = true;

    try {
        if (current_mode == 1) {
            // 执行抓块流程
            RCLCPP_INFO(this->get_logger(), "开始抓取任务");
            execute_grasp_flow();

        } else if (current_mode == 2) {
            // 执行第一层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第一层放置任务");
            execute_place_flow_1();
        } 
        else if (current_mode == 3) {
            // 执行第二层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第二层放置任务");
            execute_place_flow_2();
        } 
        else if (current_mode == 4) {
            // 当抓取失败时，上位层控制会发4告诉机械臂去执行抬高机械臂寻找物块的流程，以防止因为初始位置不合适导致相机看不到物块而抓取失败
            RCLCPP_INFO(this->get_logger(), "抓取过程未看到物块，抬高机械臂寻找");
            execute_lift_search();
        } else if (current_mode == 5) {
            // 在比赛开始时，机械臂需要先巡视扫描场地上的物块，确定狗的巡线流程
            RCLCPP_INFO(this->get_logger(), "巡视扫描物块");
            execute_look_for();
        }
        else if(current_mode==6)
        {
            RCLCPP_INFO(this->get_logger(), "调试录点模式");
            execut_pos_record();
        }
        else if (current_mode >= 10 && current_mode < 20) {
            // Move to position x
            int position_index = current_mode - 10;
            RCLCPP_INFO(this->get_logger(), "Moving to position %d", position_index);
            execute_move_to_position(position_index);
        }

        if(current_mode)
        {
            current_mode   = 0;
            arm_task_mode_ = 0;
            this->set_parameter(rclcpp::Parameter("arm_task", 0));
        }
        else {
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
void ArmTaskNode::execute_grasp_flow() {

    // 机械臂先预摆到一个合适的位置，方便相机观察和后续运动
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position, 0.3);
    std::this_thread::sleep_for(350ms);

    std::array<geometry_msgs::msg::TransformStamped,8>  transfer_array;
    int i=0;
    int count=100;      //等10s
     do{
    try{
        transfer_array[i]=tf_buffer_->lookupTransform("arm_base_link","object_frame",tf2::TimePointZero, tf2::durationFromSec(0.02));
        i++;
        std::this_thread::sleep_for(20ms);
    } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "当前找不到目标物体的TF: %s", ex.what());
            std::this_thread::sleep_for(100ms);
            count--;
    }
    }while(count!=0&&i<8);

    if(count==0)
    {
        return ;
    }

    double x=0,y=0;
    for(int i=0;i<8;i++)
    {
        x+=transfer_array[i].transform.translation.x;
        y+=transfer_array[i].transform.translation.y;
    }
    x/=8.0;
    y/=8.0;

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.pose.position.x=x;
    object_pose.pose.position.y=y;
    object_pose.pose.position.z=-0.16;
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.0-0.25, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();


    // 3.笛卡尔轨迹规划使机械臂运动到物块的位置
    RCLCPP_INFO(this->get_logger(), "移动到块的位置");
    execute_cartesian_space_trajectory(object_pose, 0.5);
    std::this_thread::sleep_for(300ms);


    // 通知气泵开始吸了
    RCLCPP_INFO(this->get_logger(), "启动气泵");
    std_msgs::msg::Int32 msg;
    msg.data = 1;
    air_pub_->publish(msg);

    std::this_thread::sleep_for(600ms);


    // 6.回到初始位置
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(grasp_finish_position, 1.5);

    std::this_thread::sleep_for(500ms);

    std_msgs::msg::Int32 ret;
    ret.data=1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(1000ms);

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_place_flow_1() {

    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position, 2.0);
    std::this_thread::sleep_for(2100ms);

    std::array<geometry_msgs::msg::TransformStamped,8>  transfer_array;
    int i=0;
    int count=100;      //等10s
     do{
    try{
        transfer_array[i]=tf_buffer_->lookupTransform("arm_base_link","object_frame",tf2::TimePointZero, tf2::durationFromSec(0.08));
        i++;
        std::this_thread::sleep_for(20ms);
    } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "当前找不到目标物体的TF: %s", ex.what());
            std::this_thread::sleep_for(100ms);
            count--;
    }
    }while(count!=0&&i<8);

    if(count==0)
    {
        RCLCPP_INFO(get_logger(),"找不到要放置的目标");
        return ;
    }

    double x=0,y=0;
    for(int i=0;i<8;i++)
    {
        x+=transfer_array[i].transform.translation.x;
        y+=transfer_array[i].transform.translation.y;
    }
    x/=8.0;
    y/=8.0;

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.pose.position.x=x;
    object_pose.pose.position.y=y;
    object_pose.pose.position.z=-0.10;
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.0-0.25, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, 1.0);

    std::this_thread::sleep_for(1000ms);

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    std_msgs::msg::Int32 msg;
    msg.data = 0;
    air_pub_->publish(msg);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(home_position_, 0.5);

    std::this_thread::sleep_for(200ms);

    std_msgs::msg::Int32 ret;
    ret.data=1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(300ms);

    RCLCPP_INFO(this->get_logger(), "第一层放块任务结束");
}

void ArmTaskNode::execute_place_flow_2() {

    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position_2, 2.0);
    std::this_thread::sleep_for(2100ms);

    std::array<geometry_msgs::msg::TransformStamped,8>  transfer_array;
    int i=0;
    int count=100;      //等10s
     do{
    try{
        transfer_array[i]=tf_buffer_->lookupTransform("arm_base_link","object_frame",tf2::TimePointZero, tf2::durationFromSec(0.08));
        i++;
        std::this_thread::sleep_for(20ms);
    } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(get_logger(), "当前找不到目标物体的TF: %s", ex.what());
            std::this_thread::sleep_for(100ms);
            count--;
    }
    }while(count!=0&&i<8);

    if(count==0)
    {
        RCLCPP_INFO(get_logger(),"找不到要放置的目标");
        return ;
    }

    double x=0,y=0;
    for(int i=0;i<8;i++)
    {
        x+=transfer_array[i].transform.translation.x;
        y+=transfer_array[i].transform.translation.y;
    }
    x/=8.0;
    y/=8.0;

    // 强制规定姿态
    geometry_msgs::msg::PoseStamped object_pose;
    object_pose.pose.position.x=x;
    object_pose.pose.position.y=y;
    object_pose.pose.position.z=0.15;
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.0-0.25, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    execute_cartesian_space_trajectory(object_pose, 1.0);

    std::this_thread::sleep_for(1000ms);

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    std_msgs::msg::Int32 msg;
    msg.data = 0;
    air_pub_->publish(msg);

    std::this_thread::sleep_for(200ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(place_position_2, 0.2);

    std::this_thread::sleep_for(200ms);

    std_msgs::msg::Int32 ret;
    ret.data=1;
    arm_finished_pub->publish(ret);

    std::this_thread::sleep_for(500ms);     //等待狗子离开

    execute_joint_space_trajectory(home_position_, 0.5);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "第二层放块任务结束");
}


void ArmTaskNode::execute_look_for() {

    // 告知视觉开始全场扫描了
    std_msgs::msg::Int32 scan_msg;
    scan_msg.data = 1;
    vision_command_pub_->publish(scan_msg);

    // 机械臂先预摆到一个合适的位置，方便相机识别全场箱子
    execute_joint_space_trajectory(look_for_position_, 1.5);
    std::this_thread::sleep_for(1500ms);

    auto start_time = std::chrono::steady_clock::now();

    auto start_joint_pos=look_for_position_;
    auto stop_joint_pos=look_for_position_;
    start_joint_pos[0]=-0.4;
    stop_joint_pos[0]=0.4;

    // 在这里会阻塞等待视觉发布会扫描完成的消息（scan_finished_被置1），告诉机械臂可以结束等待了
    while (scan_finished_ == 0) {
        if (std::chrono::steady_clock::now() - start_time > 5s) {
           execute_joint_space_trajectory(start_joint_pos, 0.2);    //机械臂旋转，执行扫描
           std::this_thread::sleep_for(200ms);
           execute_joint_space_trajectory(stop_joint_pos, 8.0);
           std::this_thread::sleep_for(8s);
           RCLCPP_INFO(get_logger(),"搜索箱子...");
        }

        std::this_thread::sleep_for(100ms);
    }

    scan_finished_ = 0; // 清状态

    // 机械臂回到初始位置，准备接受后续的抓取指令
    execute_joint_space_trajectory(home_position_, 0.5);
    std::this_thread::sleep_for(500ms);
}

void ArmTaskNode::execute_lift_search() {
    RCLCPP_INFO(this->get_logger(), "开始搜索任务");

    // geometry_msgs::msg::PoseStamped object_pose;

    // robot_msgs::msg::Armmode state2_msg;


    // if (search_for_object(object_pose)) {

    //     RCLCPP_INFO(
    //         this->get_logger(), "找到目标: [%.3f %.3f %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
    //         object_pose.pose.position.z);

    //     state2_msg.mode = 1;
    //     state2_msg.x    = object_pose.pose.position.x;
    //     state2_msg.y    = object_pose.pose.position.y;
    //     state2_msg.z    = object_pose.pose.position.z;
    //     arm_state_pub_2->publish(state2_msg);


    // } else {

    //     RCLCPP_WARN(this->get_logger(), "搜索失败");

    //     state2_msg.mode = -1;
    //     arm_state_pub_2->publish(state2_msg);
    // }
    RCLCPP_INFO(this->get_logger(), "搜索任务结束");
    //execute_joint_space_trajectory(home_position_, 1.5);
    std::this_thread::sleep_for(1500ms);
    current_mode = 0;
}



void ArmTaskNode::execute_move_to_position(int position_index) {
    if (arm_positions_.find(position_index) == arm_positions_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Position %d not found in configuration", position_index);
        return;
    }

    const auto& joint_angles = arm_positions_[position_index];
    RCLCPP_INFO(this->get_logger(), "Moving to position %d", position_index);
    execute_joint_space_trajectory(joint_angles, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "Move to position %d completed", position_index);
}

void ArmTaskNode::stop_arm_motion() {
    RCLCPP_INFO(this->get_logger(), "停止机械臂运动");
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", false)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
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

void ArmTaskNode::set_parameter_on_remote_node(
    const std::string& node_name, const std::string& /* param_name */, const rclcpp::Parameter& param) {
    auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, node_name);

    if (param_client->wait_for_service(1s)) {
        param_client->set_parameters({param});
    } else {
        RCLCPP_WARN(this->get_logger(), "Parameter service for %s not available", node_name.c_str());
    }
}

}//namespace
