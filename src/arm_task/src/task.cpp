#include "arm_task/task.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <robot_msgs/msg/armmode.hpp>
#include <robot_msgs/msg/vis.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

namespace arm_task {

namespace {

constexpr double kVisualServoExitPositionToleranceMeters = 0.015;
constexpr double kVisualServoConvergenceTimeoutSec       = 20.0;

} // namespace

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
    load_arm_positions_from_yaml();

    // Create publishers
    visual_target_pub_      = this->create_publisher<geometry_msgs::msg::PoseStamped>("visual_target_pose", 10);
    joint_space_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_space_target", 10);

    
    // 当发1时通知视觉可以开始全场扫描，当发2时通知视觉可以开始寻找并发布物块坐标使机械臂能够去抓取物块
    arm_vision_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("arm_command", 10);

    // 结束扫描，机械臂需要回到初始位置
    scan_finish_sub_ = this->create_subscription<robot_msgs::msg::Vis>(
        "scan_finish", 10, std::bind(&ArmTaskNode::scan_result_callback, this, std::placeholders::_1));

    // 跟上层控制反馈当前机械臂放置状态，是否放置完成了
    arm_place_finish_pub = this->create_publisher<robot_msgs::msg::Armmode>("arm_cmd_place_state", 10);

    // 跟上层控制反馈当前机械臂状态，是否抓到物块了
    arm_state_pub_1 = this->create_publisher<robot_msgs::msg::Armmode>("arm_cmd_state", 10);

    // 跟上层控制反馈当前机械臂搜索状态，是否找到物块了
    arm_state_pub_2 = this->create_publisher<robot_msgs::msg::Armmode>("arm_search_state", 10);

    // 上层控制命令订阅，告诉机械臂执行哪个任务
    arm_cmd_sub_ = this->create_subscription<robot_msgs::msg::Armmode>(
        "arm_cmd", 10, std::bind(&ArmTaskNode::arm_cmd_callback, this, std::placeholders::_1));

    // 气泵的控制话题，发布机械臂需要的吸取和放置命令
    air_pub_ = this->create_publisher<robot_msgs::msg::Armmode>("air_pump_target", 10);

    //
    red_distance_sub_ = this->create_subscription<robot_msgs::msg::Int>(
        "red_distance", 10, std::bind(&ArmTaskNode::red_distance_callback, this, std::placeholders::_1));


    // 参数服务的回调
    param_callback_ = this->add_on_set_parameters_callback(std::bind(&ArmTaskNode::on_parameters_changed, this, std::placeholders::_1));

    // Create parameter client for arm_calc node
    // 创建一个“远程参数客户端对象”，用于与另一个 ROS2 节点进行参数交互（读取/设置参数） 对arm_calc进行交互控制
    // 具体来说：
    // 1. 调用 std::make_shared 在堆上创建一个 rclcpp::AsyncParametersClient 实例，并返回 shared_ptr
    // 2. 该 client 绑定当前节点（this），意味着它会使用当前节点的通信接口去发起参数请求
    // 3. arm_calc_node_name_ 指定目标节点名称，client 后续所有操作都会发送到这个节点
    // 4. AsyncParametersClient 底层通过 ROS2 的参数服务（/get_parameters, /set_parameters 等）实现跨节点通信
    // 5. “Async” 表示参数请求是异步发送的（不会阻塞当前线程），通常返回 future，可选择等待或忽略结果
    // 6. 创建完成后，可以用 arm_calc_param_client_：
    //    - set_parameters(...) 向目标节点发送参数修改请求
    //    - get_parameters(...) 从目标节点获取参数
    //    - wait_for_service(...) 检查目标节点参数服务是否可用
    // 7. 在本工程中，它被用作“控制通道”：通过修改 arm_calc 节点的参数来触发运动规划/执行等行为
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

// 用于接受视觉放置坐标的函数
void ArmTaskNode::on_place_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    place_target_pose_ = *msg;
    has_place_target_  = true;
    RCLCPP_INFO(this->get_logger(), "Received place target pose");
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

// 这个函数是整个机械臂任务的核心，它根据当前的任务模式（arm_task_mode_）来决定执行哪个具体的任务流程（抓取、放置、搜索等）。它还负责管理任务的状态，
// 确保在一个任务执行过程中不会被新的任务打断，并且在任务完成后重置状态以准备下一次任务。
// 该函数为线性执行函数
void ArmTaskNode::execute_task_state_machine() {

    //current_mode = arm_task_mode_.load(); // 调试时用的，实际跑时应该注释掉，current_mode直接接受回调里的赋值（在最下面）


    // 已经在运行
    if (task_running_) {
        return;
    }

    // 没任务
    if (current_mode == 0) {
        execute_joint_space_trajectory(home_position_, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5 * 1000) + 300));

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
            execute_place_flow_first();

        } else if (current_mode == 3) {
            // 执行第二层放置流程
            RCLCPP_INFO(this->get_logger(), "开始第二层放置任务");
            execute_place_flow_second();
        } else if (current_mode == 4) {
            // 当抓取失败时，上位层控制会发4告诉机械臂去执行抬高机械臂寻找物块的流程，以防止因为初始位置不合适导致相机看不到物块而抓取失败
            RCLCPP_INFO(this->get_logger(), "抓取过程未看到物块，抬高机械臂寻找");
            execute_lift_search();
        } else if (current_mode == 5) {
            // 在比赛开始时，机械臂需要先巡视扫描场地上的物块，确定狗的巡线流程
            RCLCPP_INFO(this->get_logger(), "巡视扫描物块");
            execute_look_for();

        } else if (current_mode >= 10 && current_mode < 20) {
            // Move to position x
            int position_index = current_mode - 10;
            RCLCPP_INFO(this->get_logger(), "Moving to position %d", position_index);
            execute_move_to_position(position_index);
        }

        // Reset mode to standby after completion
        current_mode   = 0;
        arm_task_mode_ = 0;
        this->set_parameter(rclcpp::Parameter("arm_task", 0));

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
    execute_joint_space_trajectory(ready_position, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(0.5 * 1000) + 300));

    std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待机械臂稳定，确保相机准确识别位置

    // 告知相机可以开始读取数据了
    std_msgs::msg::Int32 see_msg;
    see_msg.data = 2;
    arm_vision_command_pub_->publish(see_msg);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m通知视觉可以开始寻找物块了！！！！！！\033[0m");

    float z = 0.22;

    // 2. 等待相机提供物块的位置并转化为机械臂基座坐标系下的坐标
    RCLCPP_INFO(this->get_logger(), "等待相机提供物体位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    int retry_count = 0;
    while (!get_object_pose_in_base_frame(object_pose,z) && retry_count < 30) {
        std::this_thread::sleep_for(100ms);
        retry_count++;
    }

    if (retry_count >= 30) {

        RCLCPP_WARN(this->get_logger(), "未检测到目标，抓取失败");

        robot_msgs::msg::Armmode state_msg;
        state_msg.mode = -1;
        arm_state_pub_1->publish(state_msg);

        RCLCPP_INFO(this->get_logger(), "移动到准备位置");
        execute_joint_space_trajectory(home_position_, 0.5);
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));


        return;
    }

    RCLCPP_INFO(
        this->get_logger(), "物体在坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    // 强制规定姿态
    tf2::Quaternion quat;
    quat.setRPY(0, M_PI / 2.0-0.25, 0);
    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();


    // 3.笛卡尔轨迹规划使机械臂运动到物块的位置
    RCLCPP_INFO(this->get_logger(), "移动到块的位置");
    auto approach_pose = create_approach_pose(object_pose, 0.0);
    execute_cartesian_space_trajectory(approach_pose, 1.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1.5 * 1000) + 500));


    // 通知气泵开始吸了
    RCLCPP_INFO(this->get_logger(), "kaISHI启动气泵");
    robot_msgs::msg::Armmode msg;
    msg.mode = 1;
    air_pub_->publish(msg);


    // 6.回到初始位置
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(grasp_finish_position, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 清状态，以便下位机反馈结果能正确更新状态，从下面开始就是接受三次距离反馈的结果，
    // 如果三次距离都满足条件才认为抓取成功，否则认为抓取失败
    catch_result_.store(0);

    bool red_distance_close = true;

    for (int i = 0; i < 3; i++) {

        int current_distance = red_distance_;

        RCLCPP_INFO(this->get_logger(), "第%d次检测 red_distance = %d", i + 1, current_distance);

        if (current_distance >= 100) {
            red_distance_close = false;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 给上位控制层反馈抓取结果，1为抓到，-1为没抓到
    if (red_distance_close) {

        robot_msgs::msg::Armmode msg;
        msg.mode = 1;
        arm_state_pub_1->publish(msg);

    } else {

        robot_msgs::msg::Armmode msg;
        msg.mode = -1;
        arm_state_pub_1->publish(msg);

        return;
    }

    RCLCPP_INFO(this->get_logger(), "抓取流程完成");
}

void ArmTaskNode::execute_place_flow_first() {


    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position, 0.5);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 300));

    float z = 0.22;

    RCLCPP_INFO(this->get_logger(), "等待TF提供第一层放置位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    int retry_count = 0;
    while (!get_object_pose_in_base_frame(object_pose, z) && retry_count < 30) {
        std::this_thread::sleep_for(100ms);
        retry_count++;
    }

    if (retry_count >= 30) {
        RCLCPP_WARN(this->get_logger(), "未检测到第一层放置目标");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    tf2::Quaternion quat;
    quat.setRPY(0, M_PI/2.0, 0);

    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    RCLCPP_INFO(this->get_logger(), "移动到放置位置");

    auto approach_pose = create_approach_pose(object_pose, 0.0);

    execute_cartesian_space_trajectory(approach_pose, trajectory_duration_);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    robot_msgs::msg::Armmode msg;
    msg.mode = 0;
    air_pub_->publish(msg);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(home_position_, trajectory_duration_);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    robot_msgs::msg::Armmode place_finish_msg;
    place_finish_msg.mode = 1;
    arm_place_finish_pub->publish(place_finish_msg);



    RCLCPP_INFO(this->get_logger(), "第一层放块任务结束");
}


void ArmTaskNode::execute_place_flow_second() {
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(ready_position, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 300));

    float z = 0.36;

    RCLCPP_INFO(this->get_logger(), "等待TF提供第二层放置位姿");
    geometry_msgs::msg::PoseStamped object_pose;
    int retry_count = 0;
    while (!get_object_pose_in_base_frame(object_pose, z) && retry_count < 30) {
        std::this_thread::sleep_for(100ms);
        retry_count++;
    }

    if (retry_count >= 30) {
        RCLCPP_WARN(this->get_logger(), "未检测到第二层放置目标");
        return;
    }

    RCLCPP_INFO(
        this->get_logger(), "放置坐标: [%.3f, %.3f, %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
        object_pose.pose.position.z);

    tf2::Quaternion quat;
    quat.setRPY(0, M_PI/2.0, 0);

    object_pose.pose.orientation.w = quat.getW();
    object_pose.pose.orientation.x = quat.getX();
    object_pose.pose.orientation.y = quat.getY();
    object_pose.pose.orientation.z = quat.getZ();

    RCLCPP_INFO(this->get_logger(), "移动到放置位置");

    auto approach_pose = create_approach_pose(object_pose, 0.0);

    execute_cartesian_space_trajectory(approach_pose, trajectory_duration_);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    RCLCPP_INFO(this->get_logger(), "关闭气泵");

    robot_msgs::msg::Armmode msg;
    msg.mode = 0;
    air_pub_->publish(msg);

    std::this_thread::sleep_for(500ms);

    RCLCPP_INFO(this->get_logger(), "返回初始位置");

    execute_joint_space_trajectory(home_position_, trajectory_duration_);

    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    robot_msgs::msg::Armmode place_finish_msg;
    place_finish_msg.mode = 1;
    arm_place_finish_pub->publish(place_finish_msg);


    RCLCPP_INFO(this->get_logger(), "第二层放块任务结束");
}


void ArmTaskNode::execute_look_for() {

    // 机械臂先预摆到一个合适的位置，方便相机识别全场箱子
    execute_joint_space_trajectory(look_for_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));

    // 告知视觉可以开始全场扫描了
    std_msgs::msg::Int32 scan_msg;
    scan_msg.data = 1;
    arm_vision_command_pub_->publish(scan_msg);

    auto start_time = std::chrono::steady_clock::now();

    // 在这里会阻塞等待视觉发布会扫描完成的消息（scan_finished_被置1），告诉机械臂可以结束等待了
    while (scan_finished_ == 0) {

        // 超时10秒
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(10)) {

            RCLCPP_WARN(this->get_logger(), "\033[1;32mVision scan timeout!\033[0m");

            break;
        }

        std::this_thread::sleep_for(100ms);
    }

    scan_finished_ = 0; // 清状态

    // 机械臂回到初始位置，准备接受后续的抓取指令
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));
}

void ArmTaskNode::execute_lift_search() {
    RCLCPP_INFO(this->get_logger(), "开始搜索任务");

    geometry_msgs::msg::PoseStamped object_pose;

    robot_msgs::msg::Armmode state2_msg;


    if (search_for_object(object_pose)) {

        RCLCPP_INFO(
            this->get_logger(), "找到目标: [%.3f %.3f %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
            object_pose.pose.position.z);

        state2_msg.mode = 1;
        state2_msg.x    = object_pose.pose.position.x;
        state2_msg.y    = object_pose.pose.position.y;
        state2_msg.z    = object_pose.pose.position.z;
        arm_state_pub_2->publish(state2_msg);


    } else {

        RCLCPP_WARN(this->get_logger(), "搜索失败");

        state2_msg.mode = -1;
        arm_state_pub_2->publish(state2_msg);
    }

    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));
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

void ArmTaskNode::execute_visual_servo(const geometry_msgs::msg::PoseStamped& target_pose) {
    RCLCPP_INFO(this->get_logger(), "Executing visual servo");

    // Store target pose for publishing thread
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        target_object_pose_ = target_pose;
        has_object_pose_    = true;
    }

    // Set visual servo parameters
    if (arm_calc_param_client_->wait_for_service(5s)) {
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("motion_mode", 3)});

        std::this_thread::sleep_for(100ms);

        {
            std::lock_guard<std::mutex> lock(visual_servo_state_mutex_);
            visual_servo_result_ready_ = false;
            visual_servo_succeeded_    = false;
        }

        // Start visual servo publishing thread
        visual_servo_active_ = false;
        if (visual_servo_thread_.joinable()) {
            visual_servo_thread_.join();
        }
        visual_servo_active_ = true;
        visual_servo_thread_ = std::thread(&ArmTaskNode::visual_servo_publish_thread, this);

        // Start execution
        arm_calc_param_client_->set_parameters({rclcpp::Parameter("execute_trajectory", true)});
    } else {
        RCLCPP_ERROR(this->get_logger(), "Parameter service for %s not available", arm_calc_node_name_.c_str());
    }
}


bool ArmTaskNode::wait_for_visual_servo_convergence(double position_tolerance_m, double timeout_sec) {
    std::unique_lock<std::mutex> lock(visual_servo_state_mutex_);
    const bool completed = visual_servo_state_cv_.wait_for(lock, std::chrono::duration<double>(timeout_sec), [this]() {
        return visual_servo_result_ready_ || shutdown_requested_ || !visual_servo_active_;
    });

    if (visual_servo_result_ready_) {
        return visual_servo_succeeded_;
    }

    if (!completed) {
        RCLCPP_WARN(this->get_logger(), "视觉伺服在 %.1f s 内未收敛到 %.3f m，主动停止视觉伺服", timeout_sec, position_tolerance_m);
    } else {
        RCLCPP_WARN(this->get_logger(), "视觉伺服在线程结束前未报告收敛结果，主动停止视觉伺服");
    }

    lock.unlock();
    visual_servo_active_ = false;
    visual_servo_state_cv_.notify_all();
    if (visual_servo_thread_.joinable()) {
        visual_servo_thread_.join();
    }

    return false;
}

void ArmTaskNode::visual_servo_publish_thread() {
    RCLCPP_INFO(this->get_logger(), "视觉伺服线程开始执行");

    rclcpp::Rate rate(100); // 100 Hz
    constexpr double kCameraDataLockDistanceMeters                  = 0.35;
    constexpr double kVisualServoConvergencePositionToleranceMeters = kVisualServoExitPositionToleranceMeters;
    bool camera_data_locked                                         = false;
    geometry_msgs::msg::PoseStamped last_trusted_pose;
    bool has_last_trusted_pose = false;

    auto publish_visual_servo_result = [this](bool succeeded) {
        {
            std::lock_guard<std::mutex> lock(visual_servo_state_mutex_);
            if (visual_servo_result_ready_) {
                return;
            }
            visual_servo_result_ready_ = true;
            visual_servo_succeeded_    = succeeded;
        }
        visual_servo_state_cv_.notify_all();
    };

    while (visual_servo_active_ && !shutdown_requested_) {
        geometry_msgs::msg::PoseStamped pose_to_publish;
        bool has_pose = false;

        if (!camera_data_locked) {
            // Try to get current object pose from TF
            try {
                auto target_in_base =
                    tf_buffer_->lookupTransform(base_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
                pose_to_publish.pose.position.x = target_in_base.transform.translation.x;
                pose_to_publish.pose.position.y = target_in_base.transform.translation.y;
                pose_to_publish.pose.position.z = target_in_base.transform.translation.z;
                pose_to_publish.header.frame_id = base_frame_;
                pose_to_publish.header.stamp    = this->now();
                has_pose                        = true;
                last_trusted_pose               = pose_to_publish;
                has_last_trusted_pose           = true;

                auto target_in_camera =
                    tf_buffer_->lookupTransform(camera_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
                const auto& translation = target_in_camera.transform.translation;
                const double distance_to_target =
                    std::sqrt(translation.x * translation.x + translation.y * translation.y + translation.z * translation.z);

                if (distance_to_target < kCameraDataLockDistanceMeters) {
                    camera_data_locked = true;
                    RCLCPP_INFO(
                        this->get_logger(), "camera_data_locked=true, %s 到 %s 距离为 %.3f m", camera_frame_.c_str(), object_frame_.c_str(),
                        distance_to_target);
                }

                RCLCPP_INFO_THROTTLE(
                    get_logger(), *this->get_clock(), 100, "得到目标(%lf, %lf, %lf)", pose_to_publish.pose.position.x,
                    pose_to_publish.pose.position.y, pose_to_publish.pose.position.z);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "获取变换失败: %s", ex.what());
            }
        }

        if (camera_data_locked && has_last_trusted_pose) {
            pose_to_publish = last_trusted_pose;
            has_pose        = true;

            try {
                const auto ee_tf = tf_buffer_->lookupTransform(base_frame_, tip_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
                const double dx  = pose_to_publish.pose.position.x - ee_tf.transform.translation.x;
                const double dy  = pose_to_publish.pose.position.y - ee_tf.transform.translation.y;
                const double dz  = pose_to_publish.pose.position.z - ee_tf.transform.translation.z;
                const double position_error = std::sqrt(dx * dx + dy * dy + dz * dz);

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 500,
                    "视觉伺服位置误差: %.4f m, ee=(%.3f, %.3f, %.3f), locked_target=(%.3f, %.3f, %.3f)", position_error,
                    ee_tf.transform.translation.x, ee_tf.transform.translation.y, ee_tf.transform.translation.z,
                    pose_to_publish.pose.position.x, pose_to_publish.pose.position.y, pose_to_publish.pose.position.z);

                if (position_error < kVisualServoConvergencePositionToleranceMeters) {
                    RCLCPP_INFO(
                        this->get_logger(), "视觉伺服收敛，位置误差 %.4f m 小于阈值 %.4f m", position_error,
                        kVisualServoConvergencePositionToleranceMeters);
                    publish_visual_servo_result(true);
                    visual_servo_active_ = false;
                    break;
                }
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "检查视觉伺服收敛时获取TF失败: %s", ex.what());
            }
        }

        if (!has_pose) {
            // Fall back to stored pose
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (has_object_pose_) {
                pose_to_publish = target_object_pose_;
                has_pose        = true;
            }
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 1.57, 0.0);
        pose_to_publish.pose.orientation.w = q.w();
        pose_to_publish.pose.orientation.x = q.x();
        pose_to_publish.pose.orientation.y = q.y();
        pose_to_publish.pose.orientation.z = q.z();

        if (has_pose) {
            visual_target_pub_->publish(pose_to_publish);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No object pose available for visual servo");
            publish_visual_servo_result(false);
            visual_servo_active_ = false;
            break;
        }

        rate.sleep();
    }

    publish_visual_servo_result(false);

    RCLCPP_INFO(this->get_logger(), "视觉伺服线程结束执行");
}
bool ArmTaskNode::get_object_pose_in_base_frame(geometry_msgs::msg::PoseStamped& pose_out,float z) {
    try {
        // Look up transform from base_link to camera_link
        geometry_msgs::msg::TransformStamped transform =
            tf_buffer_->lookupTransform(base_frame_, object_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));

        // The object is assumed to be at the camera frame origin
        // (in real scenario, you'd get object pose relative to camera)
        pose_out.header.frame_id  = base_frame_;
        pose_out.header.stamp     = this->now();
        pose_out.pose.position.x  = transform.transform.translation.x;
        pose_out.pose.position.y  = transform.transform.translation.y;
        pose_out.pose.position.z  = transform.transform.translation.z;
        pose_out.pose.position.z  = z;
        pose_out.pose.orientation = transform.transform.rotation;
        return true;

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "从 %s 到 %s变换失败: %s", object_frame_.c_str(), base_frame_.c_str(), ex.what());
        return false;
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


void ArmTaskNode::red_distance_callback(const robot_msgs::msg::Int& msg) { red_distance_ = msg.data; }

void ArmTaskNode::scan_result_callback(const robot_msgs::msg::Vis& msg) {
    scan_finished_ = msg.x;
    RCLCPP_INFO(this->get_logger(), "\033[1;34m收到巡视结果: %d\033[0m", scan_finished_);
}


geometry_msgs::msg::PoseStamped ArmTaskNode::create_approach_pose(const geometry_msgs::msg::PoseStamped& target_pose, double distance) {

    geometry_msgs::msg::PoseStamped approach_pose = target_pose;
    approach_pose.pose.position.z += distance;

    return approach_pose;
}


void ArmTaskNode::execute_place_place_rad() {
    // 1. Move to ready position
    RCLCPP_INFO(this->get_logger(), "移动到准备位置");
    execute_joint_space_trajectory(place_position, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000) + 500));


    // 4. Deactivate air pump to release object
    RCLCPP_INFO(this->get_logger(), "关闭气泵");
    robot_msgs::msg::Armmode msg;
    msg.mode = 0;
    air_pub_->publish(msg);
    std::this_thread::sleep_for(500ms);



    // 5. Move back to ready position
    RCLCPP_INFO(this->get_logger(), "返回准备位置");
    execute_joint_space_trajectory(home_position_, trajectory_duration_);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(trajectory_duration_ * 1000)));

    RCLCPP_INFO(this->get_logger(), "任务结束");
}



bool ArmTaskNode::search_for_object(geometry_msgs::msg::PoseStamped& object_pose) {
    RCLCPP_INFO(this->get_logger(), "开始巡视寻找目标");

    std::vector<std::vector<double>> search_positions = {

        look_middle_position_,
        look_left_position_,
        look_right_position_,

    };

    for (const auto& joints : search_positions) {

        // 移动机械臂
        execute_joint_space_trajectory(joints, 4.0);

        std::this_thread::sleep_for(2s);

        // 在当前位置等待视觉
        auto start = std::chrono::steady_clock::now();

        while (rclcpp::ok()) {

            // 检测到目标
            if (get_object_pose_in_base_frame(object_pose,0.0)) {

                RCLCPP_INFO(this->get_logger(), "巡视发现目标");

                // 等待视觉稳定
                std::this_thread::sleep_for(1800ms);

                // 再读取一次稳定值
                if (get_object_pose_in_base_frame(object_pose,0.0)) {

                    RCLCPP_INFO(
                        this->get_logger(), "稳定目标坐标: [%.3f %.3f %.3f]", object_pose.pose.position.x, object_pose.pose.position.y,
                        object_pose.pose.position.z);

                    return true;
                }
            }

            // 当前巡视点等待2秒
            auto now = std::chrono::steady_clock::now();

            if (now - start > 2s) {
                break;
            }

            std::this_thread::sleep_for(100ms);
        }
    }

    return false;
}



void ArmTaskNode::arm_cmd_callback(const robot_msgs::msg::Armmode& msg) {

     current_mode = msg.mode;
}


} // namespace arm_task
