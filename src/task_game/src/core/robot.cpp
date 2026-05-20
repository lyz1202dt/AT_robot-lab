#include <algorithm>
#include <array>
#include <chrono>
#include <core/robot.hpp>
#include <ctime>
#include <iomanip>
#include <memory>
#include <thread>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>

#include "nodes/catch_box.hpp"
#include "nodes/generate_plan.hpp"
#include "nodes/handle_plan.hpp"
#include "nodes/place_box.hpp"

using namespace std::chrono_literals;


Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    pilot = std::make_shared<Pilot>(node_);

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        if (!check_key_pressed(msg.key, 1)) {
            if (current_control_mode == 1) {
                cmd.mode = 1;
                pilot->stop();
                current_control_mode = 0;
                RCLCPP_INFO(node_->get_logger(), "请求切入手动控制");
            }
        } else {
            if (current_control_mode == 0) {
                current_control_mode = 1;
                RCLCPP_INFO(node_->get_logger(), "请求切入自动控制");
            }
        }

        if (current_control_mode == 0) {
            if (check_key_trigger(msg.key, 4)) {
                cmd.mode = 1;
                RCLCPP_INFO(node_->get_logger(), "位控站立模式");
            } else if (check_key_trigger(msg.key, 5)) {
                cmd.mode = 2;
                RCLCPP_INFO(node_->get_logger(), "普通行走模式");
            }

            cmd.vy = -static_cast<float>(std::clamp(static_cast<double>(msg.lx) / 1200.0, -1.2, 1.2));
            cmd.vx = static_cast<float>(std::clamp(static_cast<double>(msg.ly) / 1200.0, -1.2, 1.2));
            cmd.vz = -static_cast<float>(std::clamp(static_cast<double>(msg.rx) / 1200.0, -1.0, 1.0));
        } else if (current_control_mode == 1) {
            if (check_key_trigger(msg.key, 4)) {
                pilot->stop();
            } else if (check_key_trigger(msg.key, 5)) {
                // pilot->start();
            }
        }

        record_key(msg.key);
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            (void)param;
            // TODO:处理参数更新
        }
        return result;
    });

    control_timer = node_->create_wall_timer(50ms, [this]() {
        geometry_msgs::msg::TransformStamped transfer;
        try {
                transfer = tf_buffer_->lookupTransform("odom","base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
                robot_pos_transfer=transfer;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(),
                    *node_->get_clock(),
                    1000,
                    "pos=(%lf,%lf)",
                    transfer.transform.translation.x,
                    transfer.transform.translation.y);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node_->get_logger(), "获取目标 TF 失败，自动驾驶仪停止运行: %s", ex.what());
                current_control_mode = 0;
                //return;
            }

        if (current_control_mode == 1) {

            tf2::Quaternion q;
            q.setW(transfer.transform.rotation.w);
            q.setX(transfer.transform.rotation.x);
            q.setY(transfer.transform.rotation.y);
            q.setZ(transfer.transform.rotation.z);
            double cur_roll, cur_pitch, cur_yaw;
            tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

            pilot->set_state(Eigen::Vector2d(transfer.transform.translation.x, transfer.transform.translation.y), cur_yaw);

            cmd = pilot->get_command(std::chrono::high_resolution_clock::now());
        }
        cmd_pub_->publish(cmd);
    });

    //行为树注册
    bt.set_context(this);
    bt.rgister(std::make_shared<BT::SequenceNode>("root"));
    bt.rgister(std::make_shared<GeneratePlaneAction>(), "root");
    bt.rgister(std::make_shared<HandlePlaneAction>(), "root");
    bt.rgister(std::make_shared<CatchBoxAction>(), "root");
    bt.rgister(std::make_shared<PlaceBoxAction>(), "root");
    bt.set_root("root");

    action_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            bt.run();
        }
    });
}

Robot::~Robot(){
    if (action_thread && action_thread->joinable())  //子线程退出
        action_thread->join();
}

bool Robot::check_key_trigger(uint32_t current_key,int index)
{
    bool current_is_true=((current_key>>index)&0x0001);
    bool last_is_false=!((last_key>>index)&0x0001);
    return current_is_true&&last_is_false;
}

bool Robot::check_key_pressed(uint32_t current_key,int index)
{
    return ((current_key>>index)&0x0001);
}

void Robot::record_key(uint32_t current_key)
{
    last_key=current_key;
}
