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

#include "nodes/arrive_to_box.hpp"
#include "nodes/arrive_to_target.hpp"
#include "nodes/catch_box.hpp"
#include "nodes/generate_plan.hpp"
#include "nodes/place_box.hpp"
#include "nodes/lift_search.hpp"
using namespace std::chrono_literals;

namespace {

void set_manual_mode(Robot* robot) {
    robot->cmd.mode = 1;
    robot->cmd.vx = 0.0f;
    robot->cmd.vy = 0.0f;
    robot->cmd.vz = 0.0f;
    robot->pilot->stop();
    robot->bt.write_msg<std::vector<MoveBoxPlan>>("move_plan", {});
    robot->bt.write_msg<int>("plan_index", 0);
    if (const auto node = robot->bt.get_node("generate_plan_action")) {
        if (const auto action = std::dynamic_pointer_cast<GeneratePlaneAction>(node)) {
            action->reset_generated();
        }
    }
    robot->tree_start_key = Robot::kTreeIdle;
    robot->auto_pilot_enabled = true;
}

}  // namespace

Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    node_->declare_parameter<bool>("tree_debug_mode", true);
    set_tree_debug_mode(node_->get_parameter("tree_debug_mode").as_bool());

    pilot = std::make_shared<Pilot>(node_);

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        // RCLCPP_INFO_THROTTLE(
        //     node_->get_logger(),
        //     *node_->get_clock(),
        //     100,
        //     "接收遥控器输入: key=0x%X",
        //     msg.key);
        if (!check_key_pressed(msg.key, 1)) {
            if (current_control_mode == 1) {
                set_manual_mode(this);
                current_control_mode = 0;
                RCLCPP_INFO(node_->get_logger(), "请求切入手动控制");
            }
        } else {
            if (current_control_mode == 0) {
                current_control_mode = 1;
                auto_pilot_enabled = true;
                tree_start_key = kTreeGeneratePlan;
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
            if (tree_debug_mode.load() && check_key_trigger(msg.key, 4)) {
                advance_tree_stage();
            }
        }

        record_key(msg.key);
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            if (param.get_name() == "tree_debug_mode") {
                set_tree_debug_mode(param.as_bool());
            }
        }
        return result;
    });

    control_timer = node_->create_wall_timer(50ms, [this]() {
        geometry_msgs::msg::TransformStamped transfer;
        try {
                transfer = tf_buffer_->lookupTransform("map","base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
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
                if (current_control_mode == 1) {
                    set_manual_mode(this);
                    current_control_mode = 0;
                }
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
    bt.rgister(std::make_shared<ArriveToBoxAction>(), "root");
    bt.rgister(std::make_shared<CatchBoxAction>(), "root");
    bt.rgister(std::make_shared<ArriveToTargetAction>(), "root");
    bt.rgister(std::make_shared<PlaceBoxAction>(), "root");
    bt.set_root("root");

    action_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            if (auto_pilot_enabled.load()) {
                bt.run();
            }
            std::this_thread::sleep_for(50ms);
        }
    });
}

Robot::~Robot(){
    if (action_thread && action_thread->joinable())  //子线程退出
        action_thread->join();
}

void Robot::advance_tree_stage() {
    const int32_t stage = tree_start_key.load();
    if (stage < kTreeGeneratePlan || stage > kTreePlaceBox) {
        tree_start_key = kTreeArriveToBox;
        return;
    }

    if (stage == kTreeGeneratePlan) {
        tree_start_key = kTreeArriveToBox;
        return;
    }

    if (stage == kTreePlaceBox) {
        tree_start_key = kTreeArriveToBox;
        return;
    }

    tree_start_key = stage + 1;
}

void Robot::set_tree_debug_mode(bool enabled) {
    tree_debug_mode = enabled;
    RCLCPP_INFO(node_->get_logger(), "行为树调试模式: %s", enabled ? "开启" : "关闭");
}

bool Robot::is_tree_debug_mode() const {
    return tree_debug_mode.load();
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
