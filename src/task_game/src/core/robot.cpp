#include <algorithm>
#include <array>
#include <chrono>
#include <core/robot.hpp>
#include <ctime>
#include <iomanip>
#include <memory>
#include <thread>
#include <cstdlib>
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

constexpr std::array<const char*, 2> kRlRealNodeNames{"rl_real_atdog2", "rl_real_atdog3"};
constexpr const char* kTargetBoxFrameId = "object_frame";

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
    robot->auto_pilot_enabled = false;
}

}  // namespace

Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    target_box_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    node_->declare_parameter<bool>("start_game", false);
    node_->declare_parameter<bool>("tree_debug_mode", false);
    node_->declare_parameter<bool>("a_use_pnp_box_index", false);
    node_->declare_parameter<bool>("is_first_game", true);
    set_tree_debug_mode(node_->get_parameter("tree_debug_mode").as_bool());
    node_->declare_parameter<std::vector<double>>("transfer_x_limits", {transfer_x_limits_[0], transfer_x_limits_[1]});
    node_->declare_parameter<std::vector<double>>("transfer_y_limits", {transfer_y_limits_[0], transfer_y_limits_[1]});
    node_->declare_parameter<std::vector<double>>("transfer_z_limits", {transfer_z_limits_[0], transfer_z_limits_[1]});

    const auto update_limits = [this](const std::string& name, std::array<double, 2>& target) {
        const auto values = node_->get_parameter(name).as_double_array();
        if (values.size() == 2 && values[0] <= values[1]) {
            target = {values[0], values[1]};
            return;
        }
        RCLCPP_WARN(
            node_->get_logger(),
            "参数 %s 配置无效，保持范围 [%.3f, %.3f]",
            name.c_str(),
            target[0],
            target[1]);
    };

    update_limits("transfer_x_limits", transfer_x_limits_);
    update_limits("transfer_y_limits", transfer_y_limits_);
    update_limits("transfer_z_limits", transfer_z_limits_);

    pilot = std::make_shared<Pilot>(node_);

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        if (msg.just_reconnected) {
            reconnect_ignore_frames_ = kReconnectIgnoreFrames;
            last_key = msg.key;
            RCLCPP_INFO(node_->get_logger(), "遥控器重连恢复，暂时忽略前%d帧模式切换", kReconnectIgnoreFrames);
        }
        const bool ignore_mode_switch = reconnect_ignore_frames_ > 0;
        if (ignore_mode_switch) {
            --reconnect_ignore_frames_;
        }

        // RCLCPP_INFO_THROTTLE(
        //     node_->get_logger(),
        //     *node_->get_clock(),
        //     100,
        //     "接收遥控器输入: key=0x%X",
        //     msg.key);
        if (!ignore_mode_switch) {
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
        }

        if (current_control_mode == 0) {
            if (check_key_trigger(msg.key, 4)) {
                cmd.mode = 1;
                RCLCPP_INFO(node_->get_logger(), "位控站立模式");
            } else if (check_key_trigger(msg.key, 5)) {
                cmd.mode = 2;
                RCLCPP_INFO(node_->get_logger(), "普通行走模式");
            }

            cmd.vy = -static_cast<float>(std::clamp(static_cast<double>(msg.lx) / 1200.0, -0.8, 0.8));
            cmd.vx = static_cast<float>(std::clamp(static_cast<double>(msg.ly) / 1200.0, -1.5, 2.5));
            cmd.vz = -static_cast<float>(std::clamp(static_cast<double>(msg.rx) / 1200.0, -2.5, 2.5));
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
                continue;
            }

            auto update_limit = [&](const char* name, std::array<double, 2>& target) {
                if (param.get_name() != name) {
                    return false;
                }
                const auto values = param.as_double_array();
                if (values.size() != 2 || values[0] > values[1]) {
                    result.successful = false;
                    result.reason = std::string(name) + " 必须是长度为2且按最小值到最大值排列的数组";
                    return true;
                }
                target = {values[0], values[1]};
                return true;
            };

            if (update_limit("transfer_x_limits", transfer_x_limits_)) {
                continue;
            }
            if (update_limit("transfer_y_limits", transfer_y_limits_)) {
                continue;
            }
            update_limit("transfer_z_limits", transfer_z_limits_);

            if(param.get_name() == "start_game")
            {
                start_game = param.as_bool();
            }
        }
        return result;
    });

    control_timer = node_->create_wall_timer(10ms, [this]() {
        geometry_msgs::msg::TransformStamped transfer;
        try {
                transfer = tf_buffer_->lookupTransform("map","base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
                robot_pos_transfer=transfer;
                if (is_position_out_of_bounds(transfer)) {
                    ++out_of_bounds_frames_;
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "检测到机器人位置越界: x=%.3f y=%.3f z=%.3f, 连续越界帧数 %d/%d, 允许范围 x[%.3f, %.3f] y[%.3f, %.3f] z[%.3f, %.3f]",
                        transfer.transform.translation.x,
                        transfer.transform.translation.y,
                        transfer.transform.translation.z,
                        out_of_bounds_frames_,
                        kOutOfBoundsStopFrames,
                        transfer_x_limits_[0],
                        transfer_x_limits_[1],
                        transfer_y_limits_[0],
                        transfer_y_limits_[1],
                        transfer_z_limits_[0],
                        transfer_z_limits_[1]);
                    if (out_of_bounds_frames_ >= kOutOfBoundsStopFrames) {
                        stop_rl_real_nodes();
                    }
                    if (current_control_mode == 1) {
                        set_manual_mode(this);
                        current_control_mode = 0;
                    }
                    cmd_pub_->publish(cmd);
                    return;
                }
                out_of_bounds_frames_ = 0;
                // RCLCPP_INFO_THROTTLE(
                //     node_->get_logger(),
                //     *node_->get_clock(),
                //     1000,
                //     "pos=(%lf,%lf)",
                //     transfer.transform.translation.x,
                //     transfer.transform.translation.y);
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
            publish_active_box_target_tf();
        }
        cmd_pub_->publish(cmd);
    });

    //行为树注册：一轮计划先抓平板箱 box0，再抓手上箱 box1，放置时反过来先放 box1。
    bt.set_context(this);
    bt.rgister(std::make_shared<BT::SequenceNode>("root"));
    bt.rgister(std::make_shared<GeneratePlaneAction>(), "root");
    bt.rgister(std::make_shared<ArriveToBoxAction>(BoxSlot::Box0), "root");
    bt.rgister(std::make_shared<CatchBoxAction>(BoxSlot::Box0), "root");
    bt.rgister(std::make_shared<ArriveToBoxAction>(BoxSlot::Box1), "root");
    bt.rgister(std::make_shared<CatchBoxAction>(BoxSlot::Box1), "root");
    bt.rgister(std::make_shared<ArriveToTargetAction>(BoxSlot::Box1), "root");
    bt.rgister(std::make_shared<PlaceBoxAction>(BoxSlot::Box1), "root");
    bt.rgister(std::make_shared<ArriveToTargetAction>(BoxSlot::Box0), "root");
    bt.rgister(std::make_shared<PlaceBoxAction>(BoxSlot::Box0), "root");
    bt.set_root("root");

    action_thread = std::make_shared<std::thread>([this]() {
        while (rclcpp::ok()) {
            if (auto_pilot_enabled.load()) {
                bt.run();
            }
            //std::this_thread::sleep_for(50ms);
        }
    });
}

Robot::~Robot(){
    if (action_thread && action_thread->joinable())  //子线程退出
        action_thread->join();
}

void Robot::publish_active_box_target_tf() {
    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;
    if (!bt.read_msg("move_plan", move_plan) || !bt.read_msg("plan_index", plan_index)) {
        return;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        return;
    }

    geometry_msgs::msg::TransformStamped target_tf;
    const auto stage = tree_start_key.load();
    const auto& current_plan = move_plan[plan_index];
    const std::array<float, 2>* target_box_pos = nullptr;
    if (stage == kTreeGeneratePlan || stage == kTreeArriveToBox0 || stage == kTreeCatchBox0) {
        target_box_pos = &current_plan.box0.pick_box_pos;
        target_tf.transform.translation.z = 0.25f;
    } else if (stage == kTreeArriveToBox1 || stage == kTreeCatchBox1) {
        target_box_pos = &current_plan.box1.pick_box_pos;
        target_tf.transform.translation.z = 0.25f;
    } else if (stage == kTreeArriveToDst1 || stage == kTreePlaceBox1) {
        target_box_pos = &current_plan.box1.place_box_pos;
        target_tf.transform.translation.z = current_plan.box1.place_at_second_floor ? 0.5f : 0.25f;
    } else if (stage == kTreeArriveToDst0 || stage == kTreePlaceBox0) {
        target_box_pos = &current_plan.box0.place_box_pos;
        target_tf.transform.translation.z = current_plan.box0.place_at_second_floor ? 0.5f : 0.25f;
    } else {
        return;
    }

    target_tf.header.stamp = node_->get_clock()->now();
    target_tf.header.frame_id = "map";
    target_tf.child_frame_id = kTargetBoxFrameId;
    target_tf.transform.translation.x = (*target_box_pos)[0];
    target_tf.transform.translation.y = (*target_box_pos)[1];
    target_tf.transform.rotation.x = 0.0;
    target_tf.transform.rotation.y = 0.0;
    target_tf.transform.rotation.z = 0.0;
    target_tf.transform.rotation.w = 1.0;
    target_box_tf_broadcaster_->sendTransform(target_tf);
}

bool Robot::is_position_out_of_bounds(const geometry_msgs::msg::TransformStamped& transfer) const {
    const auto& translation = transfer.transform.translation;
    return translation.x < transfer_x_limits_[0] || translation.x > transfer_x_limits_[1] ||
           translation.y < transfer_y_limits_[0] || translation.y > transfer_y_limits_[1] ||
           translation.z < transfer_z_limits_[0] || translation.z > transfer_z_limits_[1];
}

void Robot::stop_rl_real_nodes() {
    bool expected = false;
    if (!rl_real_stop_requested_.compare_exchange_strong(expected, true)) {
        return;
    }

    for (const auto* node_name : kRlRealNodeNames) {
        const std::string command = std::string("pkill -f '/rl_sar/.*") + node_name + "$' || pkill -x '" + node_name + "'";
        const int result = std::system(command.c_str());
        if (result == 0) {
            RCLCPP_WARN(node_->get_logger(), "已终止节点 %s", node_name);
            continue;
        }
        RCLCPP_WARN(node_->get_logger(), "未找到运行中的节点 %s", node_name);
    }
}

void Robot::enter_manual_mode() {
    set_manual_mode(this);
    current_control_mode = 0;
}

void Robot::advance_tree_stage() {
    const int32_t stage = tree_start_key.load();
    if (stage < kTreeGeneratePlan || stage > kTreePlaceBox0) {
        tree_start_key = kTreeArriveToBox0;
        return;
    }

    if (stage == kTreeGeneratePlan || stage == kTreePlaceBox0) {
        tree_start_key = kTreeArriveToBox0;
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
