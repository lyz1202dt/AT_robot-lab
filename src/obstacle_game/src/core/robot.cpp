#include <chrono>
#include <cmath>
#include <core/robot.hpp>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <rclcpp/logging.hpp>
#include <sstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>
#include "core/record.hpp"

using namespace std::chrono_literals;

namespace {

std::filesystem::path get_record_directory(const std::string& record_prefix)
{
    const std::filesystem::path prefix_path(record_prefix);
    if (prefix_path.has_parent_path()) {
        return prefix_path.parent_path();
    }
    return std::filesystem::current_path();
}

std::string get_record_prefix_name(const std::string& record_prefix)
{
    const std::filesystem::path prefix_path(record_prefix);
    const auto filename = prefix_path.filename().string();
    return filename.empty() ? "record" : filename;
}

std::string trim_copy(const std::string& value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::vector<std::string> split_scene_paths(const std::string& scene_path)
{
    std::vector<std::string> paths;
    std::stringstream ss(scene_path);
    std::string item;
    while (std::getline(ss, item, ';')) {
        item = trim_copy(item);
        if (!item.empty()) {
            paths.push_back(item);
        }
    }
    return paths;
}

std::vector<std::string> select_scene_yamls(rclcpp::Node::SharedPtr node, const std::string& scene_path, const std::string& record_prefix)
{
    if (!scene_path.empty()) {
        auto paths = split_scene_paths(scene_path);
        RCLCPP_INFO(node->get_logger(), "使用参数指定路径文件数量: %zu", paths.size());
        for (std::size_t i = 0; i < paths.size(); ++i) {
            RCLCPP_INFO(node->get_logger(), "路径%zu: %s", i, paths[i].c_str());
        }
        return paths;
    }

    const auto record_dir = get_record_directory(record_prefix);
    const auto preferred_yaml = record_dir / "obstacle_game.yaml";
    if (std::filesystem::exists(preferred_yaml)) {
        RCLCPP_INFO(node->get_logger(), "使用优先路径文件: %s", preferred_yaml.string().c_str());
        return {preferred_yaml.string()};
    }

    const auto prefix_name = get_record_prefix_name(record_prefix);
    std::filesystem::path latest_yaml;
    std::string latest_name;
    try {
        if (std::filesystem::exists(record_dir)) {
            for (const auto& entry : std::filesystem::directory_iterator(record_dir)) {
                if (!entry.is_regular_file() || entry.path().extension() != ".yaml") {
                    continue;
                }
                const auto name = entry.path().filename().string();
                if (name.rfind(prefix_name, 0) != 0 || name == "obstacle_game.yaml") {
                    continue;
                }
                if (latest_name.empty() || name > latest_name) {
                    latest_name = name;
                    latest_yaml = entry.path();
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "扫描路径文件失败: %s", e.what());
    }

    if (!latest_yaml.empty()) {
        RCLCPP_INFO(node->get_logger(), "未找到obstacle_game.yaml，使用最新录制路径文件: %s", latest_yaml.string().c_str());
        return {latest_yaml.string()};
    }

    RCLCPP_WARN(node->get_logger(), "未找到obstacle_game.yaml或%s*.yaml，自动轨迹为空", prefix_name.c_str());
    return {""};
}

}  // namespace


Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
    node_->declare_parameter<std::string>("scene_path","");
    node_->declare_parameter<std::string>("yaml_file_path","./record");
    node_->declare_parameter<int>("switch_path", 0);

    const auto yaml_paths = select_scene_yamls(
        node_,
        node_->get_parameter("scene_path").as_string(),
        node_->get_parameter("yaml_file_path").as_string());

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    pilots_.reserve(yaml_paths.size());
    for (const auto& yaml_path : yaml_paths) {
        pilots_.push_back(std::make_shared<Pilot>(node_, yaml_path));
    }
    if (pilots_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "未加载任何轨迹Pilot，自动控制不可用");
    }
    record=std::make_shared<Record>(node_);

    policy_done_sub_ = node_->create_subscription<robot_msgs::msg::Int>("policy_done", 10, [this](const robot_msgs::msg::Int& msg) {
        auto pilot = active_pilot();
        if (pilot) {
            pilot->notify_policy_done(msg.data);
        }
    });

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        const bool record_mode_active = check_key_pressed(msg.key, 2);
        const bool record_option_modifier_active = check_key_pressed(msg.key, 9);
        const bool reuse_record_option_keys = record_mode_active && record_option_modifier_active;

        // TODO:处理并发布遥控器数据
        if (!check_key_pressed(msg.key,1)) // 拨杆处于中间位置或向下位置，即手动控制
        {
            if (current_control_mode == 1 && ++manual_switch_request_count_ >= kManualSwitchDebounceFrames) {
                cmd.mode = 1;    // 如果刚才是自动控制，那么切入手动控制时进入位控站立模式(可能是有紧急情况)
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->reset();
                    pilot->stop();
                }
                current_control_mode = 0;
                manual_switch_request_count_ = 0;
                RCLCPP_INFO(node_->get_logger(), "请求切入手动控制");
            }
        }
        else {      //拨杆处于向上位置，切入自动模式
            manual_switch_request_count_ = 0;
            if(current_control_mode==0)
            {
                current_control_mode = 1;
                RCLCPP_INFO(node_->get_logger(), "请求切入自动控制");
            }
        }

        // 只有手动控制模式下可以使用遥控器修改机器人当前使用的策略
        if (current_control_mode == 0) {
            if (check_key_trigger(msg.key,4)) {         //模式控制
                cmd.mode = 1; // 位控站立
                current_record_policy_id = 1;
                RCLCPP_INFO(node_->get_logger(), "getup/位控站立模式");
            } else if (check_key_trigger(msg.key,5)) {
                cmd.mode = 2; // 普通行走策略
                current_record_policy_id = 2;
                RCLCPP_INFO(node_->get_logger(), "walk/普通行走模式");
            } else if (check_key_trigger(msg.key,6)) {
                cmd.mode = 3; // 台阶策略
                current_record_policy_id = 3;
                RCLCPP_INFO(node_->get_logger(), "stairs/台阶模式");
            } else if (check_key_trigger(msg.key,3)) {
                cmd.mode = 4; // 沙地策略
                current_record_policy_id = 4;
                RCLCPP_INFO(node_->get_logger(), "sand/沙地模式");
            } else if (check_key_trigger(msg.key,11)) {
                if (!reuse_record_option_keys) {
                    cmd.mode = 5; // 斜坡策略
                    current_record_policy_id = 5;
                    RCLCPP_INFO(node_->get_logger(), "slope/斜坡模式");
                }
            } else if (check_key_trigger(msg.key,12)) {
                cmd.mode = 6; // 限高杆策略
                current_record_policy_id = 6;
                RCLCPP_INFO(node_->get_logger(), "bar/限高杆模式");
            } else if (check_key_trigger(msg.key,13)) {
                if (!reuse_record_option_keys) {
                    cmd.mode = 7; // 木桥策略
                    current_record_policy_id = 7;
                    RCLCPP_INFO(node_->get_logger(), "bridge/木桥模式");
                }
            } else if (check_key_trigger(msg.key,10)) {
                cmd.mode = 8; // 翻墙状态
                current_record_policy_id = 8;
                RCLCPP_INFO(node_->get_logger(), "cross_wall/翻墙模式");
            }

            //摇杆赋值
            cmd.vy=-(float)std::clamp((double)msg.lx/1200.0,-0.8,0.8);
            cmd.vx=(float)std::clamp((double)msg.ly/1200.0,-1.2,1.2);
            cmd.vz=-(float)std::clamp((double)msg.rx/1200.0,-1.0,1.0);


        } else if (current_control_mode == 1) {
            if (check_key_trigger(msg.key,4)) {     //复位并停止
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->reset();
                    pilot->stop();
                }
            } else if (check_key_trigger(msg.key, 5)) {
                if (!robot_pose_valid_ || !sync_pilot_state_from_transform(robot_pos_transfer)) {
                    RCLCPP_ERROR(node_->get_logger(), "自动轨迹启动失败，尚未获取有效map->base_link位姿");
                } else {
                    auto pilot = active_pilot();
                    if (!pilot || !pilot->start()) {  // 开始执行自动控制
                        RCLCPP_ERROR(node_->get_logger(), "自动轨迹启动失败，请检查scene_path和路径点");
                    }
                }
            } else if (check_key_trigger(msg.key, 6)) {
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->stop();         //  自动控制执行暂停
                }
            }
        }

        
        if(record_mode_active)   //表示开始记录周期
        {
            if(!record_yaml_opened)
            {
                record_yaml_opened=true;
                pending_record_stand_option_ = false;
                pending_record_yaw_lock_option_ = false;
                const auto now = std::chrono::system_clock::now();
                const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                std::tm local_tm{};
                localtime_r(&now_time, &local_tm);

                std::ostringstream oss;
                oss << node_->get_parameter("yaml_file_path").as_string()
                    << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".yaml";
                record->set_output_yaml(oss.str());

                RCLCPP_INFO(node_->get_logger(),"开始记录");
            }

            if (reuse_record_option_keys && check_key_trigger(msg.key, 13)) {
                pending_record_stand_option_ = true;
                RCLCPP_INFO(node_->get_logger(), "下一录制点将写入 stand_at_target=true, stand_duration=2");
            }

            if (reuse_record_option_keys && check_key_trigger(msg.key, 11)) {
                pending_record_yaw_lock_option_ = true;
                RCLCPP_INFO(node_->get_logger(), "下一录制点将写入 constraint_target_yaw=true, allow_y_vel=true");
            }

            if (check_key_trigger(msg.key, 14)) {  //按键按下后记录一次点位
                if (!robot_pose_valid_) {
                    RCLCPP_WARN(node_->get_logger(), "尚未获取有效map->base_link位姿，跳过本次录点");
                    record_key(msg.key);
                    return;
                }
                Record::PathPoint target;
                target.target_pos[0]=robot_pos_transfer.transform.translation.x;
                target.target_pos[1]=robot_pos_transfer.transform.translation.y;

                tf2::Quaternion q;
                q.setW(robot_pos_transfer.transform.rotation.w);
                q.setX(robot_pos_transfer.transform.rotation.x);
                q.setY(robot_pos_transfer.transform.rotation.y);
                q.setZ(robot_pos_transfer.transform.rotation.z);
                double cur_roll, cur_pitch, cur_yaw;
                tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

                target.target_yaw=cur_yaw;
                target.constraint_target_yaw=false;
                target.target_vel=0.01;
                target.max_accelation=0.3;
                target.max_velocity=1.0;
                target.max_omega=1.2;
                target.adjust_min_vel=0.5;
                target.adjust_min_omega=0.4;
                target.allow_start_dir_error=0.05;
                target.allow_final_dir_error=0.05;
                target.allow_final_pos_allow=0.05;
                target.allow_y_vel=false;
                target.trajectory_connection_radius=0.0;
                target.stand_at_target=false;
                target.stand_duration=0.0;
                target.kp={3.0,2.5,2.0};
                target.policy_id=current_record_policy_id;

                if (pending_record_stand_option_) {
                    target.stand_at_target = true;
                    target.stand_duration = 2.0;
                    pending_record_stand_option_ = false;
                }

                if (pending_record_yaw_lock_option_) {
                    target.constraint_target_yaw = true;
                    target.allow_y_vel = true;
                    pending_record_yaw_lock_option_ = false;
                }

                record->record_pos(target);
                RCLCPP_INFO(node_->get_logger(),"记录点位, policy_id=%d", target.policy_id);
            }
        }
        else {
            if(record_yaml_opened)
            {
                record_yaml_opened=false;
                pending_record_stand_option_ = false;
                pending_record_yaw_lock_option_ = false;
                record->finishe_record();
                RCLCPP_INFO(node_->get_logger(),"完成记录");
            }
        }
        if (current_control_mode == 0) {
            manual_switch_request_count_ = 0;
        }

        record_key(msg.key);
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            if (param.get_name() != "switch_path") {
                continue;
            }

            const int path_id = param.as_int();
            if (path_id < 0 || static_cast<std::size_t>(path_id) >= pilots_.size()) {
                RCLCPP_ERROR(node_->get_logger(), "switch_path=%d超过轨迹数量%zu，忽略本次切换", path_id, pilots_.size());
                continue;
            }
            switch_to_path(path_id);
        }
        return result;
    });

    control_timer = node_->create_wall_timer(50ms, [this]() {
        geometry_msgs::msg::TransformStamped transfer;
        try {
            transfer = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.05));
            robot_pos_transfer = transfer;
            robot_pose_valid_ = sync_pilot_state_from_transform(transfer);
            if (!robot_pose_valid_) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "map->base_link位姿数值无效，自动驾驶仪停止运行");
                if (current_control_mode == 1) {
                    auto pilot = active_pilot();
                    if (pilot) {
                        pilot->stop();
                    }
                    cmd.mode = 1;
                    cmd.vx = 0.0f;
                    cmd.vy = 0.0f;
                    cmd.vz = 0.0f;
                    current_control_mode = 0;
                }
                cmd_pub_->publish(cmd);
                return;
            }
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(),
                *node_->get_clock(),
                1000,
                "pos=(%lf,%lf)",
                transfer.transform.translation.x,
                transfer.transform.translation.y);
        } catch (const tf2::TransformException& ex) {
            robot_pose_valid_ = false;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "获取目标 TF 失败，自动驾驶仪停止运行: %s", ex.what());
            if (current_control_mode == 1) {
                auto pilot = active_pilot();
                if (pilot) {
                    pilot->stop();
                }
                cmd.mode = 1;
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = 0.0f;
                current_control_mode = 0;
            }
            cmd_pub_->publish(cmd);
            return;
        }

        if (current_control_mode == 1) {
            // geometry_msgs::msg::TransformStamped transfer;
            // try {
            //     transfer = tf_buffer_->lookupTransform("base_link", "map", tf2::TimePointZero, tf2::durationFromSec(0.05));
            //     robot_pos_transfer=transfer;
            // } catch (const tf2::TransformException& ex) {
            //     RCLCPP_WARN(node_->get_logger(), "获取目标 TF 失败，自动驾驶仪停止运行: %s", ex.what());
            //     current_control_mode = 0;
            //     return;
            // }

            auto pilot = active_pilot();
            if (pilot) {
                if (active_path_id_ != 0 && pilot->get_target_id() < 0) {
                    if (switch_to_path(0)) {
                        node_->set_parameter(rclcpp::Parameter("switch_path", 0));
                        pilot = active_pilot();
                    }
                }
                if (pilot) {
                    cmd = pilot->get_command(std::chrono::high_resolution_clock::now());
                }
            } else {
                cmd.mode = 1;
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = 0.0f;
            }
        }
        cmd_pub_->publish(cmd);
    });
}

bool Robot::sync_pilot_state_from_transform(const geometry_msgs::msg::TransformStamped& transfer)
{
    const auto& translation = transfer.transform.translation;
    const auto& rotation = transfer.transform.rotation;

    if (!std::isfinite(translation.x) || !std::isfinite(translation.y) ||
        !std::isfinite(rotation.x) || !std::isfinite(rotation.y) ||
        !std::isfinite(rotation.z) || !std::isfinite(rotation.w)) {
        return false;
    }

    tf2::Quaternion q;
    q.setW(rotation.w);
    q.setX(rotation.x);
    q.setY(rotation.y);
    q.setZ(rotation.z);
    if (q.length2() <= 1e-12) {
        return false;
    }
    q.normalize();

    double cur_roll, cur_pitch, cur_yaw;
    tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

    if (!std::isfinite(cur_yaw)) {
        return false;
    }

    for (const auto& pilot : pilots_) {
        pilot->set_state(Eigen::Vector2d(translation.x, translation.y), cur_yaw);
    }
    return true;
}

std::shared_ptr<Pilot> Robot::active_pilot() const
{
    if (active_path_id_ < 0 || static_cast<std::size_t>(active_path_id_) >= pilots_.size()) {
        return nullptr;
    }
    return pilots_[active_path_id_];
}

bool Robot::switch_to_path(int path_id)
{
    if (path_id < 0 || static_cast<std::size_t>(path_id) >= pilots_.size()) {
        RCLCPP_ERROR(node_->get_logger(), "目标轨迹编号越界: path_id=%d, paths=%zu", path_id, pilots_.size());
        return false;
    }

    if (path_id == active_path_id_) {
        return true;
    }

    const int previous_path_id = active_path_id_;
    auto previous_pilot = active_pilot();
    if (previous_pilot) {
        previous_pilot->stop();
    }

    if (path_id != 0 && !pilots_.empty() && previous_path_id != 0) {
        pilots_[0]->stop();
    }

    auto target_pilot = pilots_[path_id];
    if (!target_pilot->start()) {
        RCLCPP_ERROR(node_->get_logger(), "轨迹%d启动失败，保持当前轨迹%d", path_id, active_path_id_);
        return false;
    }

    active_path_id_ = path_id;
    RCLCPP_INFO(node_->get_logger(), "切换执行轨迹: %d -> %d", previous_path_id, active_path_id_);
    return true;
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
