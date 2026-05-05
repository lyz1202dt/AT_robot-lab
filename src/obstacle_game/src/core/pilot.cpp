#include <core/pilot.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <exception>

namespace {

constexpr double kMaxYawRate = 1.5;

double read_required_double(const YAML::Node &node, const std::string &key)
{
    return node[key].as<double>();
}

}  // namespace

Pilot::Pilot(rclcpp::Node::SharedPtr node_, const std::string yaml_path)
    : node_(std::move(node_))
{
    if (!yaml_path.empty()) {
        load_paths(yaml_path);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Pilot未提供yaml路径，轨迹为空");
    }

    reset();
}

Pilot::~Pilot() = default;

bool Pilot::start()
{
    if (paths_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "轨迹为空，无法开始执行");
        return false;
    }

    if (is_finished_) {
        current_path_index_ = 0;
        reset_segment_progress();
        is_finished_ = false;
        current_linear_speed_ = 0.0;
    }

    is_running_ = true;
    first_run = false;
    return true;
}

bool Pilot::stop()
{
    is_running_ = false;
    current_linear_speed_ = 0.0;
    first_run = false;
    return true;
}

bool Pilot::reset()
{
    current_path_index_ = 0;
    is_running_ = false;
    is_finished_ = paths_.empty();
    current_linear_speed_ = 0.0;
    first_run = false;
    reset_segment_progress();
    return !paths_.empty();
}

void Pilot::set_state(const Eigen::Vector2d &pos, const double &yaw)
{
    current_pos_ = pos;
    current_yaw_ = yaw;
    has_state_ = true;
}

bool Pilot::get_current_path_info(uint32_t &path_num, float &time_rate)
{
    if (paths_.empty()) {
        return false;
    }

    if (is_finished_) {
        path_num = static_cast<uint32_t>(paths_.size() - 1);
        time_rate = 1.0f;
        return true;
    }

    path_num = static_cast<uint32_t>(current_path_index_);

    if (!has_state_) {
        time_rate = 0.0f;
        return true;
    }

    if (!segment_initialized_) {
        segment_start_pos_ = current_pos_;
        segment_start_distance_ = (paths_[current_path_index_].target_pos - segment_start_pos_).norm();
        segment_initialized_ = true;
    }

    if (segment_start_distance_ <= 1e-6) {
        time_rate = 1.0f;
        return true;
    }

    const double remain_distance = (paths_[current_path_index_].target_pos - current_pos_).norm();
    const double progress = std::clamp(1.0 - remain_distance / segment_start_distance_, 0.0, 1.0);
    time_rate = static_cast<float>(progress);
    return true;
}

robot_msgs::msg::Cmd Pilot::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    robot_msgs::msg::Cmd cmd = make_zero_command();

    if (paths_.empty() || !has_state_ || !is_running_ || is_finished_) {
        return cmd;
    }

    if (current_path_index_ >= paths_.size()) {
        is_running_ = false;
        is_finished_ = true;
        return cmd;
    }

    if (!first_run) {
        last_command_time_ = time;
        first_run = true;
    }

    if (!segment_initialized_) {
        segment_start_pos_ = current_pos_;
        segment_start_distance_ = (paths_[current_path_index_].target_pos - segment_start_pos_).norm();
        segment_initialized_ = true;
    }

    if (advance_if_current_target_reached()) {
        return make_zero_command();
    }

    const PathPoint &path = paths_[current_path_index_];
    const auto elapsed = time - last_command_time_;
    last_command_time_ = time;

    double dt = std::chrono::duration<double>(elapsed).count();
    if (dt < 0.0) {
        dt = 0.0;
    }
    dt = std::min(dt, 0.1);

    const Eigen::Vector2d pos_err = path.target_pos - current_pos_;
    const double distance = pos_err.norm();

    if (distance <= path.err_allow) {
        if (advance_if_current_target_reached()) {
            return make_zero_command();
        }
        return cmd;
    }

    const double target_heading = std::atan2(pos_err.y(), pos_err.x());
    const double yaw_err = normalize_angle(target_heading - current_yaw_);

    const Eigen::Vector2d segment_direction = get_segment_direction(path);
    const double feedforward_speed = compute_feedforward_speed(path, distance, dt);
    const Eigen::Vector2d feedforward_world_vel = segment_direction * feedforward_speed;
    const Eigen::Vector2d feedback_world_vel(path.kp.x() * pos_err.x(), path.kp.y() * pos_err.y());
    Eigen::Vector2d desired_world_vel = feedforward_world_vel + feedback_world_vel;

    const double desired_speed = desired_world_vel.norm();
    if (desired_speed > path.max_velocity && desired_speed > 1e-6) {
        desired_world_vel *= (path.max_velocity / desired_speed);
    }

    const bool need_heading_alignment = std::abs(yaw_err) > path.allow_start_dir_error;
    if (need_heading_alignment) {
        desired_world_vel.setZero();
        current_linear_speed_ = 0.0;
    }

    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);
    cmd.vx = static_cast<float>(cos_yaw * desired_world_vel.x() + sin_yaw * desired_world_vel.y());
    cmd.vy = static_cast<float>(-sin_yaw * desired_world_vel.x() + cos_yaw * desired_world_vel.y());
    cmd.vz = static_cast<float>(clamp_abs(path.kp.z() * yaw_err, kMaxYawRate));
    cmd.mode = path.policy_id;
    cmd.wheel_vel = 0.0f;

    return cmd;
}

bool Pilot::load_paths(const std::string &yaml_path)
{
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node paths_node = root["paths"];
        if (!paths_node || !paths_node.IsSequence()) {
            RCLCPP_ERROR(node_->get_logger(), "yaml文件%s缺少paths数组", yaml_path.c_str());
            return false;
        }

        paths_.clear();
        paths_.reserve(paths_node.size());
        for (const auto &path_node : paths_node) {
            PathPoint point;
            point.policy_id = path_node["policy_id"].as<int32_t>();
            point.target_pos.x() = read_required_double(path_node["target_pos"], "x");
            point.target_pos.y() = read_required_double(path_node["target_pos"], "y");
            point.target_vel = path_node["target_vel"].as<double>();
            point.max_velocity = path_node["max_velocity"].as<double>();
            point.max_accelation = path_node["max_accelation"].as<double>();
            point.kp.x() = read_required_double(path_node["kp"], "x");
            point.kp.y() = read_required_double(path_node["kp"], "y");
            point.kp.z() = read_required_double(path_node["kp"], "yaw");
            point.allow_start_dir_error = path_node["allow_start_dir_error"].as<double>();
            point.err_allow = path_node["err_allow"].as<double>();
            point.adjust_min_vel = path_node["adjust_min_vel"].as<double>();
            paths_.push_back(point);
        }

        RCLCPP_INFO(node_->get_logger(), "成功加载%zu个导航点: %s", paths_.size(), yaml_path.c_str());
        return !paths_.empty();
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "加载导航点失败: %s, error: %s", yaml_path.c_str(), e.what());
        paths_.clear();
        return false;
    }
}

void Pilot::reset_segment_progress()
{
    segment_start_pos_.setZero();
    segment_start_distance_ = 0.0;
    segment_initialized_ = false;
}

Eigen::Vector2d Pilot::get_segment_direction(const PathPoint &path) const
{
    Eigen::Vector2d direction = path.target_pos - segment_start_pos_;
    if (direction.norm() <= 1e-6) {
        direction = path.target_pos - current_pos_;
    }
    if (direction.norm() <= 1e-6) {
        return Eigen::Vector2d::Zero();
    }
    return direction.normalized();
}

double Pilot::compute_feedforward_speed(const PathPoint &path, double distance, double dt)
{
    double target_speed = path.max_velocity;
    const double stop_limited_speed = std::sqrt(
        std::max(0.0, path.target_vel * path.target_vel + 2.0 * path.max_accelation * distance));
    target_speed = std::min(target_speed, stop_limited_speed);

    if (distance <= 2.0 * path.err_allow) {
        target_speed = std::max(target_speed, path.adjust_min_vel);
    }

    if (dt > 0.0) {
        const double max_speed_step = path.max_accelation * dt;
        if (target_speed > current_linear_speed_) {
            target_speed = std::min(target_speed, current_linear_speed_ + max_speed_step);
        } else {
            target_speed = std::max(target_speed, current_linear_speed_ - max_speed_step);
        }
    }

    current_linear_speed_ = std::clamp(target_speed, 0.0, path.max_velocity);
    return current_linear_speed_;
}

bool Pilot::advance_if_current_target_reached()
{
    while (current_path_index_ < paths_.size()) {
        const double distance = (paths_[current_path_index_].target_pos - current_pos_).norm();
        if (distance > paths_[current_path_index_].err_allow) {
            return false;
        }

        current_linear_speed_ = paths_[current_path_index_].target_vel;
        ++current_path_index_;
        reset_segment_progress();

        if (current_path_index_ < paths_.size()) {
            segment_start_pos_ = current_pos_;
            segment_start_distance_ = (paths_[current_path_index_].target_pos - segment_start_pos_).norm();
            segment_initialized_ = true;
        }
    }

    is_running_ = false;
    is_finished_ = true;
    current_linear_speed_ = 0.0;
    first_run = false;
    return true;
}

robot_msgs::msg::Cmd Pilot::make_zero_command() const
{
    robot_msgs::msg::Cmd cmd;
    cmd.mode = 1;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

double Pilot::normalize_angle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double Pilot::clamp_abs(double value, double limit)
{
    return std::clamp(value, -limit, limit);
}
