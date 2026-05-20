#include <core/pilot.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

constexpr double kEpsilon = 1e-6;
constexpr double kWalkMode=2;

}  // namespace

Pilot::Pilot(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
}

bool Pilot::start(std::function<void(int success)> finished_cb)
{
    if (!has_target_) {
        RCLCPP_WARN(node_->get_logger(), "Pilot尚未设置目标，无法开始执行");
        return false;
    }

    finished_cb_ = std::move(finished_cb);
    is_running_ = true;
    first_run_ = false;
    current_linear_speed_ = 0.0;
    phase_ = ControlPhase::kMoveToPosition;
    return true;
}

bool Pilot::stop()
{
    is_running_ = false;
    first_run_ = false;
    current_linear_speed_ = 0.0;
    phase_ = ControlPhase::kIdle;
    return true;
}

bool Pilot::set_target(const TargetPoint &target)
{
    if (!target_config_valid(target)) {
        return false;
    }

    target_ = target;
    has_target_ = true;
    phase_ = is_running_ ? ControlPhase::kMoveToPosition : ControlPhase::kIdle;
    current_linear_speed_ = 0.0;
    first_run_ = false;

    return true;
}

void Pilot::set_state(const Eigen::Vector2d &pos, const float &yaw)
{
    current_pos_ = pos;
    current_yaw_ = static_cast<double>(yaw);
    has_state_ = true;
}

robot_msgs::msg::Cmd Pilot::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    robot_msgs::msg::Cmd cmd = make_zero_command();

    if (!is_running_ || !has_state_ || !target_available()) {
        return cmd;
    }

    if (!first_run_) {
        last_command_time_ = time;
        first_run_ = true;
    }

    const auto elapsed = time - last_command_time_;
    last_command_time_ = time;

    double dt = std::chrono::duration<double>(elapsed).count();
    if (dt < 0.0) {
        dt = 0.0;
    }
    dt = std::min(dt, 0.1);

    const Eigen::Vector2d pos_err_world = target_.target_pos - current_pos_;
    const double distance = pos_err_world.norm();
    const double cos_yaw = std::cos(current_yaw_);
    const double sin_yaw = std::sin(current_yaw_);

    if (phase_ == ControlPhase::kMoveToPosition && is_position_reached()) {
        if (target_.constraint_target_yaw) {
            phase_ = ControlPhase::kAlignFinalYaw;
            current_linear_speed_ = 0.0;
        } else {
            finish_current_task(1);
            return make_zero_command();
        }
    }

    if (phase_ == ControlPhase::kAlignFinalYaw) {
        if (is_yaw_reached()) {
            finish_current_task(1);
            return make_zero_command();
        }

        cmd.mode = kWalkMode;
        const double final_yaw_err = normalize_angle(static_cast<double>(target_.target_yaw) - current_yaw_);
        cmd.vz = static_cast<float>(compute_limited_omega(final_yaw_err, dt));
        return cmd;
    }

    if (phase_ != ControlPhase::kMoveToPosition) {
        return cmd;
    }

    const double target_heading = std::atan2(pos_err_world.y(), pos_err_world.x());
    const double heading_err = normalize_angle(target_heading - current_yaw_);
    const double forward_err = cos_yaw * pos_err_world.x() + sin_yaw * pos_err_world.y();
    const double lateral_err = -sin_yaw * pos_err_world.x() + cos_yaw * pos_err_world.y();

    Eigen::Vector2d desired_body_vel = Eigen::Vector2d::Zero();
    double desired_yaw = target_heading;

    if (target_.allow_y_vel) {
        const double limited_speed = compute_limited_linear_speed(distance, dt);
        Eigen::Vector2d desired_world_vel(target_.kp.x() * pos_err_world.x(), target_.kp.y() * pos_err_world.y());

        double vel_norm = desired_world_vel.norm();
        if (vel_norm > limited_speed && vel_norm > kEpsilon) {
            desired_world_vel *= (limited_speed / vel_norm);
            vel_norm = desired_world_vel.norm();
        }

        if (distance <= static_cast<double>(target_.allow_final_pos_allow)) {
            const double min_speed = std::max(0.0, static_cast<double>(target_.adjust_min_vel));
            if (vel_norm > kEpsilon && vel_norm < min_speed) {
                desired_world_vel *= (min_speed / vel_norm);
            }
        }

        desired_body_vel.x() = cos_yaw * desired_world_vel.x() + sin_yaw * desired_world_vel.y();
        desired_body_vel.y() = -sin_yaw * desired_world_vel.x() + cos_yaw * desired_world_vel.y();

        if (target_.constraint_target_yaw &&
            distance <= std::max(static_cast<double>(target_.allow_final_pos_allow), kEpsilon)) {
            desired_yaw = static_cast<double>(target_.target_yaw);
        }
    } else {
        const bool should_align_before_move = std::abs(heading_err) > static_cast<double>(target_.allow_start_dir_error);
        if (should_align_before_move) {
            current_linear_speed_ = 0.0;
        } else {
            const double limited_speed = compute_limited_linear_speed(std::max(0.0, forward_err), dt);
            const double raw_vx = target_.kp.x() * forward_err;
            double vx = std::clamp(raw_vx, -limited_speed, limited_speed);
            if (distance <= static_cast<double>(target_.allow_final_pos_allow)) {
                const double min_speed = std::max(0.0, static_cast<double>(target_.adjust_min_vel));
                if (std::abs(vx) > kEpsilon && std::abs(vx) < min_speed) {
                    vx = std::copysign(min_speed, vx);
                }
            }
            desired_body_vel.x() = vx;
        }

        if (distance <= static_cast<double>(target_.allow_final_pos_allow)) {
            desired_body_vel.y() = std::clamp(
                target_.kp.y() * lateral_err,
                -static_cast<double>(target_.max_velocity),
                static_cast<double>(target_.max_velocity));
        }

        if (target_.constraint_target_yaw &&
            distance <= std::max(static_cast<double>(target_.allow_final_pos_allow), kEpsilon)) {
            desired_yaw = static_cast<double>(target_.target_yaw);
        }
    }

    const double body_speed = desired_body_vel.norm();
    if (body_speed > static_cast<double>(target_.max_velocity) && body_speed > kEpsilon) {
        desired_body_vel *= (static_cast<double>(target_.max_velocity) / body_speed);
    }

    cmd.mode = kWalkMode;
    cmd.vx = static_cast<float>(desired_body_vel.x());
    cmd.vy = static_cast<float>(desired_body_vel.y());
    cmd.vz = static_cast<float>(compute_limited_omega(normalize_angle(desired_yaw - current_yaw_), dt));
    cmd.wheel_vel = 0.0f;
    return cmd;
}

bool Pilot::target_available() const
{
    return has_target_;
}

bool Pilot::target_config_valid(const TargetPoint &target) const
{
    if (target.constraint_target_yaw && std::abs(target.target_vel) > kEpsilon) {
        RCLCPP_ERROR(node_->get_logger(), "Pilot目标非法: 约束最终朝向时target_vel必须为0");
        return false;
    }

    if (target.max_velocity < 0.0f || target.max_accelation < 0.0f || target.max_omega < 0.0f ||
        target.allow_final_pos_allow < 0.0f || target.allow_final_dir_error < 0.0f ||
        target.allow_start_dir_error < 0.0f || target.adjust_min_vel < 0.0f || target.adjust_min_omega < 0.0f) {
        RCLCPP_ERROR(node_->get_logger(), "Pilot目标非法: 速度/误差/限幅参数不能为负数");
        return false;
    }

    return true;
}

bool Pilot::is_position_reached() const
{
    return has_state_ &&
           (target_.target_pos - current_pos_).norm() <= static_cast<double>(target_.allow_final_pos_allow);
}

bool Pilot::is_yaw_reached() const
{
    if (!target_.constraint_target_yaw) {
        return true;
    }

    const double yaw_error = normalize_angle(static_cast<double>(target_.target_yaw) - current_yaw_);
    return std::abs(yaw_error) <= static_cast<double>(target_.allow_final_dir_error);
}

void Pilot::finish_current_task(int success)
{
    is_running_ = false;
    first_run_ = false;
    current_linear_speed_ = 0.0;
    phase_ = ControlPhase::kIdle;

    if (finished_cb_) {
        auto cb = std::move(finished_cb_);
        finished_cb_ = nullptr;
        cb(success);
    }
}

double Pilot::compute_limited_linear_speed(double distance, double dt)
{
    const double max_velocity = std::max(0.0, static_cast<double>(target_.max_velocity));
    const double target_velocity = std::max(0.0, static_cast<double>(target_.target_vel));
    const double max_accel = std::max(0.0, static_cast<double>(target_.max_accelation));

    double limited_speed = max_velocity;
    if (max_accel > kEpsilon) {
        const double stop_limited_speed =
            std::sqrt(std::max(0.0, target_velocity * target_velocity + 2.0 * max_accel * distance));
        limited_speed = std::min(limited_speed, stop_limited_speed);
    }

    if (distance <= std::max(static_cast<double>(target_.allow_final_pos_allow), kEpsilon)) {
        limited_speed = std::max(limited_speed, static_cast<double>(target_.adjust_min_vel));
    }

    if (dt > 0.0 && max_accel > kEpsilon) {
        const double max_delta = max_accel * dt;
        if (limited_speed > current_linear_speed_) {
            limited_speed = std::min(limited_speed, current_linear_speed_ + max_delta);
        } else {
            limited_speed = std::max(limited_speed, current_linear_speed_ - max_delta);
        }
    }

    current_linear_speed_ = std::clamp(limited_speed, 0.0, max_velocity);
    return current_linear_speed_;
}

double Pilot::compute_limited_omega(double yaw_error, double) const
{
    const double raw_omega = target_.kp.z() * yaw_error;
    const double max_omega = std::max(0.0, static_cast<double>(target_.max_omega));

    double omega = clamp_abs(raw_omega, max_omega);
    const double min_omega = std::max(0.0, static_cast<double>(target_.adjust_min_omega));
    if (std::abs(yaw_error) > kEpsilon && std::abs(omega) < min_omega) {
        omega = std::copysign(min_omega, yaw_error);
        omega = clamp_abs(omega, max_omega);
    }

    return omega;
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
