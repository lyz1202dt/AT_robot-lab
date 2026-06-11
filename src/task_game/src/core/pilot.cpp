#include <core/pilot.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

constexpr int kStandMode = 1;
constexpr int kWalkMode = 2;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMinDistance = 1e-4;
constexpr double kMinDuration = 0.05;
constexpr double kDefaultAccel = 0.25;
constexpr double kDefaultVelocity = 0.7;
constexpr double kTinyError = 1e-5;

struct MotionSample {
    double position{0.0};
    double velocity{0.0};
    double duration{0.0};
};

MotionSample sample_trapezoid_profile(
    double distance,
    double elapsed,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_acceleration) {
    MotionSample sample;
    if (distance <= kMinDistance) {
        return sample;
    }

    const double vmax = std::max(max_velocity, 0.05);
    const double accel = std::max(max_acceleration, 0.05);
    const double v0 = std::clamp(start_velocity, 0.0, vmax);
    const double v1 = std::clamp(end_velocity, 0.0, vmax);
    const double peak_for_triangle = std::sqrt(std::max(0.0, accel * distance + 0.5 * (v0 * v0 + v1 * v1)));
    const double vpeak = std::min(vmax, std::max({peak_for_triangle, v0, v1}));

    const double accel_time = std::max(0.0, (vpeak - v0) / accel);
    const double decel_time = std::max(0.0, (vpeak - v1) / accel);
    const double accel_dist = (vpeak * vpeak - v0 * v0) / (2.0 * accel);
    const double decel_dist = (vpeak * vpeak - v1 * v1) / (2.0 * accel);
    const double cruise_dist = std::max(0.0, distance - accel_dist - decel_dist);
    const double cruise_time = vpeak > kMinDistance ? cruise_dist / vpeak : 0.0;

    sample.duration = std::max(kMinDuration, accel_time + cruise_time + decel_time);
    const double t = std::clamp(elapsed, 0.0, sample.duration);

    if (t < accel_time) {
        sample.velocity = v0 + accel * t;
        sample.position = v0 * t + 0.5 * accel * t * t;
        return sample;
    }

    if (t < accel_time + cruise_time) {
        const double cruise_elapsed = t - accel_time;
        sample.velocity = vpeak;
        sample.position = accel_dist + vpeak * cruise_elapsed;
        return sample;
    }

    const double decel_elapsed = t - accel_time - cruise_time;
    sample.velocity = std::max(v1, vpeak - accel * decel_elapsed);
    sample.position = accel_dist + cruise_dist + vpeak * decel_elapsed - 0.5 * accel * decel_elapsed * decel_elapsed;

    if (elapsed >= sample.duration) {
        sample.position = distance;
        sample.velocity = v1;
    } else {
        sample.position = std::clamp(sample.position, 0.0, distance);
    }
    return sample;
}

void clamp_translation(Eigen::Vector2d& velocity, double max_velocity) {
    const double limit = std::max(max_velocity, 0.0);
    const double norm = velocity.norm();
    if (limit > 0.0 && norm > limit) {
        velocity *= limit / norm;
    }
}

void apply_minimum(double& command, double error, double minimum) {
    if (std::abs(error) <= kTinyError || minimum <= 0.0) {
        return;
    }
    if (std::abs(command) < minimum) {
        command = std::copysign(minimum, command == 0.0 ? error : command);
    }
}

}  // namespace

Pilot::Pilot(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {
}

Pilot::~Pilot() = default;

double Pilot::normalize_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double Pilot::clamp_abs(double value, double limit) {
    const double abs_limit = std::abs(limit);
    if (abs_limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -abs_limit, abs_limit);
}

Eigen::Vector2d Pilot::world_to_body(const Eigen::Vector2d& vector, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Eigen::Vector2d(c * vector.x() + s * vector.y(), -s * vector.x() + c * vector.y());
}

robot_msgs::msg::Cmd Pilot::stand_command() const {
    robot_msgs::msg::Cmd cmd;
    cmd.mode = kStandMode;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

void Pilot::reset_execution() {
    current_index_ = 0;
    state_ = PilotState::Idle;
    resume_state_ = PilotState::Running;
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = 0.0;
    aiming_done_ = false;
    transition_ = CubicTransition{};
    finished_cb_ = nullptr;
}

void Pilot::begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed) {
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = start_speed;
    segment_start_time_ = time;
    transition_ = CubicTransition{};

    if (current_index_ < targets_.size()) {
        aiming_done_ = targets_[current_index_].allow_y_vel;
    } else {
        aiming_done_ = true;
    }
}

void Pilot::finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time) {
    if (current_index_ + 1 < targets_.size()) {
        const auto previous_target = targets_[current_index_];
        ++current_index_;
        segment_start_pos_ = previous_target.target_pos;
        segment_start_yaw_ = current_yaw_;
        segment_start_speed_ = std::max(0.0f, previous_target.target_vel);
        segment_start_time_ = time;
        transition_ = CubicTransition{};
        aiming_done_ = targets_[current_index_].allow_y_vel;
        state_ = PilotState::Running;
        return;
    }

    state_ = PilotState::Adjusting;
    transition_ = CubicTransition{};
}

bool Pilot::start(std::function<void(int success)> finished_cb) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (targets_.empty()) {
        return false;
    }

    finished_cb_ = std::move(finished_cb);
    const auto now = std::chrono::high_resolution_clock::now();

    if (state_ == PilotState::Paused) {
        state_ = resume_state_ == PilotState::Finished ? PilotState::Running : resume_state_;
        if (state_ == PilotState::Running) {
            begin_current_segment(now, 0.0);
        }
        return true;
    }

    if (state_ == PilotState::Finished || current_index_ >= targets_.size()) {
        current_index_ = 0;
    }

    state_ = PilotState::Running;
    begin_current_segment(now, 0.0);
    return true;
}

bool Pilot::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == PilotState::Running || state_ == PilotState::Adjusting) {
        resume_state_ = state_;
        state_ = PilotState::Paused;
    } else if (state_ != PilotState::Finished) {
        state_ = PilotState::Idle;
    }
    transition_ = CubicTransition{};
    return true;
}

bool Pilot::set_target(const std::vector<TargetPoint>& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_ = target;
    reset_execution();
    return !targets_.empty();
}

bool Pilot::set_target(const TargetPoint& target) {
    return set_target(std::vector<TargetPoint>{target});
}

void Pilot::set_state(const Eigen::Vector2d& pos, const float& yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pos_ = pos;
    current_yaw_ = static_cast<float>(normalize_angle(yaw));
}

robot_msgs::msg::Cmd Pilot::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time) {
    std::function<void(int)> callback_to_call;
    robot_msgs::msg::Cmd cmd = stand_command();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (targets_.empty() || state_ == PilotState::Idle || state_ == PilotState::Paused || state_ == PilotState::Finished ||
            current_index_ >= targets_.size()) {
            return cmd;
        }

        TargetPoint& target = targets_[current_index_];
        cmd.mode = kWalkMode;
        cmd.wheel_vel = 0.0f;

        if (state_ == PilotState::Adjusting) {
            const Eigen::Vector2d pos_error_world = target.target_pos - current_pos_;
            const Eigen::Vector2d pos_error_body = world_to_body(pos_error_world, current_yaw_);
            const double yaw_error = target.constraint_target_yaw ? normalize_angle(target.target_yaw - current_yaw_) : 0.0;
            const bool pos_ok = pos_error_world.norm() <= static_cast<double>(target.allow_final_pos_allow);
            const bool yaw_ok = !target.constraint_target_yaw || std::abs(yaw_error) <= static_cast<double>(target.allow_final_dir_error);

            if (pos_ok && yaw_ok) {
                state_ = PilotState::Finished;
                current_index_ = targets_.size();
                callback_to_call = std::move(finished_cb_);
                cmd = stand_command();
            } else {
                Eigen::Vector2d velocity_body(
                    target.kp.x() * pos_error_body.x(),
                    target.kp.y() * pos_error_body.y());
                apply_minimum(velocity_body.x(), pos_error_body.x(), target.adjust_min_vel);
                apply_minimum(velocity_body.y(), pos_error_body.y(), target.adjust_min_vel);
                clamp_translation(velocity_body, target.max_velocity);

                double omega = target.kp.z() * yaw_error;
                if (target.constraint_target_yaw) {
                    apply_minimum(omega, yaw_error, target.adjust_min_omega);
                }
                omega = clamp_abs(omega, target.max_omega);

                cmd.vx = static_cast<float>(velocity_body.x());
                cmd.vy = static_cast<float>(velocity_body.y());
                cmd.vz = static_cast<float>(omega);
            }
        } else {
            Eigen::Vector2d reference_pos = target.target_pos;
            Eigen::Vector2d reference_vel = Eigen::Vector2d::Zero();
            double reference_yaw = target.constraint_target_yaw ? target.target_yaw : current_yaw_;
            double reference_omega = 0.0;
            double segment_duration = kMinDuration;
            double segment_progress = 1.0;

            if (transition_.active) {
                const double elapsed = std::chrono::duration<double>(time - transition_.start_time).count();
                const double duration = std::max(transition_.duration, kMinDuration);
                const double u = std::clamp(elapsed / duration, 0.0, 1.0);
                const double u2 = u * u;
                const double u3 = u2 * u;
                const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
                const double h10 = u3 - 2.0 * u2 + u;
                const double h01 = -2.0 * u3 + 3.0 * u2;
                const double h11 = u3 - u2;
                const double dh00 = 6.0 * u2 - 6.0 * u;
                const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;
                const double dh01 = -6.0 * u2 + 6.0 * u;
                const double dh11 = 3.0 * u2 - 2.0 * u;

                reference_pos = h00 * transition_.start_pos + h10 * duration * transition_.start_vel +
                                h01 * transition_.end_pos + h11 * duration * transition_.end_vel;
                reference_vel = (dh00 * transition_.start_pos + dh10 * duration * transition_.start_vel +
                                 dh01 * transition_.end_pos + dh11 * duration * transition_.end_vel) /
                                duration;

                const double yaw_delta = normalize_angle(transition_.end_yaw - transition_.start_yaw);
                reference_yaw = normalize_angle(transition_.start_yaw + yaw_delta * u);
                reference_omega = yaw_delta / duration;

                if (elapsed >= duration) {
                    ++current_index_;
                    segment_start_pos_ = transition_.end_pos;
                    segment_start_yaw_ = static_cast<float>(transition_.end_yaw);
                    segment_start_speed_ = transition_.end_vel.norm();
                    segment_start_time_ = time;
                    transition_ = CubicTransition{};
                    aiming_done_ = true;
                    if (current_index_ >= targets_.size()) {
                        state_ = PilotState::Adjusting;
                    }
                }
            } else {
                Eigen::Vector2d segment_vec = target.target_pos - segment_start_pos_;
                double distance = segment_vec.norm();

                if (distance <= kMinDistance) {
                    finish_current_target(time);
                    return cmd;
                }

                Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
                if (distance > kMinDistance) {
                    direction = segment_vec / distance;
                }
                const double path_yaw = std::atan2(direction.y(), direction.x());

                if (!target.allow_y_vel && !aiming_done_) {
                    const double yaw_error = normalize_angle(path_yaw - current_yaw_);
                    if (std::abs(yaw_error) < static_cast<double>(target.allow_start_dir_error)) {
                        aiming_done_ = true;
                        begin_current_segment(time, 0.0);
                        segment_vec = target.target_pos - segment_start_pos_;
                        distance = segment_vec.norm();
                    } else {
                        cmd.vx = 0.0f;
                        cmd.vy = 0.0f;
                        cmd.vz = static_cast<float>(clamp_abs(target.kp.z() * yaw_error, target.max_omega));
                        return cmd;
                    }
                }

                const double elapsed = std::chrono::duration<double>(time - segment_start_time_).count();
                const MotionSample profile = sample_trapezoid_profile(
                    distance,
                    elapsed,
                    segment_start_speed_,
                    std::max(0.0f, target.target_vel),
                    target.max_velocity > 0.0f ? target.max_velocity : kDefaultVelocity,
                    target.max_accelation > 0.0f ? target.max_accelation : kDefaultAccel);

                segment_duration = profile.duration;
                segment_progress = distance > kMinDistance ? std::clamp(profile.position / distance, 0.0, 1.0) : 1.0;
                reference_pos = segment_start_pos_ + direction * profile.position;
                reference_vel = direction * profile.velocity;

                if (target.constraint_target_yaw) {
                    const double yaw_delta = normalize_angle(target.target_yaw - segment_start_yaw_);
                    reference_yaw = normalize_angle(segment_start_yaw_ + yaw_delta * segment_progress);
                    reference_omega = segment_duration > kMinDuration ? yaw_delta / segment_duration : 0.0;
                } else if (!target.allow_y_vel) {
                    reference_yaw = path_yaw;
                } else {
                    reference_yaw = current_yaw_;
                }

                const float connection_radius = target.trajectory_connection_radius;
                if (current_index_ + 1 < targets_.size() && connection_radius > 0.0f &&
                    (target.target_pos - current_pos_).norm() < static_cast<double>(connection_radius)) {
                    const TargetPoint& next_target = targets_[current_index_ + 1];
                    const Eigen::Vector2d next_vec = next_target.target_pos - target.target_pos;
                    const double next_distance = next_vec.norm();
                    if (next_distance > kMinDistance) {
                        const Eigen::Vector2d next_direction = next_vec / next_distance;
                        const double blend_distance = std::min(static_cast<double>(connection_radius), next_distance * 0.5);
                        const double blend_speed = std::max({profile.velocity, static_cast<double>(next_target.target_vel), 0.2});

                        transition_.active = true;
                        transition_.start_time = time;
                        transition_.start_pos = reference_pos;
                        transition_.start_vel = reference_vel;
                        transition_.end_pos = target.target_pos + next_direction * blend_distance;
                        transition_.end_vel = next_direction * std::min(
                            blend_speed,
                            static_cast<double>(next_target.max_velocity > 0.0f ? next_target.max_velocity : kDefaultVelocity));
                        transition_.duration = std::max(
                            kMinDuration,
                            (transition_.end_pos - transition_.start_pos).norm() /
                                std::max(transition_.start_vel.norm(), 0.2));
                        transition_.start_yaw = static_cast<float>(reference_yaw);
                        transition_.end_yaw = next_target.constraint_target_yaw
                                                  ? next_target.target_yaw
                                                  : static_cast<float>(std::atan2(next_direction.y(), next_direction.x()));
                    }
                }

                if (elapsed >= profile.duration && !transition_.active) {
                    finish_current_target(time);
                    return cmd;
                }
            }

            const Eigen::Vector2d pos_error_body = world_to_body(reference_pos - current_pos_, current_yaw_);
            Eigen::Vector2d velocity_body = world_to_body(reference_vel, current_yaw_);
            velocity_body.x() += target.kp.x() * pos_error_body.x();
            velocity_body.y() += target.kp.y() * pos_error_body.y();
            clamp_translation(velocity_body, target.max_velocity);

            const double yaw_error = normalize_angle(reference_yaw - current_yaw_);
            const double omega = clamp_abs(reference_omega + target.kp.z() * yaw_error, target.max_omega);

            cmd.vx = static_cast<float>(velocity_body.x());
            cmd.vy = static_cast<float>(velocity_body.y());
            cmd.vz = static_cast<float>(omega);
        }
    }

    if (callback_to_call) {
        callback_to_call(1);
    }
    return cmd;
}
