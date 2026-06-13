#include <core/pilot.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace {

// Cmd.mode 的约定来自现有控制链：1=位控站立，2=普通行走。
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

// 梯形/三角速度规划采样：给定路程和时间，返回当前参考路程、速度和总时长。
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
    // 距离不够加速到 vmax 时自动退化为三角速度曲线。
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
    // 补偿执行器死区：有误差时，命令幅值不能小于最小可动速度。
    if (std::abs(command) < minimum) {
        command = std::copysign(minimum, command == 0.0 ? error : command);
    }
}

void apply_translation_minimum(Eigen::Vector2d& velocity, const Eigen::Vector2d& error, double minimum) {
    if (error.norm() <= kTinyError || minimum <= 0.0) {
        return;
    }

    const double norm = velocity.norm();
    if (norm > minimum) {
        return;
    }

    if (norm > kTinyError) {
        velocity *= minimum / norm;
    } else {
        velocity = error.normalized() * minimum;
    }
}

bool translation_is_above_minimum(const Eigen::Vector2d& velocity, double minimum) {
    return minimum > 0.0 && velocity.norm() > minimum;
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

Eigen::Vector2d Pilot::world_to_body(const Eigen::Vector2d& vector, double yaw)
{
    // 地图系误差/速度旋转到机器人机体系，最终 Cmd.vx/vy 使用机体系。
    Eigen::Rotation2Dd R(-yaw);
    return R * vector;
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
    // 每段轨迹都以当前机器人状态为起点，避免 stop/start 后继续追旧时间轴。
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = start_speed;
    segment_start_time_ = time;
    transition_ = CubicTransition{};

    if (current_index_ < targets_.size()) {
        // allow_y_vel=true 可直接边走边转；false 则需要先进入瞄准阶段。
        aiming_done_ = targets_[current_index_].allow_y_vel;
    } else {
        aiming_done_ = true;
    }
}

void Pilot::finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time) {
    if (current_index_ + 1 < targets_.size()) {
        const auto previous_target = targets_[current_index_];
        ++current_index_;
        // 中间目标点不做最终微调，直接切到下一段；最终目标才进入 Adjusting。
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
        // stop 只暂停执行，不清空轨迹；再次 start 从当前进度继续。
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
        // 记录暂停前状态，get_command 会因为 Paused 输出站立零速。
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
            // 规划时间结束后进入微调：只用当前位置误差闭环，直到满足最终容差。
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
                Eigen::Vector2d velocity_body = Eigen::Vector2d::Zero();
                bool translation_above_minimum = false;
                if (!pos_ok) {
                    velocity_body.x() = target.kp.x() * pos_error_body.x();
                    velocity_body.y() = target.kp.y() * pos_error_body.y();
                    clamp_translation(velocity_body, target.max_velocity);
                    translation_above_minimum = translation_is_above_minimum(velocity_body, target.adjust_min_vel);
                    if (!translation_above_minimum) {
                        apply_translation_minimum(velocity_body, pos_error_body, target.adjust_min_vel);
                        clamp_translation(velocity_body, target.max_velocity);
                    }
                }

                double omega = 0.0;
                if (target.constraint_target_yaw && !yaw_ok && (target.allow_y_vel || pos_ok)) {
                    omega = target.kp.z() * yaw_error;
                    if (!translation_above_minimum) {
                        apply_minimum(omega, yaw_error, target.adjust_min_omega);
                    }
                }
                omega = clamp_abs(omega, target.max_omega);

                cmd.vx = static_cast<float>(velocity_body.x());
                cmd.vy = static_cast<float>(velocity_body.y());
                cmd.vz = static_cast<float>(omega);
            }
        } else {
            // Running 阶段先生成轨迹参考值，再叠加 P 控制反馈。
            Eigen::Vector2d reference_pos = target.target_pos;
            Eigen::Vector2d reference_vel = Eigen::Vector2d::Zero();
            double reference_yaw = target.constraint_target_yaw ? target.target_yaw : current_yaw_;
            double reference_omega = 0.0;
            double segment_duration = kMinDuration;
            double segment_progress = 1.0;

            if (transition_.active) {
                // 连接半径内启用三次 Hermite 曲线，起终点同时约束位置和速度。
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
                // 默认轨迹是一段直线，沿 segment_start_pos_ -> target_pos 做速度规划。
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
                    // 非横移模式先原地瞄准目标方向，满足阈值后本段不再回到瞄准。
                    const double yaw_error = normalize_angle(path_yaw - current_yaw_);
                    if (std::abs(yaw_error) < static_cast<double>(target.allow_start_dir_error)) {
                        begin_current_segment(time, 0.0);
                        // begin_current_segment 会按 allow_y_vel 初始化 aiming_done_，
                        // 这里必须重新置 true，否则 allow_y_vel=false 的轨迹会永远卡在瞄准阶段。
                        aiming_done_ = true;
                        segment_vec = target.target_pos - segment_start_pos_;
                        distance = segment_vec.norm();
                    } else {
                        double omega = target.kp.z() * yaw_error;
                        apply_minimum(omega, yaw_error, target.adjust_min_omega);
                        cmd.vx = 0.0f;
                        cmd.vy = 0.0f;
                        cmd.vz = static_cast<float>(clamp_abs(omega, target.max_omega));
                        return cmd;
                    }
                }

                const double elapsed = std::chrono::duration<double>(time - segment_start_time_).count();
                // 轨迹规划速度是前馈项，后面会再加 kp * error 形成最终速度。
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

                if (!target.allow_y_vel) {
                    // 非横移模式行走阶段只保持朝向路径方向，最终 target_yaw 留给 Adjusting 阶段原地对准。
                    reference_yaw = path_yaw;
                    reference_omega = 0.0;
                } else if (target.constraint_target_yaw) {
                    const double yaw_delta = normalize_angle(target.target_yaw - segment_start_yaw_);
                    reference_yaw = normalize_angle(segment_start_yaw_ + yaw_delta * segment_progress);
                    reference_omega = segment_duration > kMinDuration ? yaw_delta / segment_duration : 0.0;
                } else {
                    reference_yaw = current_yaw_;
                }

                const float connection_radius = target.trajectory_connection_radius;
                if (current_index_ + 1 < targets_.size() && connection_radius > 0.0f &&
                    (target.target_pos - current_pos_).norm() < static_cast<double>(connection_radius)) {
                    // 提前转入下一段方向，避免在连接点附近速度方向突然跳变。
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

            // 最终控制律：机体系速度 = 轨迹规划速度 + P 控制反馈。
            const Eigen::Vector2d pos_error_body = world_to_body(reference_pos - current_pos_, current_yaw_);
            Eigen::Vector2d velocity_body = world_to_body(reference_vel, current_yaw_);
            velocity_body.x() += target.kp.x() * pos_error_body.x();
            velocity_body.y() += target.kp.y() * pos_error_body.y();
            const Eigen::Vector2d target_error_body = world_to_body(target.target_pos - current_pos_, current_yaw_);
            clamp_translation(velocity_body, target.max_velocity);
            const bool translation_above_minimum = translation_is_above_minimum(velocity_body, target.adjust_min_vel);
            if (!translation_above_minimum) {
                apply_translation_minimum(velocity_body, target_error_body, target.adjust_min_vel);
                clamp_translation(velocity_body, target.max_velocity);
            }

            const double yaw_error = normalize_angle(reference_yaw - current_yaw_);
            double omega = reference_omega + target.kp.z() * yaw_error;
            if (!translation_above_minimum) {
                apply_minimum(omega, yaw_error, target.adjust_min_omega);
            }
            omega = clamp_abs(omega, target.max_omega);

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
