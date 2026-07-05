#include <core/pilot.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <exception>
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

double read_required_double(const YAML::Node& node, const std::string& key)
{
    return node[key].as<double>();
}

double read_optional_double(const YAML::Node& node, const std::string& key, double default_value)
{
    return node[key] ? node[key].as<double>() : default_value;
}

bool read_optional_bool(const YAML::Node& node, const std::string& key, bool default_value)
{
    return node[key] ? node[key].as<bool>() : default_value;
}

MotionSample sample_trapezoid_profile(
    double distance,
    double elapsed,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_acceleration)
{
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

void clamp_translation(Eigen::Vector2d& velocity, double max_velocity)
{
    const double limit = std::max(max_velocity, 0.0);
    const double norm = velocity.norm();
    if (limit > 0.0 && norm > limit) {
        velocity *= limit / norm;
    }
}

void apply_minimum(double& command, double error, double minimum)
{
    if (std::abs(error) <= kTinyError || minimum <= 0.0) {
        return;
    }
    if (std::abs(command) < minimum) {
        command = std::copysign(minimum, command == 0.0 ? error : command);
    }
}

void apply_translation_minimum(Eigen::Vector2d& velocity, const Eigen::Vector2d& error, double minimum)
{
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

bool translation_is_above_minimum(const Eigen::Vector2d& velocity, double minimum)
{
    return minimum > 0.0 && velocity.norm() > minimum;
}

}  // namespace

Pilot::Pilot(rclcpp::Node::SharedPtr node, const std::string yaml_path)
    : node_(std::move(node))
{
    if (!yaml_path.empty()) {
        load_paths(yaml_path);
    } else {
        RCLCPP_WARN(this->node_->get_logger(), "Pilot未提供yaml路径，轨迹为空");
    }

    reset();
}

Pilot::~Pilot() = default;

bool Pilot::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (paths_.empty()) {
        RCLCPP_WARN(node_->get_logger(), "轨迹为空，无法开始执行");
        return false;
    }

    const auto now = std::chrono::high_resolution_clock::now();
    if (state_ == PilotState::Paused) {
        state_ = resume_state_ == PilotState::Finished ? PilotState::Running : resume_state_;
        if (state_ == PilotState::Running) {
            begin_current_segment(now, 0.0);
        } else if (state_ == PilotState::ExternalAction) {
            transition_ = CubicTransition{};
        } else if (state_ == PilotState::Standing) {
            stand_start_time_ = now;
        }
        return true;
    }

    if (state_ == PilotState::Finished || current_path_index_ >= paths_.size()) {
        current_path_index_ = 0;
    }

    if (is_external_action_policy(paths_[current_path_index_])) {
        state_ = PilotState::ExternalAction;
        transition_ = CubicTransition{};
        policy_done_pending_ = false;
        done_policy_id_ = 0;
    } else {
        state_ = PilotState::Running;
        begin_current_segment(now, 0.0);
    }
    return true;
}

bool Pilot::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == PilotState::Running || state_ == PilotState::Standing || state_ == PilotState::Adjusting ||
        state_ == PilotState::ExternalAction) {
        resume_state_ = state_;
        state_ = PilotState::Paused;
    } else if (state_ != PilotState::Finished) {
        state_ = PilotState::Idle;
    }
    transition_ = CubicTransition{};
    return true;
}

bool Pilot::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    reset_execution();
    return !paths_.empty();
}

void Pilot::set_state(const Eigen::Vector2d& pos, const double& yaw)
{
    std::lock_guard<std::mutex> lock(mutex_);
    current_pos_ = pos;
    current_yaw_ = normalize_angle(yaw);
    has_state_ = true;
}

bool Pilot::get_current_path_info(uint32_t& path_num, float& time_rate)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (paths_.empty()) {
        return false;
    }

    if (state_ == PilotState::Finished || current_path_index_ >= paths_.size()) {
        path_num = static_cast<uint32_t>(paths_.size() - 1);
        time_rate = 1.0f;
        return true;
    }

    path_num = static_cast<uint32_t>(current_path_index_);
    const double total_distance = (paths_[current_path_index_].target_pos - segment_start_pos_).norm();
    if (!has_state_ || total_distance <= kMinDistance) {
        time_rate = 0.0f;
        return true;
    }

    const double remain_distance = (paths_[current_path_index_].target_pos - current_pos_).norm();
    time_rate = static_cast<float>(std::clamp(1.0 - remain_distance / total_distance, 0.0, 1.0));
    return true;
}

void Pilot::notify_policy_done(int32_t policy_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != PilotState::ExternalAction || current_path_index_ >= paths_.size()) {
        return;
    }

    const auto& path = paths_[current_path_index_];
    if (path.policy_id != policy_id || !is_external_action_policy(path)) {
        return;
    }

    policy_done_pending_ = true;
    done_policy_id_ = policy_id;
}

robot_msgs::msg::Cmd Pilot::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    robot_msgs::msg::Cmd cmd = walk_zero_command();

    std::lock_guard<std::mutex> lock(mutex_);
    if (paths_.empty() || !has_state_ || state_ == PilotState::Idle || state_ == PilotState::Paused) {
        return stand_command();
    }
    if (state_ == PilotState::Finished) {
        return stand_command();
    }
    if (current_path_index_ >= paths_.size()) {
        return stand_command();
    }

    PathPoint& path = paths_[current_path_index_];
    cmd.mode = policy_id_to_cmd_mode(path.policy_id);

    if (is_external_action_policy(path) && state_ != PilotState::ExternalAction) {
        state_ = PilotState::ExternalAction;
        transition_ = CubicTransition{};
    }

    if (state_ == PilotState::ExternalAction) {
        if (policy_done_pending_ && done_policy_id_ == path.policy_id) {
            policy_done_pending_ = false;
            done_policy_id_ = 0;
            RCLCPP_INFO(
                node_->get_logger(),
                "外部固定动作完成，切换下一段: target=(%.3f, %.3f), policy_id=%d",
                path.target_pos.x(),
                path.target_pos.y(),
                path.policy_id);
            advance_after_stand(time);

            if (state_ == PilotState::Finished || current_path_index_ >= paths_.size()) {
                return stand_command();
            }

            PathPoint& next_path = paths_[current_path_index_];
            cmd.mode = policy_id_to_cmd_mode(next_path.policy_id);
        }
        return cmd;
    }

    if (state_ == PilotState::Standing) {
        const double elapsed = std::chrono::duration<double>(time - stand_start_time_).count();
        if (elapsed < std::max(0.0, path.stand_duration)) {
            return stand_command();
        }
        advance_after_stand(time);
        if (current_path_index_ >= paths_.size() || state_ != PilotState::Adjusting) {
            return stand_command();
        }
    }

    if (state_ == PilotState::Adjusting) {
        const Eigen::Vector2d pos_error_world = path.target_pos - current_pos_;
        const Eigen::Vector2d pos_error_body = world_to_body(pos_error_world, current_yaw_);
        const double yaw_error = path.constraint_target_yaw ? normalize_angle(path.target_yaw - current_yaw_) : 0.0;
        const bool pos_ok = pos_error_world.norm() <= path.allow_final_pos_allow;
        const bool yaw_ok = !path.constraint_target_yaw || std::abs(yaw_error) <= path.allow_final_dir_error;

        const auto finish_path = [&]() {
            state_ = PilotState::Finished;
            current_path_index_ = paths_.size();
            cmd = stand_command();
        };

        const auto set_position_adjust_command = [&]() {
            Eigen::Vector2d velocity_body = Eigen::Vector2d::Zero();
            velocity_body.x() = path.kp.x() * pos_error_body.x();
            velocity_body.y() = path.kp.y() * pos_error_body.y();
            clamp_translation(velocity_body, path.max_velocity);
            const bool translation_above_minimum = translation_is_above_minimum(velocity_body, path.adjust_min_vel);
            if (!translation_above_minimum) {
                apply_translation_minimum(velocity_body, pos_error_body, path.adjust_min_vel);
                clamp_translation(velocity_body, path.max_velocity);
            }

            cmd.vx = static_cast<float>(velocity_body.x());
            cmd.vy = static_cast<float>(velocity_body.y());
            cmd.vz = 0.0f;
        };

        const auto set_yaw_adjust_command = [&]() {
            double omega = 0.0;
            if (path.constraint_target_yaw && !yaw_ok) {
                omega = path.kp.z() * yaw_error;
                apply_minimum(omega, yaw_error, path.adjust_min_omega);
            }
            omega = clamp_abs(omega, path.max_omega);
            cmd.vx = 0.0f;
            cmd.vy = 0.0f;
            cmd.vz = static_cast<float>(omega);
        };

        for (int step = 0; step < 3; ++step) {
            if (adjust_phase_ == AdjustPhase::Position) {
                if (!pos_ok) {
                    set_position_adjust_command();
                    break;
                }
                adjust_phase_ = path.constraint_target_yaw ? AdjustPhase::Yaw : AdjustPhase::FinalPosition;
                continue;
            }

            if (adjust_phase_ == AdjustPhase::Yaw) {
                if (!yaw_ok) {
                    set_yaw_adjust_command();
                    break;
                }
                adjust_phase_ = AdjustPhase::FinalPosition;
                continue;
            }

            if (!pos_ok) {
                set_position_adjust_command();
                break;
            }
            finish_path();
            break;
        }
        return cmd;
    }

    Eigen::Vector2d reference_pos = path.target_pos;
    Eigen::Vector2d reference_vel = Eigen::Vector2d::Zero();
    double reference_yaw = path.constraint_target_yaw ? path.target_yaw : current_yaw_;
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

        reference_pos = h00 * transition_.start_pos + h10 * duration * transition_.start_vel + h01 * transition_.end_pos +
                        h11 * duration * transition_.end_vel;
        reference_vel = (dh00 * transition_.start_pos + dh10 * duration * transition_.start_vel + dh01 * transition_.end_pos +
                         dh11 * duration * transition_.end_vel) /
                        duration;

        const double yaw_delta = normalize_angle(transition_.end_yaw - transition_.start_yaw);
        reference_yaw = normalize_angle(transition_.start_yaw + yaw_delta * u);
        reference_omega = yaw_delta / duration;

        if (elapsed >= duration) {
            ++current_path_index_;
            segment_start_pos_ = transition_.end_pos;
            segment_start_yaw_ = transition_.end_yaw;
            segment_start_speed_ = transition_.end_vel.norm();
            segment_start_time_ = time;
            transition_ = CubicTransition{};
            aiming_done_ = true;
            if (current_path_index_ >= paths_.size()) {
                state_ = PilotState::Adjusting;
                adjust_phase_ = AdjustPhase::Position;
            }
        }
    } else {
        Eigen::Vector2d segment_vec = path.target_pos - segment_start_pos_;
        double distance = segment_vec.norm();

        if (distance <= kMinDistance) {
            finish_current_target(time);
            return cmd;
        }

        Eigen::Vector2d direction = segment_vec / distance;
        const double path_yaw = std::atan2(direction.y(), direction.x());

        if (!path.allow_y_vel && !aiming_done_) {
            const double yaw_error = normalize_angle(path_yaw - current_yaw_);
            if (std::abs(yaw_error) < path.allow_start_dir_error) {
                begin_current_segment(time, 0.0);
                aiming_done_ = true;
                segment_vec = path.target_pos - segment_start_pos_;
                distance = segment_vec.norm();
                if (distance <= kMinDistance) {
                    finish_current_target(time);
                    return cmd;
                }
                direction = segment_vec / distance;
            } else {
                double omega = path.kp.z() * yaw_error;
                apply_minimum(omega, yaw_error, path.adjust_min_omega);
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.vz = static_cast<float>(clamp_abs(omega, path.max_omega));
                return cmd;
            }
        }

        const double elapsed = std::chrono::duration<double>(time - segment_start_time_).count();
        const MotionSample profile = sample_trapezoid_profile(
            distance,
            elapsed,
            segment_start_speed_,
            std::max(0.0, path.target_vel),
            path.max_velocity > 0.0 ? path.max_velocity : kDefaultVelocity,
            path.max_accelation > 0.0 ? path.max_accelation : kDefaultAccel);

        segment_duration = profile.duration;
        segment_progress = distance > kMinDistance ? std::clamp(profile.position / distance, 0.0, 1.0) : 1.0;
        reference_pos = segment_start_pos_ + direction * profile.position;
        reference_vel = direction * profile.velocity;

        if (!path.allow_y_vel) {
            reference_yaw = path_yaw;
            reference_omega = 0.0;
        } else if (path.constraint_target_yaw) {
            const double yaw_delta = normalize_angle(path.target_yaw - segment_start_yaw_);
            reference_yaw = normalize_angle(segment_start_yaw_ + yaw_delta * segment_progress);
            reference_omega = segment_duration > kMinDuration ? yaw_delta / segment_duration : 0.0;
        } else {
            reference_yaw = current_yaw_;
        }

        const double connection_radius = path.trajectory_connection_radius;
        if (current_path_index_ + 1 < paths_.size() && connection_radius > 0.0 &&
            (path.target_pos - current_pos_).norm() < connection_radius) {
            const PathPoint& next_path = paths_[current_path_index_ + 1];
            const Eigen::Vector2d next_vec = next_path.target_pos - path.target_pos;
            const double next_distance = next_vec.norm();
            if (next_distance > kMinDistance) {
                const Eigen::Vector2d next_direction = next_vec / next_distance;
                const double blend_distance = std::min(connection_radius, next_distance * 0.5);
                const double blend_speed = std::max({profile.velocity, next_path.target_vel, 0.2});

                transition_.active = true;
                transition_.start_time = time;
                transition_.start_pos = reference_pos;
                transition_.start_vel = reference_vel;
                transition_.end_pos = path.target_pos + next_direction * blend_distance;
                transition_.end_vel = next_direction * std::min(
                    blend_speed,
                    next_path.max_velocity > 0.0 ? next_path.max_velocity : kDefaultVelocity);
                transition_.duration = std::max(
                    kMinDuration,
                    (transition_.end_pos - transition_.start_pos).norm() / std::max(transition_.start_vel.norm(), 0.2));
                transition_.start_yaw = reference_yaw;
                transition_.end_yaw = next_path.constraint_target_yaw ? next_path.target_yaw : std::atan2(next_direction.y(), next_direction.x());
            }
        }

        const bool reached_target = (path.target_pos - current_pos_).norm() <= path.allow_final_pos_allow;
        if (reached_target && !transition_.active) {
            finish_current_target(time);
            return cmd;
        }
    }

    if (current_path_index_ >= paths_.size()) {
        return cmd;
    }

    PathPoint& active_path = paths_[current_path_index_];
    cmd.mode = policy_id_to_cmd_mode(active_path.policy_id);
    const Eigen::Vector2d pos_error_body = world_to_body(reference_pos - current_pos_, current_yaw_);
    Eigen::Vector2d velocity_body = world_to_body(reference_vel, current_yaw_);
    velocity_body.x() += active_path.kp.x() * pos_error_body.x();
    velocity_body.y() += active_path.kp.y() * pos_error_body.y();
    const Eigen::Vector2d target_error_body = world_to_body(active_path.target_pos - current_pos_, current_yaw_);
    clamp_translation(velocity_body, active_path.max_velocity);
    const bool translation_above_minimum = translation_is_above_minimum(velocity_body, active_path.adjust_min_vel);
    if (!translation_above_minimum) {
        apply_translation_minimum(velocity_body, target_error_body, active_path.adjust_min_vel);
        clamp_translation(velocity_body, active_path.max_velocity);
    }

    const double yaw_error = normalize_angle(reference_yaw - current_yaw_);
    double omega = reference_omega + active_path.kp.z() * yaw_error;
    if (!translation_above_minimum) {
        apply_minimum(omega, yaw_error, active_path.adjust_min_omega);
    }
    omega = clamp_abs(omega, active_path.max_omega);

    cmd.vx = static_cast<float>(velocity_body.x());
    cmd.vy = static_cast<float>(velocity_body.y());
    cmd.vz = static_cast<float>(omega);
    return cmd;
}

bool Pilot::load_paths(const std::string& yaml_path)
{
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node paths_node = root["paths"];
        if (!paths_node || !paths_node.IsSequence()) {
            RCLCPP_ERROR(node_->get_logger(), "yaml文件%s缺少paths数组", yaml_path.c_str());
            return false;
        }

        std::vector<PathPoint> loaded_paths;
        loaded_paths.reserve(paths_node.size());
        for (const auto& path_node : paths_node) {
            PathPoint point;
            point.policy_id = path_node["policy_id"].as<int32_t>();
            point.target_pos.x() = read_required_double(path_node["target_pos"], "x");
            point.target_pos.y() = read_required_double(path_node["target_pos"], "y");
            point.target_vel = read_optional_double(path_node, "target_vel", point.target_vel);
            point.max_velocity = read_optional_double(path_node, "max_velocity", point.max_velocity);
            point.max_accelation = read_optional_double(path_node, "max_accelation", point.max_accelation);
            point.max_omega = read_optional_double(path_node, "max_omega", point.max_omega);
            if (path_node["kp"]) {
                point.kp.x() = read_optional_double(path_node["kp"], "x", point.kp.x());
                point.kp.y() = read_optional_double(path_node["kp"], "y", point.kp.y());
                point.kp.z() = read_optional_double(path_node["kp"], "yaw", point.kp.z());
            }
            point.allow_start_dir_error = read_optional_double(path_node, "allow_start_dir_error", point.allow_start_dir_error);
            point.allow_final_dir_error = read_optional_double(path_node, "allow_final_dir_error", point.allow_start_dir_error);
            point.allow_final_pos_allow = read_optional_double(path_node, "allow_final_pos_allow", read_optional_double(path_node, "err_allow", point.allow_final_pos_allow));
            point.adjust_min_vel = read_optional_double(path_node, "adjust_min_vel", point.adjust_min_vel);
            point.adjust_min_omega = read_optional_double(path_node, "adjust_min_omega", read_optional_double(path_node, "min_omega", point.adjust_min_omega));
            point.constraint_target_yaw = read_optional_bool(path_node, "constraint_target_yaw", point.constraint_target_yaw);
            point.target_yaw = read_optional_double(path_node, "target_yaw", point.target_yaw);
            point.allow_y_vel = read_optional_bool(path_node, "allow_y_vel", point.allow_y_vel);
            point.trajectory_connection_radius = read_optional_double(path_node, "trajectory_connection_radius", point.trajectory_connection_radius);
            point.stand_at_target = read_optional_bool(path_node, "stand_at_target", point.stand_at_target);
            point.stand_duration = read_optional_double(path_node, "stand_duration", point.stand_duration);
            loaded_paths.push_back(point);
        }

        std::lock_guard<std::mutex> lock(mutex_);
        paths_ = std::move(loaded_paths);
        reset_execution();
        RCLCPP_INFO(node_->get_logger(), "成功加载%zu个导航点: %s", paths_.size(), yaml_path.c_str());
        return !paths_.empty();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "加载导航点失败: %s, error: %s", yaml_path.c_str(), e.what());
        std::lock_guard<std::mutex> lock(mutex_);
        paths_.clear();
        reset_execution();
        return false;
    }
}

void Pilot::reset_execution()
{
    current_path_index_ = 0;
    state_ = paths_.empty() ? PilotState::Finished : PilotState::Idle;
    adjust_phase_ = AdjustPhase::Position;
    resume_state_ = PilotState::Running;
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = 0.0;
    aiming_done_ = false;
    segment_start_time_ = {};
    stand_start_time_ = {};
    policy_done_pending_ = false;
    done_policy_id_ = 0;
    transition_ = CubicTransition{};
}

void Pilot::begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed)
{
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = start_speed;
    segment_start_time_ = time;
    transition_ = CubicTransition{};

    if (current_path_index_ < paths_.size()) {
        aiming_done_ = paths_[current_path_index_].allow_y_vel;
    } else {
        aiming_done_ = true;
    }

    if (current_path_index_ < paths_.size() && is_external_action_policy(paths_[current_path_index_])) {
        policy_done_pending_ = false;
        done_policy_id_ = 0;
    }
}

void Pilot::finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    if (current_path_index_ >= paths_.size()) {
        state_ = PilotState::Finished;
        return;
    }

    const auto& current_path = paths_[current_path_index_];
    RCLCPP_INFO(
        node_->get_logger(),
        "目标点%zu/%zu执行完成: target=(%.3f, %.3f), policy_id=%d",
        current_path_index_ + 1,
        paths_.size(),
        current_path.target_pos.x(),
        current_path.target_pos.y(),
        current_path.policy_id);

    if (should_stand_at_current_target()) {
        state_ = PilotState::Standing;
        stand_start_time_ = time;
        transition_ = CubicTransition{};
        return;
    }

    advance_after_stand(time);
}

void Pilot::advance_after_stand(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    if (current_path_index_ + 1 < paths_.size()) {
        const auto previous_path = paths_[current_path_index_];
        ++current_path_index_;
        stand_start_time_ = {};
        transition_ = CubicTransition{};
        policy_done_pending_ = false;
        done_policy_id_ = 0;
        if (is_external_action_policy(paths_[current_path_index_])) {
            segment_start_pos_ = current_pos_;
            segment_start_yaw_ = current_yaw_;
            segment_start_speed_ = 0.0;
            segment_start_time_ = time;
            aiming_done_ = true;
            state_ = PilotState::ExternalAction;
        } else {
            if (is_external_action_policy(previous_path)) {
                segment_start_pos_ = current_pos_;
            } else {
                segment_start_pos_ = previous_path.target_pos;
            }
            segment_start_yaw_ = current_yaw_;
            segment_start_speed_ = std::max(0.0, previous_path.target_vel);
            segment_start_time_ = time;
            aiming_done_ = paths_[current_path_index_].allow_y_vel;
            state_ = PilotState::Running;
        }
        return;
    }

    const auto& previous_path = paths_[current_path_index_];
    stand_start_time_ = {};
    if (is_external_action_policy(previous_path)) {
        current_path_index_ = paths_.size();
        state_ = PilotState::Finished;
        transition_ = CubicTransition{};
        policy_done_pending_ = false;
        done_policy_id_ = 0;
        return;
    }

    state_ = PilotState::Adjusting;
    adjust_phase_ = AdjustPhase::Position;
    transition_ = CubicTransition{};
}

bool Pilot::should_stand_at_current_target() const
{
    if (current_path_index_ >= paths_.size()) {
        return false;
    }

    const auto& path = paths_[current_path_index_];
    return path.stand_at_target && path.stand_duration > 0.0 && path.trajectory_connection_radius <= 0.0;
}

robot_msgs::msg::Cmd Pilot::stand_command() const
{
    robot_msgs::msg::Cmd cmd;
    cmd.mode = kStandMode;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

robot_msgs::msg::Cmd Pilot::walk_zero_command() const
{
    robot_msgs::msg::Cmd cmd;
    cmd.mode = kWalkMode;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

bool Pilot::is_external_action_policy(const PathPoint& path)
{
    return path.policy_id == 8;
}

int32_t Pilot::policy_id_to_cmd_mode(int32_t policy_id)
{
    return policy_id;
}

double Pilot::normalize_angle(double angle)
{
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
    const double abs_limit = std::abs(limit);
    if (abs_limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -abs_limit, abs_limit);
}

Eigen::Vector2d Pilot::world_to_body(const Eigen::Vector2d& vector, double yaw)
{
    Eigen::Rotation2Dd rotation(-yaw);
    return rotation * vector;
}
