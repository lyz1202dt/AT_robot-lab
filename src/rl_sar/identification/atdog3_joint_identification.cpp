#include "leg_driver/leg_driver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kDofCount = 12;
constexpr double kPi = 3.14159265358979323846;
std::atomic_bool g_shutdown_requested{false};

void signal_handler(int) {
    g_shutdown_requested.store(true);
}

enum class Phase {
    HoldCurrent,
    RampToCenter,
    RunExperiment,
    RampBack,
    FinalHold
};

std::string phase_name(Phase phase) {
    switch (phase) {
        case Phase::HoldCurrent:
            return "HOLD_CURRENT";
        case Phase::RampToCenter:
            return "RAMP_TO_CENTER";
        case Phase::RunExperiment:
            return "RUN_EXPERIMENT";
        case Phase::RampBack:
            return "RAMP_BACK";
        case Phase::FinalHold:
            return "FINAL_HOLD";
    }
    return "UNKNOWN";
}

struct Config {
    double control_hz{};
    double wait_feedback_seconds{};
    double hold_current_seconds{};
    double ramp_to_center_seconds{};
    double ramp_back_seconds{};
    double final_hold_seconds{};
    double feedback_timeout_ms{};
    int max_consecutive_feedback_misses{};
    int flush_every_n_cycles{};
    double max_position_step_per_cycle{};
    double max_velocity_abs{};
    double max_torque_abs{};
    double measured_limit_margin_rad{};
    double max_initial_center_error_rad{};
    std::string csv_path;
    std::string experiment_type;
    double experiment_duration_seconds{};
    double sine_frequency_hz{};
    double sine_phase_rad{};
    double step_offset_rad{};
    double step_hold_before_seconds{};
    double step_transition_seconds{};
    double step_hold_after_seconds{};
    std::array<int, kDofCount> mapping{};
    std::array<bool, kDofCount> enabled{};
    std::array<double, kDofCount> center_rad{};
    std::array<double, kDofCount> lower_limit_rad{};
    std::array<double, kDofCount> upper_limit_rad{};
    std::array<double, kDofCount> amplitude_rad{};
    std::array<double, kDofCount> kp{};
    std::array<double, kDofCount> kd{};
};

template <typename T>
std::array<T, kDofCount> read_array(const YAML::Node& node, const std::string& key) {
    const auto values = node[key];
    if (!values || !values.IsSequence() || values.size() != kDofCount) {
        throw std::runtime_error(key + " 必须包含12个元素");
    }

    std::array<T, kDofCount> result{};
    for (std::size_t i = 0; i < kDofCount; ++i) {
        result[i] = values[i].as<T>();
    }
    return result;
}

Config load_config(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    Config config;

    const auto runtime = root["runtime"];
    const auto safety = root["safety"];
    const auto logging = root["logging"];
    const auto joints = root["joints"];
    const auto experiment = root["experiment"];

    if (!runtime || !safety || !logging || !joints || !experiment) {
        throw std::runtime_error("配置必须包含 runtime、safety、logging、joints 和 experiment");
    }

    config.control_hz = runtime["control_hz"].as<double>();
    config.wait_feedback_seconds = runtime["wait_feedback_seconds"].as<double>();
    config.hold_current_seconds = runtime["hold_current_seconds"].as<double>();
    config.ramp_to_center_seconds = runtime["ramp_to_center_seconds"].as<double>();
    config.ramp_back_seconds = runtime["ramp_back_seconds"].as<double>();
    config.final_hold_seconds = runtime["final_hold_seconds"].as<double>();
    config.feedback_timeout_ms = safety["feedback_timeout_ms"].as<double>();
    config.max_consecutive_feedback_misses = safety["max_consecutive_feedback_misses"].as<int>();
    config.max_position_step_per_cycle = safety["max_position_step_per_cycle"].as<double>();
    config.max_velocity_abs = safety["max_velocity_abs"].as<double>();
    config.max_torque_abs = safety["max_torque_abs"].as<double>();
    config.measured_limit_margin_rad = safety["measured_limit_margin_rad"].as<double>();
    config.max_initial_center_error_rad = safety["max_initial_center_error_rad"].as<double>();
    config.csv_path = logging["csv_path"].as<std::string>();
    config.flush_every_n_cycles = logging["flush_every_n_cycles"].as<int>();
    config.mapping = read_array<int>(joints, "mapping");
    config.enabled = read_array<bool>(joints, "enabled");
    config.center_rad = read_array<double>(joints, "center_rad");
    config.lower_limit_rad = read_array<double>(joints, "lower_limit_rad");
    config.upper_limit_rad = read_array<double>(joints, "upper_limit_rad");
    config.amplitude_rad = read_array<double>(joints, "amplitude_rad");
    config.kp = read_array<double>(joints, "kp");
    config.kd = read_array<double>(joints, "kd");
    config.experiment_type = experiment["type"].as<std::string>();
    config.experiment_duration_seconds = experiment["duration_seconds"].as<double>();
    config.sine_frequency_hz = experiment["sine"]["frequency_hz"].as<double>();
    config.sine_phase_rad = experiment["sine"]["phase_rad"].as<double>();
    config.step_offset_rad = experiment["smooth_step"]["offset_rad"].as<double>();
    config.step_hold_before_seconds = experiment["smooth_step"]["hold_before_seconds"].as<double>();
    config.step_transition_seconds = experiment["smooth_step"]["transition_seconds"].as<double>();
    config.step_hold_after_seconds = experiment["smooth_step"]["hold_after_seconds"].as<double>();
    return config;
}

void validate_config(const Config& config) {
    if (config.control_hz <= 0.0 || config.control_hz > 1000.0) {
        throw std::runtime_error("control_hz 必须在 (0, 1000] 范围内");
    }
    if (config.wait_feedback_seconds <= 0.0 || config.hold_current_seconds < 0.0 ||
        config.ramp_to_center_seconds <= 0.0 || config.ramp_back_seconds <= 0.0 ||
        config.final_hold_seconds < 0.0 || config.experiment_duration_seconds <= 0.0) {
        throw std::runtime_error("运行时长配置不合法");
    }
    if (config.feedback_timeout_ms <= 0.0 || config.max_consecutive_feedback_misses < 1 ||
        config.max_position_step_per_cycle <= 0.0 || config.max_velocity_abs <= 0.0 ||
        config.max_torque_abs <= 0.0 || config.measured_limit_margin_rad < 0.0 ||
        config.max_initial_center_error_rad <= 0.0 || config.flush_every_n_cycles < 1) {
        throw std::runtime_error("安全或日志参数不合法");
    }
    if (config.experiment_type != "hold" && config.experiment_type != "smooth_step" &&
        config.experiment_type != "sine") {
        throw std::runtime_error("experiment.type 仅支持 hold、smooth_step、sine");
    }
    if (config.sine_frequency_hz <= 0.0 || config.step_transition_seconds <= 0.0 ||
        config.step_hold_before_seconds < 0.0 || config.step_hold_after_seconds < 0.0) {
        throw std::runtime_error("正弦或平滑阶跃参数不合法");
    }

    std::set<int> mapped_indices;
    bool has_enabled_joint = false;
    for (std::size_t i = 0; i < kDofCount; ++i) {
        if (config.mapping[i] < 0 || config.mapping[i] >= static_cast<int>(kDofCount) ||
            !mapped_indices.insert(config.mapping[i]).second) {
            throw std::runtime_error("joints.mapping 必须是0到11的不重复映射");
        }
        if (config.lower_limit_rad[i] >= config.upper_limit_rad[i]) {
            throw std::runtime_error("关节" + std::to_string(i) + "的下限必须小于上限");
        }
        if (config.center_rad[i] < config.lower_limit_rad[i] ||
            config.center_rad[i] > config.upper_limit_rad[i]) {
            throw std::runtime_error("关节" + std::to_string(i) + "的中心角超出软限位");
        }
        if (config.kp[i] < 0.0 || config.kd[i] < 0.0) {
            throw std::runtime_error("Kp和Kd不能为负数");
        }
        if (!config.enabled[i]) {
            continue;
        }
        has_enabled_joint = true;
        if (config.center_rad[i] - std::abs(config.amplitude_rad[i]) < config.lower_limit_rad[i] ||
            config.center_rad[i] + std::abs(config.amplitude_rad[i]) > config.upper_limit_rad[i]) {
            throw std::runtime_error("关节" + std::to_string(i) + "的正弦轨迹超出软限位");
        }
        const double step_target = config.center_rad[i] + config.step_offset_rad;
        if (step_target < config.lower_limit_rad[i] || step_target > config.upper_limit_rad[i]) {
            throw std::runtime_error("关节" + std::to_string(i) + "的阶跃目标超出软限位");
        }
    }
    if (!has_enabled_joint) {
        throw std::runtime_error("至少启用一个关节");
    }
}

double smoothstep(double ratio) {
    const double value = std::clamp(ratio, 0.0, 1.0);
    return value * value * (3.0 - 2.0 * value);
}

uint64_t steady_time_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

uint32_t steady_time_ms32() {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

std::array<double, kDofCount> unpack_state(
    const std::array<LegState_t, 4>& legs, const std::array<int, kDofCount>& mapping, int field) {
    std::array<double, kDofCount> values{};
    for (std::size_t dof = 0; dof < kDofCount; ++dof) {
        const int hardware_index = mapping[dof];
        const auto& state = legs[hardware_index / 3].joint[hardware_index % 3];
        values[dof] = field == 0 ? state.rad : (field == 1 ? state.omega : state.torque);
    }
    return values;
}

std::array<LegTarget_t, 4> make_targets(
    const Config& config, const std::array<double, kDofCount>& command) {
    std::array<LegTarget_t, 4> targets{};
    for (std::size_t dof = 0; dof < kDofCount; ++dof) {
        const int hardware_index = config.mapping[dof];
        auto& target = targets[hardware_index / 3].joint[hardware_index % 3];
        target.rad = static_cast<float>(command[dof]);
        target.omega = 0.0f;
        target.torque = 0.0f;
        target.kp = static_cast<float>(config.kp[dof]);
        target.kd = static_cast<float>(config.kd[dof]);
    }
    return targets;
}

void send_safe_damping(LegDriver& driver) {
    std::array<LegTarget_t, 4> targets{};
    driver.enable_control(false);
    for (int i = 0; i < 5; ++i) {
        driver.set_leg_target(targets, steady_time_ms32());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

class SafetyGuard {
public:
    explicit SafetyGuard(LegDriver& driver) : driver_(driver) {}
    ~SafetyGuard() {
        if (armed_) {
            send_safe_damping(driver_);
        }
    }
    void disarm() { armed_ = false; }

private:
    LegDriver& driver_;
    bool armed_{true};
};

class CsvLogger {
public:
    CsvLogger(const std::string& path, int flush_every_n_cycles)
        : stream_(path), flush_every_n_cycles_(flush_every_n_cycles) {
        if (!stream_.is_open()) {
            throw std::runtime_error("无法打开CSV文件: " + path);
        }
        stream_ << "steady_time_ns,elapsed_s,cycle_index,phase,experiment_type,bottom_time_ms,dof,"
                   "hardware_index,leg_index,joint_index,enabled,q_cmd_rad,dq_cmd_rad_s,tau_cmd_nm,kp,kd,"
                   "q_meas_rad,dq_meas_rad_s,tau_meas_nm,q_error_rad,feedback_ok,set_target_ok\n";
        stream_ << std::setprecision(10);
    }

    void write_cycle(uint64_t time_ns, double elapsed_s, uint64_t cycle_index, Phase phase,
                     const Config& config, uint32_t bottom_time,
                     const std::array<double, kDofCount>& command,
                     const std::array<double, kDofCount>& measured_position,
                     const std::array<double, kDofCount>& measured_velocity,
                     const std::array<double, kDofCount>& measured_torque, bool feedback_ok,
                     bool set_target_ok) {
        for (std::size_t dof = 0; dof < kDofCount; ++dof) {
            const int hardware_index = config.mapping[dof];
            stream_ << time_ns << ',' << elapsed_s << ',' << cycle_index << ',' << phase_name(phase) << ','
                    << config.experiment_type << ',' << bottom_time << ',' << dof << ',' << hardware_index << ','
                    << hardware_index / 3 << ',' << hardware_index % 3 << ',' << config.enabled[dof] << ','
                    << command[dof] << ",0,0," << config.kp[dof] << ',' << config.kd[dof] << ','
                    << measured_position[dof] << ',' << measured_velocity[dof] << ',' << measured_torque[dof] << ','
                    << command[dof] - measured_position[dof] << ',' << feedback_ok << ',' << set_target_ok << '\n';
        }
        if (++cycles_since_flush_ >= flush_every_n_cycles_) {
            stream_.flush();
            cycles_since_flush_ = 0;
        }
    }

private:
    std::ofstream stream_;
    int flush_every_n_cycles_{};
    int cycles_since_flush_{};
};

std::array<double, kDofCount> experiment_command(const Config& config,
                                                  const std::array<double, kDofCount>& center,
                                                  double elapsed_s) {
    auto command = center;
    if (config.experiment_type == "hold") {
        return command;
    }

    for (std::size_t dof = 0; dof < kDofCount; ++dof) {
        if (!config.enabled[dof]) {
            continue;
        }
        if (config.experiment_type == "sine") {
            command[dof] = config.center_rad[dof] + config.amplitude_rad[dof] *
                                                        std::sin(2.0 * kPi * config.sine_frequency_hz * elapsed_s +
                                                                 config.sine_phase_rad);
        } else {
            const double transition_start = config.step_hold_before_seconds;
            const double transition_end = transition_start + config.step_transition_seconds;
            if (elapsed_s <= transition_start) {
                command[dof] = config.center_rad[dof];
            } else if (elapsed_s < transition_end) {
                command[dof] = config.center_rad[dof] +
                               config.step_offset_rad *
                                   smoothstep((elapsed_s - transition_start) / config.step_transition_seconds);
            } else {
                command[dof] = config.center_rad[dof] + config.step_offset_rad;
            }
        }
    }
    return command;
}

void apply_step_limit(std::array<double, kDofCount>& command,
                      const std::array<double, kDofCount>& previous, double max_step) {
    for (std::size_t i = 0; i < kDofCount; ++i) {
        command[i] = std::clamp(command[i], previous[i] - max_step, previous[i] + max_step);
    }
}

void check_runtime_safety(const Config& config,
                          const std::array<double, kDofCount>& command,
                          const std::array<double, kDofCount>& measured_position,
                          const std::array<double, kDofCount>& measured_velocity,
                          const std::array<double, kDofCount>& measured_torque) {
    for (std::size_t i = 0; i < kDofCount; ++i) {
        if (command[i] < config.lower_limit_rad[i] || command[i] > config.upper_limit_rad[i]) {
            throw std::runtime_error("关节" + std::to_string(i) + "的目标角超出软限位");
        }
        if (measured_position[i] < config.lower_limit_rad[i] - config.measured_limit_margin_rad ||
            measured_position[i] > config.upper_limit_rad[i] + config.measured_limit_margin_rad) {
            throw std::runtime_error("关节" + std::to_string(i) + "的实测角超出保护范围");
        }
        if (std::abs(measured_velocity[i]) > config.max_velocity_abs) {
            throw std::runtime_error("关节" + std::to_string(i) + "的实测速度超限");
        }
        if (std::abs(measured_torque[i]) > config.max_torque_abs) {
            throw std::runtime_error("关节" + std::to_string(i) + "的实测力矩超限");
        }
    }
}

int run(const Config& config) {
    LegDriver driver;
    SafetyGuard safety_guard(driver);
    driver.set_motor_error_callback([](uint16_t state) {
        std::cerr << "电机异常，motor_state=" << state << std::endl;
        g_shutdown_requested.store(true);
    });

    std::array<LegState_t, 4> legs_state{};
    uint32_t bottom_time = 0;
    const auto feedback_deadline = std::chrono::steady_clock::now() +
                                   std::chrono::duration<double>(config.wait_feedback_seconds);
    while (!g_shutdown_requested.load() && std::chrono::steady_clock::now() < feedback_deadline) {
        if (driver.get_leg_state(legs_state, bottom_time)) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!driver.get_leg_state(legs_state, bottom_time)) {
        throw std::runtime_error("等待关节反馈超时，未启用位置控制");
    }

    const auto initial_position = unpack_state(legs_state, config.mapping, 0);
    for (std::size_t i = 0; i < kDofCount; ++i) {
        if (initial_position[i] < config.lower_limit_rad[i] - config.measured_limit_margin_rad ||
            initial_position[i] > config.upper_limit_rad[i] + config.measured_limit_margin_rad) {
            throw std::runtime_error("初始关节" + std::to_string(i) + "超出保护范围");
        }
        if (config.enabled[i] &&
            std::abs(initial_position[i] - config.center_rad[i]) > config.max_initial_center_error_rad) {
            throw std::runtime_error("启用关节" + std::to_string(i) + "距离中心角过大");
        }
    }

    CsvLogger logger(config.csv_path, config.flush_every_n_cycles);
    auto command = initial_position;
    auto previous_command = command;
    auto measured_position = initial_position;
    auto measured_velocity = unpack_state(legs_state, config.mapping, 1);
    auto measured_torque = unpack_state(legs_state, config.mapping, 2);
    auto center_command = initial_position;
    auto ramp_back_start = initial_position;
    for (std::size_t i = 0; i < kDofCount; ++i) {
        if (config.enabled[i]) {
            center_command[i] = config.center_rad[i];
        }
    }

    driver.enable_control(true);

    const double period_seconds = 1.0 / config.control_hz;
    const auto period = std::chrono::duration<double>(period_seconds);
    auto next_cycle = std::chrono::steady_clock::now();
    auto phase_start = next_cycle;
    auto last_feedback = next_cycle;
    Phase phase = Phase::HoldCurrent;
    uint64_t cycle_index = 0;
    int feedback_misses = 0;

    while (!g_shutdown_requested.load()) {
        const auto now = std::chrono::steady_clock::now();
        const double phase_elapsed = std::chrono::duration<double>(now - phase_start).count();

        const bool feedback_ok = driver.get_leg_state(legs_state, bottom_time);
        if (feedback_ok) {
            measured_position = unpack_state(legs_state, config.mapping, 0);
            measured_velocity = unpack_state(legs_state, config.mapping, 1);
            measured_torque = unpack_state(legs_state, config.mapping, 2);
            last_feedback = now;
            feedback_misses = 0;
        } else {
            ++feedback_misses;
        }
        const double feedback_age_ms = std::chrono::duration<double, std::milli>(now - last_feedback).count();
        if (feedback_misses >= config.max_consecutive_feedback_misses ||
            feedback_age_ms > config.feedback_timeout_ms) {
            throw std::runtime_error("关节反馈丢失或超时");
        }

        switch (phase) {
            case Phase::HoldCurrent:
                command = initial_position;
                if (phase_elapsed >= config.hold_current_seconds) {
                    phase = Phase::RampToCenter;
                    phase_start = now;
                }
                break;
            case Phase::RampToCenter: {
                const double alpha = smoothstep(phase_elapsed / config.ramp_to_center_seconds);
                for (std::size_t i = 0; i < kDofCount; ++i) {
                    command[i] = initial_position[i] + alpha * (center_command[i] - initial_position[i]);
                }
                if (phase_elapsed >= config.ramp_to_center_seconds) {
                    command = center_command;
                    phase = Phase::RunExperiment;
                    phase_start = now;
                }
                break;
            }
            case Phase::RunExperiment:
                command = experiment_command(config, center_command, phase_elapsed);
                if (phase_elapsed >= config.experiment_duration_seconds) {
                    ramp_back_start = command;
                    phase = Phase::RampBack;
                    phase_start = now;
                }
                break;
            case Phase::RampBack: {
                const double alpha = smoothstep(phase_elapsed / config.ramp_back_seconds);
                for (std::size_t i = 0; i < kDofCount; ++i) {
                    command[i] = ramp_back_start[i] + alpha * (center_command[i] - ramp_back_start[i]);
                }
                if (phase_elapsed >= config.ramp_back_seconds) {
                    command = center_command;
                    phase = Phase::FinalHold;
                    phase_start = now;
                }
                break;
            }
            case Phase::FinalHold:
                command = center_command;
                if (phase_elapsed >= config.final_hold_seconds) {
                    send_safe_damping(driver);
                    safety_guard.disarm();
                    return 0;
                }
                break;
        }

        apply_step_limit(command, previous_command, config.max_position_step_per_cycle);
        check_runtime_safety(config, command, measured_position, measured_velocity, measured_torque);
        const auto targets = make_targets(config, command);
        const bool set_target_ok = driver.set_leg_target(targets, steady_time_ms32());
        if (!set_target_ok) {
            throw std::runtime_error("发送关节目标失败");
        }

        const double total_elapsed = cycle_index * period_seconds;
        logger.write_cycle(steady_time_ns(), total_elapsed, cycle_index, phase, config, bottom_time,
                           command, measured_position, measured_velocity, measured_torque, feedback_ok,
                           set_target_ok);
        previous_command = command;
        ++cycle_index;
        next_cycle += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_cycle);
    }

    throw std::runtime_error("收到退出信号");
}

void print_usage(const char* program) {
    std::cout << "用法:\n  " << program << " <配置文件.yaml>\n  " << program
              << " --validate <配置文件.yaml>\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    try {
        bool validate_only = false;
        std::string config_path;
        if (argc == 2) {
            config_path = argv[1];
        } else if (argc == 3 && std::string(argv[1]) == "--validate") {
            validate_only = true;
            config_path = argv[2];
        } else {
            print_usage(argv[0]);
            return 1;
        }

        const Config config = load_config(config_path);
        validate_config(config);
        std::cout << "配置校验通过：实验类型=" << config.experiment_type
                  << "，控制频率=" << config.control_hz << " Hz，CSV=" << config.csv_path << std::endl;
        if (validate_only) {
            return 0;
        }

        std::cout << "警告：即将连接实机并发送位置命令。请确保机器狗已可靠悬空且急停可用。" << std::endl;
        return run(config);
    } catch (const std::exception& error) {
        std::cerr << "参数辨识程序退出：" << error.what() << std::endl;
        return 1;
    }
}
