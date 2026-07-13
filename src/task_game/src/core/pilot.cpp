#include <core/pilot.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

/**
 * @file pilot.cpp
 * @brief Pilot 控制器的实现。核心由三部分组成：
 *        ①匿名命名空间中的轨迹规划与辅助函数；
 *        ②Pilot 的公共生命周期接口（start/stop/set_target/set_state）；
 *        ③每帧核心入口 get_command() 的状态机+控制律。
 *
 * 控制架构简述：
 *   world 误差 ──→ 旋转变换(body) ──→ P 控制器 ──→ 限幅/死区 ──→ Cmd(vx,vy,vz)
 *                       ↑                              ↑
 *                  前馈速度(规划)              前馈角速度(规划)
 *
 * Trapezoid Profile (梯形速度曲线):
 *   速度
 *    ↑    _______ vpeak ________
 *    |   /                      \
 *    |  / 加速段  匀速段  减速段  \
 *    | /                          \
 *    +──────────────────────────────→ 时间
 *   距离短时退化：若 √(a*D + ½(v0²+v1²)) < vmax，退化为三角曲线（无匀速段）
 *
 * Cubic Hermite Transition (三次 Hermite 过渡):
 *   用于多段轨迹衔接，保证位置和速度 C¹ 连续：
 *   P(u) = (2u³-3u²+1)P0 + (u³-2u²+u)*T*V0 + (-2u³+3u²)P1 + (u³-u²)*T*V1
 *   u ∈ [0,1], T = transition duration
 */
namespace {

// Cmd.mode 的约定来自现有控制链：
//   1 = 位控站立（机器人锁定位置）
//   2 = 普通行走（接受 vx/vy/vz 指令）
constexpr int kStandMode = 1;
constexpr int kWalkMode = 2;

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinDistance = 1e-4;     ///< 最小有效距离(m)，偏差小于此值视为已到位
constexpr double kMinDuration = 0.05;     ///< 最小规划时长(s)，防止零除
constexpr double kDefaultAccel = 0.25;    ///< 默认加速度(m/s²)
constexpr double kDefaultVelocity = 0.7;  ///< 默认最大速度(m/s)
constexpr double kTinyError = 1e-5;       ///< 极微小误差阈值，用于判断是否无需补偿
constexpr double kPiTurnDeadband = 0.08;  ///< 180° 临界区宽度(rad)，用于稳定初始瞄准方向

/**
 * @brief 速度规划采样结果
 * @param position  在 elapsed 时刻的参考位置 (m)，沿运动方向从起点算起的累计距离
 * @param velocity  在 elapsed 时刻的参考速度 (m/s)
 * @param duration  整段轨迹规划的总时长 (s)
 */
struct MotionSample {
    double position{0.0};
    double velocity{0.0};
    double duration{0.0};
};

/**
 * @brief 梯形/三角速度曲线采样
 *
 * 给定总路程 distance 和起止速度 (start_velocity, end_velocity)，
 * 在最大速度和最大加速度约束下规划一条速度剖面，
 * 返回 elapsed 时刻对应的参考位置、参考速度和总规划时长。
 *
 * 速度剖面分三段：加速段 → 匀速段(可选) → 减速段
 *
 * @param distance        总路程 (m)，必须 > kMinDistance
 * @param elapsed         从这段轨迹起点开始计算的时间 (s)
 * @param start_velocity  起点速度 (m/s)，会被 clamp 到 [0, vmax]
 * @param end_velocity    终点目标速度 (m/s)，会被 clamp 到 [0, vmax]
 * @param max_velocity    允许的最大速度 (m/s)
 * @param max_acceleration 最大加速度 (m/s²)
 * @return MotionSample  位置、速度、总时长
 *
 * 退化逻辑：
 *   当 √(accel * distance + ½(v0² + v1²)) < max_velocity 时，说明距离太短来不及加速到
 *   最大速度，此时自动退化为三角形速度曲线（加速→马上减速），无匀速平台段。
 */
MotionSample sample_trapezoid_profile(
    double distance,
    double elapsed,
    double start_velocity,
    double end_velocity,
    double max_velocity,
    double max_acceleration) {
    MotionSample sample;
    // 距离太小，直接返回零值
    if (distance <= kMinDistance) {
        return sample;
    }

    // 参数有效性保护：速度和加速度不能低于 0.05
    const double vmax = std::max(max_velocity, 0.05);
    const double accel = std::max(max_acceleration, 0.05);
    const double v0 = std::clamp(start_velocity, 0.0, vmax);
    const double v1 = std::clamp(end_velocity, 0.0, vmax);

    // 计算三角速度曲线的峰值（即刚好无匀速段的临界速度）
    // 公式来自运动学: v_peak² = a*D + ½(v0² + v1²)
    const double peak_for_triangle = std::sqrt(std::max(0.0, accel * distance + 0.5 * (v0 * v0 + v1 * v1)));
    // 实际峰值 = min(用户上限, 三角曲线需要的峰值)，且不低于起止速度
    const double vpeak = std::min(vmax, std::max({peak_for_triangle, v0, v1}));

    // 计算各段的时间
    const double accel_time = std::max(0.0, (vpeak - v0) / accel);  // 加速时间
    const double decel_time = std::max(0.0, (vpeak - v1) / accel);  // 减速时间
    // 计算各段的距离
    const double accel_dist = (vpeak * vpeak - v0 * v0) / (2.0 * accel);  // 加速段距离
    const double decel_dist = (vpeak * vpeak - v1 * v1) / (2.0 * accel);  // 减速段距离
    // 剩余距离为匀速段
    const double cruise_dist = std::max(0.0, distance - accel_dist - decel_dist);
    const double cruise_time = vpeak > kMinDistance ? cruise_dist / vpeak : 0.0;

    // 总规划时长
    sample.duration = std::max(kMinDuration, accel_time + cruise_time + decel_time);

    // 将 elapsed 限制在有效范围内求采样
    const double t = std::clamp(elapsed, 0.0, sample.duration);

    // ←── 加速段：v(t) = v0 + a*t,  s(t) = v0*t + ½*a*t²
    if (t < accel_time) {
        sample.velocity = v0 + accel * t;
        sample.position = v0 * t + 0.5 * accel * t * t;
        return sample;
    }

    // ←── 匀速段：v(t) = vpeak,  s(t) = s_accel + vpeak * (t - t_accel)
    if (t < accel_time + cruise_time) {
        const double cruise_elapsed = t - accel_time;
        sample.velocity = vpeak;
        sample.position = accel_dist + vpeak * cruise_elapsed;
        return sample;
    }

    // ←── 减速段：v(t) = vpeak - a*t_dec,  s(t) = s_accel + s_cruise + vpeak*t_dec - ½*a*t_dec²
    const double decel_elapsed = t - accel_time - cruise_time;
    sample.velocity = std::max(v1, vpeak - accel * decel_elapsed);
    sample.position = accel_dist + cruise_dist + vpeak * decel_elapsed - 0.5 * accel * decel_elapsed * decel_elapsed;

    // 时间耗尽时精确收束到终点（消除数值误差）
    if (elapsed >= sample.duration) {
        sample.position = distance;
        sample.velocity = v1;
    } else {
        sample.position = std::clamp(sample.position, 0.0, distance);
    }
    return sample;
}

/**
 * @brief 平移速度限幅：保持方向不变，将速度大小限制在 max_velocity 以内
 * @param velocity     [in/out] 世界系或机体系的速度向量，原地修改
 * @param max_velocity 速度上限 (m/s)
 */
void clamp_translation(Eigen::Vector2d& velocity, double max_velocity) {
    const double limit = std::max(max_velocity, 0.0);
    const double norm = velocity.norm();
    if (limit > 0.0 && norm > limit) {
        velocity *= limit / norm;
    }
}

/**
 * @brief 执行器死区补偿（标量版）
 *
 * 当误差存在但 P 控制器输出幅值太小（低于底盘最小可动速度）时，
 * 将命令强制提升至 minimum 级别，方向保持与命令（或误差）一致。
 *
 * @param command [in/out] 速度指令值（角速度或线速度分量）
 * @param error   当前跟踪误差
 * @param minimum 死区阈值，命令低于此值但误差存在时上提到此值
 */
void apply_minimum(double& command, double error, double minimum) {
    if (std::abs(error) <= kTinyError || minimum <= 0.0) {
        return;
    }
    // 补偿执行器死区：有误差时，命令幅值不能小于最小可动速度
    if (std::abs(command) < minimum) {
        // 保持符号一致：command==0 时取 error 的符号，否则保持 command 原有符号
        command = std::copysign(minimum, command == 0.0 ? error : command);
    }
}

/**
 * @brief 执行器死区补偿（向量版，用于平移速度）
 *
 * 当误差存在但速度向量模长小于 minimum 时：
 * - 若原速度非零：保持方向，放大模长至 minimum
 * - 若原速度为零：以误差方向输出 minimum 大小的速度
 *
 * @param velocity [in/out] 速度向量
 * @param error    位置误差向量
 * @param minimum  最小速度阈值
 */
void apply_translation_minimum(Eigen::Vector2d& velocity, const Eigen::Vector2d& error, double minimum) {
    if (error.norm() <= kTinyError || minimum <= 0.0) {
        return;
    }

    const double norm = velocity.norm();
    if (norm > minimum) {
        return;  // 速度已经足够大，不需要补偿
    }

    // 速度太小但非零：保持方向，放大到 minimum
    if (norm > kTinyError) {
        velocity *= minimum / norm;
    } else {
        // 速度为零：以误差方向输出 minimum 大小的速度
        velocity = error.normalized() * minimum;
    }
}

/**
 * @brief 判断平移速度是否超过最小阈值（用于决策是否还需要死区补偿）
 * @return true  速度足够大，无需死区补偿
 *         false 速度太小（但误差可能存在），需要 apply_translation_minimum
 */
bool translation_is_above_minimum(const Eigen::Vector2d& velocity, double minimum) {
    return minimum > 0.0 && velocity.norm() > minimum;
}

}  // namespace

// ====================================================================
// Pilot 公共接口实现
// ====================================================================

Pilot::Pilot(rclcpp::Node::SharedPtr node)
    : node_(std::move(node)) {
}

Pilot::~Pilot() = default;

/**
 * @brief 角度归一化到 [-π, π]
 *
 * 例如 3π  → π,  -3π → -π,  270° → -90°
 */
double Pilot::normalize_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

/**
 * @brief 对称限幅：|value| ≤ |limit|
 * @param value 输入值（可正可负）
 * @param limit 限幅上限（取绝对值）
 * @return 限幅后的值；limit ≤ 0 时返回 0
 */
double Pilot::clamp_abs(double value, double limit) {
    const double abs_limit = std::abs(limit);
    if (abs_limit <= 0.0) {
        return 0.0;
    }
    return std::clamp(value, -abs_limit, abs_limit);
}

/**
 * @brief 世界坐标系 → 机器人机体坐标系 旋转
 * @param vector 世界系下的向量（位置误差或速度）
 * @param yaw    机器人当前在世界系下的偏航角
 * @return 机体系下的向量（x 前方, y 左方），可直接赋值给 Cmd.vx/vy
 *
 * 旋转矩阵 R(-yaw)，即把世界系向量反向旋转 yaw 角度，
 * 得到"从机器人视角看"该向量的分量。
 */
Eigen::Vector2d Pilot::world_to_body(const Eigen::Vector2d& vector, double yaw)
{
    Eigen::Rotation2Dd R(-yaw);
    return R * vector;
}

/**
 * @brief 构造站立控制指令
 * @return mode=1（位控站立），所有速度为零的 Cmd
 */
robot_msgs::msg::Cmd Pilot::stand_command() const {
    robot_msgs::msg::Cmd cmd;
    cmd.mode = kStandMode;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

/**
 * @brief 构造行走零速指令
 * @return mode=2（普通行走），但所有速度为零的 Cmd
 *
 * 与 stand_command 的区别是 mode 为行走模式，底盘不会进入位控锁定。
 * 用于 Finished 且 stop_when_finished_=false 的场景。
 */
robot_msgs::msg::Cmd Pilot::walk_zero_command() const {
    robot_msgs::msg::Cmd cmd;
    cmd.mode = kWalkMode;
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.vz = 0.0f;
    cmd.wheel_vel = 0.0f;
    return cmd;
}

/**
 * @brief 重置所有执行状态
 *
 * set_target 时调用，将机芯回初始状态：
 * - 代次递增（使旧回调失效）
 * - 索引和目标归零
 * - 清空过渡数据和回调
 */
void Pilot::reset_execution() {
    ++generation_;
    current_index_ = 0;
    state_ = PilotState::Idle;
    adjust_phase_ = AdjustPhase::Position;
    resume_state_ = PilotState::Running;
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = 0.0;
    aiming_done_ = false;
    aiming_pi_turn_latched_ = false;
    aiming_pi_turn_sign_ = 1.0;
    transition_ = CubicTransition{};
    finished_cb_ = nullptr;
    stop_when_finished_ = true;
}

/**
 * @brief 初始化当前轨迹段的时间基准和瞄准状态
 *
 * 每次开始新的一段（直线或过渡）时调用：
 * - 以当前机器人位姿作为速度规划的起点
 * - 根据目标点 allow_y_vel 初始化 aiming_done_
 * - 清空过渡数据
 *
 * 设计意义：每次规划都以"此刻"的真实位置和时间为基准，
 * 避免 stop/start 后继续追赶旧的时间轴、导致速度跳变。
 */
void Pilot::begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed) {
    segment_start_pos_ = current_pos_;
    segment_start_yaw_ = current_yaw_;
    segment_start_speed_ = start_speed;
    segment_start_time_ = time;
    transition_ = CubicTransition{};
    aiming_pi_turn_latched_ = false;
    aiming_pi_turn_sign_ = 1.0;

    if (current_index_ < targets_.size()) {
        // allow_y_vel=true 可直接边走边转
        // allow_y_vel=false 需要先原地瞄准 path_yaw，aiming_done_=false 会触发瞄准逻辑
        aiming_done_ = targets_[current_index_].allow_y_vel;
    } else {
        aiming_done_ = true;
    }
}

/**
 * @brief 完成当前目标点，推进到下一目标或进入微调
 *
 * 两种情况：
 * 1. 还有后续目标 → 直接切到下一段（中间目标不做 Adjusting 微调）
 * 2. 已经是最后一个目标 → 进入 Adjusting 状态做最终到位修正
 *
 * 第 1 种情况下的起点直接设为上一目标点 target_pos（而非当前实际位置），
 * 因为我们假定上一段已经到位。
 */
void Pilot::finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time) {
    if (current_index_ + 1 < targets_.size()) {
        // 还有后续目标：直接切换到下一段
        const auto previous_target = targets_[current_index_];
        ++current_index_;
        // 中间目标点不做最终微调，以下一段目标的理论位置为起点
        segment_start_pos_ = previous_target.target_pos;
        segment_start_yaw_ = current_yaw_;
        segment_start_speed_ = std::max(0.0f, previous_target.target_vel);
        segment_start_time_ = time;
        transition_ = CubicTransition{};
        aiming_done_ = targets_[current_index_].allow_y_vel;
        aiming_pi_turn_latched_ = false;
        aiming_pi_turn_sign_ = 1.0;
        state_ = PilotState::Running;
        return;
    }

    // 最后一个目标：进入最终微调阶段
    state_ = PilotState::Adjusting;
    adjust_phase_ = AdjustPhase::Position;  // 先做位置微调
    transition_ = CubicTransition{};
}

/**
 * @brief 启动轨迹执行
 *
 * 三种场景：
 * 1. 初始启动 → generation_+1，从 current_pos_ 开始规划第一段
 * 2. Paused 恢复 → 从暂停位置继续（不清空轨迹）
 * 3. Finished 后重新启动 → 索引归零，从头再来
 *
 * @return targets_ 为空时返回 false，否则 true
 */
bool Pilot::start(std::function<void(int success)> finished_cb, bool stop_when_finished) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (targets_.empty()) {
        return false;
    }

    ++generation_;
    finished_cb_ = std::move(finished_cb);
    stop_when_finished_ = stop_when_finished;
    const auto now = std::chrono::high_resolution_clock::now();

    if (state_ == PilotState::Paused) {
        // 从暂停恢复：恢复之前的状态（Running/Adjusting），
        // 若恢复为 Running 则重新校准时间基准
        state_ = resume_state_ == PilotState::Finished ? PilotState::Running : resume_state_;
        if (state_ == PilotState::Running) {
            begin_current_segment(now, 0.0);
        }
        return true;
    }

    // 已完成或索引越界 → 从头开始
    if (state_ == PilotState::Finished || current_index_ >= targets_.size()) {
        current_index_ = 0;
    }

    state_ = PilotState::Running;
    begin_current_segment(now, 0.0);
    return true;
}

/**
 * @brief 停止执行
 *
 * 暂停机制：
 * - Running/Adjusting → 记录 resume_state_ 后切到 Paused（get_command 会输出站立）
 * - 其他非 Finished 状态 → 切到 Idle
 * - Finished 不处理（已经完成的任务不需要 stop）
 */
bool Pilot::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == PilotState::Running || state_ == PilotState::Adjusting) {
        // 记录暂停前状态，get_command 会因 Paused 输出站立零速
        resume_state_ = state_;
        state_ = PilotState::Paused;
    } else if (state_ != PilotState::Finished) {
        state_ = PilotState::Idle;
    }
    transition_ = CubicTransition{};
    return true;
}

/**
 * @brief 带代次检查的停止
 *
 * 场景：行为树的异步节点发出 stop 请求时，可能新的 set_target+start 已经发生。
 * 此时旧请求不应起作用，代次不匹配即直接忽略。
 */
bool Pilot::stop_if_generation_matches(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_) {
        return false;  // 代次不匹配，忽略
    }

    if (state_ == PilotState::Running || state_ == PilotState::Adjusting) {
        resume_state_ = state_;
        state_ = PilotState::Paused;
    } else {
        state_ = PilotState::Idle;
    }
    transition_ = CubicTransition{};
    return true;
}

/**
 * @brief 带代次检查的设置"完成后站立"
 *
 * 场景：轨迹完成后，异步定时器到期才允许切到站立模式。
 * 如果期间已经开始了新轨迹（generation 已变），则安全忽略。
 */
bool Pilot::enable_stop_when_finished_if_generation_matches(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (generation != generation_) {
        return false;
    }

    if (state_ != PilotState::Finished) {
        return false;
    }

    stop_when_finished_ = true;
    return true;
}

/// @brief 获取当前代次编号（线程安全）
std::uint64_t Pilot::generation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return generation_;
}

/**
 * @brief 设置多段目标轨迹
 * @param target 目标点序列
 * @return target 非空时返回 true
 * @note 会触发 reset_execution()，清空所有运行中状态
 */
bool Pilot::set_target(const std::vector<TargetPoint>& target) {
    std::lock_guard<std::mutex> lock(mutex_);
    targets_ = target;
    reset_execution();
    return !targets_.empty();
}

/// @brief 设置单个目标点（包装为单元素 vector 调用上述重载版本）
bool Pilot::set_target(const TargetPoint& target) {
    return set_target(std::vector<TargetPoint>{target});
}

/**
 * @brief 更新机器人位姿（由控制定时器线程高频调用）
 * @param pos 世界系位置
 * @param yaw 偏航角（内部归一化到 [-π, π]）
 */
void Pilot::set_state(const Eigen::Vector2d& pos, const float& yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_pos_ = pos;
    current_yaw_ = static_cast<float>(normalize_angle(yaw));
}

// ====================================================================
// get_command —— 核心控制入口（每帧调用）
// ====================================================================
//
// 整体流程（按状态分支）：
//
//   ┌─ Idle/Paused ────→ stand_command()（站立不动）
//   │
//   ├─ Finished ───────→ stop_when_finished_? stand : walk_zero
//   │
//   ├─ Running ────────→ 1. 检查是否需要原地瞄准（allow_y_vel=false && !aiming_done_）
//   │                    2. 梯形速度规划得到参考位置/速度（前馈）
//   │                    3. 检查是否需要 CubicTransition（多段衔接）
//   │                    4. P 反馈：velocity_body = world_to_body(ref_vel) + kp * error_body
//   │                    5. 限幅 + 死区补偿 → 输出 Cmd
//   │                    6. 时间耗尽 → finish_current_target()
//   │
//   └─ Adjusting ──────→ 三步循环：Position → Yaw → FinalPosition
//                         全部满足 → Finished + 触发 finished_cb
//
// 设计要点：
// - world_to_body() 将所有世界系量转换到机体系，故 Cmd 直接使用 body 速度
// - 死区补偿 (apply_minimum / apply_translation_minimum) 确保微小误差也能产生运动
// - CubicTransition 保证多段轨迹位置和速度连续，消除连接点的速度跳变
// ====================================================================
robot_msgs::msg::Cmd Pilot::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time) {
    // 回调不能在锁内调用（可能导致死锁），先存下来，锁释放后再调用
    std::function<void(int)> callback_to_call;
    robot_msgs::msg::Cmd cmd = walk_zero_command();

    {
        std::lock_guard<std::mutex> lock(mutex_);

        // ---- 快速路径：无需执行轨迹的状态 ----
        // Idle / Paused：输出站立，机器人锁定不动
        if (targets_.empty() || state_ == PilotState::Idle || state_ == PilotState::Paused) {
            return stand_command();
        }
        // Finished：按配置决定用站立还是行走零速（stop_when_finished_）
        if (state_ == PilotState::Finished) {
            return stop_when_finished_ ? stand_command() : walk_zero_command();
        }
        // 防御：索引异常
        if (current_index_ >= targets_.size()) {
            return stand_command();
        }

        // 当前正在执行的目标点（通过引用直接修改 target_pos 之外的运行时无关字段）
        TargetPoint& target = targets_[current_index_];

        // ============================================================
        // 状态分支 1：Adjusting —— 最终微调阶段
        // ============================================================
        if (state_ == PilotState::Adjusting) {
            // ---- Adjusting 阶段逻辑 ----
            // 三步微调：Position → Yaw → FinalPosition，用 for 循环实现状态推进
            // 每步满足条件后自动 fall-through 到下一步，全部满足则 finish_target

            // 计算当前误差
            const Eigen::Vector2d pos_error_world = target.target_pos - current_pos_;  // 世界系位置误差
            const Eigen::Vector2d pos_error_body = world_to_body(pos_error_world, current_yaw_);  // 转为机体系
            const double yaw_error = target.constraint_target_yaw ? normalize_angle(target.target_yaw - current_yaw_) : 0.0;
            const bool pos_ok = pos_error_world.norm() <= static_cast<double>(target.allow_final_pos_allow);
            const bool yaw_ok = !target.constraint_target_yaw || std::abs(yaw_error) <= static_cast<double>(target.allow_final_dir_error);

            /// 全部微调完成，标记为 Finished 并准备回调
            const auto finish_target = [&]() {
                state_ = PilotState::Finished;
                current_index_ = targets_.size();
                callback_to_call = std::move(finished_cb_);
                cmd = stop_when_finished_ ? stand_command() : walk_zero_command();
            };

            /// 位置微调：P 控制 + 限幅 + 死区补偿，只输出平移速度，不旋转
            const auto set_position_adjust_command = [&]() {
                Eigen::Vector2d velocity_body = Eigen::Vector2d::Zero();
                // P 控制：v = kp * error（在机体系下直接计算，无需 world_to_body）
                velocity_body.x() = target.kp.x() * pos_error_body.x();
                velocity_body.y() = target.kp.y() * pos_error_body.y();
                clamp_translation(velocity_body, target.max_velocity);
                // 死区补偿：如果 P 输出太小导致底盘无法响应，提升至最小可动速度
                const bool translation_above_minimum = translation_is_above_minimum(velocity_body, target.adjust_min_vel);
                if (!translation_above_minimum) {
                    apply_translation_minimum(velocity_body, pos_error_body, target.adjust_min_vel);
                    clamp_translation(velocity_body, target.max_velocity);
                }

                cmd.vx = static_cast<float>(velocity_body.x());
                cmd.vy = static_cast<float>(velocity_body.y());
                cmd.vz = 0.0f;  // 位置微调期间不旋转
            };

            /// 方向微调：原地旋转到 target_yaw，输出纯角速度，平移速度为零
            const auto set_yaw_adjust_command = [&]() {
                double omega = 0.0;
                if (target.constraint_target_yaw && !yaw_ok) {
                    omega = target.kp.z() * yaw_error;
                    apply_minimum(omega, yaw_error, target.adjust_min_omega);
                }
                omega = clamp_abs(omega, target.max_omega);

                cmd.vx = 0.0f;  // 原地旋转，平移速度为零
                cmd.vy = 0.0f;
                cmd.vz = static_cast<float>(omega);
            };

            // 三步微调的状态机循环（最多迭代 3 步，用 for+continue 实现 fall-through）
            for (int step = 0; step < 3; ++step) {
                // Step 1: Position —— 平移逼近目标点
                if (adjust_phase_ == AdjustPhase::Position) {
                    if (!pos_ok) {
                        set_position_adjust_command();
                        break;  // 还没到位，下次继续
                    }
                    // 位置已到位：跳到下一步（Yaw 或 FinalPosition）
                    adjust_phase_ = target.constraint_target_yaw ? AdjustPhase::Yaw : AdjustPhase::FinalPosition;
                    continue;  // fall-through 到下一 case
                }

                // Step 2: Yaw —— 原地对准方向（仅 constraint_target_yaw=true）
                if (adjust_phase_ == AdjustPhase::Yaw) {
                    if (!yaw_ok) {
                        set_yaw_adjust_command();
                        break;  // 还没对准，下次继续
                    }
                    // 方向已对准：跳到最终位置复检
                    adjust_phase_ = AdjustPhase::FinalPosition;
                    continue;  // fall-through
                }

                // Step 3: FinalPosition —— 复检位置（转向可能导致漂移）
                if (!pos_ok) {
                    set_position_adjust_command();
                    break;
                }
                // 全部满足！标记完成
                finish_target();
                break;
            }
        } else {
            // ============================================================
            // 状态分支 2：Running —— 轨迹规划 + P 反馈控制
            // ============================================================

            // 初始化本帧的参考量（前馈项），后续按 transition_ 或梯形规划覆写
            Eigen::Vector2d reference_pos = target.target_pos;
            Eigen::Vector2d reference_vel = Eigen::Vector2d::Zero();
            double reference_yaw = target.constraint_target_yaw ? target.target_yaw : current_yaw_;
            double reference_omega = 0.0;
            double segment_duration = kMinDuration;
            double segment_progress = 1.0;  // [0,1] 当前段完成比例

            // ---- Running 子分支 A：CubicTransition 三次 Hermite 过渡 ----
            if (transition_.active) {
                // 三次 Hermite 曲线：P(u)=h00*P0+h10*T*V0+h01*P1+h11*T*V1, u∈[0,1]
                // 保证起终点位置和速度都连续 → 前一段和下一段之间无速度跳变
                const double elapsed = std::chrono::duration<double>(time - transition_.start_time).count();
                const double duration = std::max(transition_.duration, kMinDuration);
                const double u = std::clamp(elapsed / duration, 0.0, 1.0);  // 归一化进度 [0,1]
                const double u2 = u * u;
                const double u3 = u2 * u;

                // Hermite 基函数（位置）：h00,h10 控制起点；h01,h11 控制终点
                const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;   // h00(0)=1, h00(1)=0
                const double h10 = u3 - 2.0 * u2 + u;            // h10(0)=0, h10(1)=0
                const double h01 = -2.0 * u3 + 3.0 * u2;         // h01(0)=0, h01(1)=1
                const double h11 = u3 - u2;                       // h11(0)=0, h11(1)=0

                // Hermite 基函数的导数（除以 duration 后得到速度系数）
                const double dh00 = 6.0 * u2 - 6.0 * u;          // dh00(0)=0, dh00(1)=0
                const double dh10 = 3.0 * u2 - 4.0 * u + 1.0;    // dh10(0)=1, dh10(1)=0
                const double dh01 = -6.0 * u2 + 6.0 * u;         // dh01(0)=0, dh01(1)=0
                const double dh11 = 3.0 * u2 - 2.0 * u;           // dh11(0)=0, dh11(1)=1

                // 参考位置 = 基函数 × 约束值
                reference_pos = h00 * transition_.start_pos + h10 * duration * transition_.start_vel +
                                h01 * transition_.end_pos + h11 * duration * transition_.end_vel;
                // 参考速度 = 基函数导数 × 约束值 / duration
                reference_vel = (dh00 * transition_.start_pos + dh10 * duration * transition_.start_vel +
                                 dh01 * transition_.end_pos + dh11 * duration * transition_.end_vel) /
                                duration;

                // 朝向也做线性插值 + 恒定参考角速度
                const double yaw_delta = normalize_angle(transition_.end_yaw - transition_.start_yaw);
                reference_yaw = normalize_angle(transition_.start_yaw + yaw_delta * u);
                reference_omega = yaw_delta / duration;

                // 过渡完成：推进到下一个目标点
                if (elapsed >= duration) {
                    ++current_index_;
                    segment_start_pos_ = transition_.end_pos;
                    segment_start_yaw_ = static_cast<float>(transition_.end_yaw);
                    segment_start_speed_ = transition_.end_vel.norm();
                    segment_start_time_ = time;
                    transition_ = CubicTransition{};  // 清除过渡数据
                    aiming_done_ = true;
                    aiming_pi_turn_latched_ = false;
                    aiming_pi_turn_sign_ = 1.0;
                    // 过渡完成后若已无更多目标，则进入最终微调
                    if (current_index_ >= targets_.size()) {
                        state_ = PilotState::Adjusting;
                        adjust_phase_ = AdjustPhase::Position;
                    }
                }
            } else {
                // ---- Running 子分支 B：梯形速度规划直线段 ----
                // 沿 segment_start_pos_ → target.target_pos 方向做一维速度规划

                Eigen::Vector2d segment_vec = target.target_pos - segment_start_pos_;
                double distance = segment_vec.norm();

                // 距离太小，直接结束当前段
                if (distance <= kMinDistance) {
                    finish_current_target(time);
                    return cmd;
                }

                // 计算运动方向（单位向量）和路径偏航角
                Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
                if (distance > kMinDistance) {
                    direction = segment_vec / distance;
                }
                const double path_yaw = std::atan2(direction.y(), direction.x());  // 路径方向角

                // ---- 原地瞄准阶段（allow_y_vel=false && !aiming_done_） ----
                if (!target.allow_y_vel && !aiming_done_) {
                    // 非横移模式：先原地旋转对准 path_yaw，满足阈值后开始直线前进
                    const double raw_yaw_error = normalize_angle(path_yaw - current_yaw_);
                    double yaw_error = raw_yaw_error;
                    const bool near_pi = std::abs(std::abs(raw_yaw_error) - kPi) < kPiTurnDeadband;
                    if (near_pi) {
                        if (!aiming_pi_turn_latched_) {
                            aiming_pi_turn_sign_ = current_yaw_ < 0.0f ? 1.0 : -1.0;
                            aiming_pi_turn_latched_ = true;
                            RCLCPP_INFO(
                                node_->get_logger(),
                                "Pilot 初始瞄准进入 180 度临界区: index=%zu, current_yaw=%.4f, path_yaw=%.4f, raw_error=%.4f, turn_sign=%.0f",
                                current_index_,
                                current_yaw_,
                                path_yaw,
                                raw_yaw_error,
                                aiming_pi_turn_sign_);
                        }
                        yaw_error = std::copysign(std::abs(raw_yaw_error), aiming_pi_turn_sign_);
                    }
                    if (std::abs(yaw_error) < static_cast<double>(target.allow_start_dir_error)) {
                        const bool pi_turn_was_latched = aiming_pi_turn_latched_;
                        // 瞄准完成，重新校准时间基准，开始直线段
                        begin_current_segment(time, 0.0);
                        // ⚠️ begin_current_segment 会按 allow_y_vel 初始化 aiming_done_，
                        // 所以这里必须重新置 true，否则 allow_y_vel=false 会永远卡在瞄准阶段
                        aiming_done_ = true;
                        segment_vec = target.target_pos - segment_start_pos_;
                        distance = segment_vec.norm();
                        if (pi_turn_was_latched) {
                            RCLCPP_INFO(
                                node_->get_logger(),
                                "Pilot 180 度初始瞄准完成: index=%zu, current_yaw=%.4f, path_yaw=%.4f",
                                current_index_,
                                current_yaw_,
                                path_yaw);
                        }
                    } else {
                        // 还在瞄准：输出纯旋转，平移速度为零
                        double omega = target.kp.z() * yaw_error;
                        apply_minimum(omega, yaw_error, target.adjust_min_omega);
                        cmd.vx = 0.0f;
                        cmd.vy = 0.0f;
                        cmd.vz = static_cast<float>(clamp_abs(omega, target.max_omega));
                        return cmd;
                    }
                }

                // ---- 梯形速度规划 ----
                const double elapsed = std::chrono::duration<double>(time - segment_start_time_).count();

                // 前馈项：梯形速度曲线采样，返回 (参考位置, 参考速度, 规划总时长)
                const MotionSample profile = sample_trapezoid_profile(
                    distance,
                    elapsed,
                    segment_start_speed_,
                    std::max(0.0f, target.target_vel),
                    target.max_velocity > 0.0f ? target.max_velocity : kDefaultVelocity,
                    target.max_accelation > 0.0f ? target.max_accelation : kDefaultAccel);

                segment_duration = profile.duration;
                segment_progress = distance > kMinDistance ? std::clamp(profile.position / distance, 0.0, 1.0) : 1.0;

                // 沿 direction 方向展开参考位置和速度（世界系）
                reference_pos = segment_start_pos_ + direction * profile.position;
                reference_vel = direction * profile.velocity;

                // ---- 朝向参考量计算 ----
                if (!target.allow_y_vel) {
                    // 非横移模式：直线前进时保持朝向 path_yaw，最终 target_yaw 留给 Adjusting 阶段
                    reference_yaw = path_yaw;
                    reference_omega = 0.0;
                } else if (target.constraint_target_yaw) {
                    // 横移模式 + 约束朝向：yaw 按 segment_progress 线性插值
                    const double yaw_delta = normalize_angle(target.target_yaw - segment_start_yaw_);
                    reference_yaw = normalize_angle(segment_start_yaw_ + yaw_delta * segment_progress);
                    reference_omega = segment_duration > kMinDuration ? yaw_delta / segment_duration : 0.0;
                } else {
                    // 横移模式 + 不约束朝向：朝向保持当前不变，不做干预
                    reference_yaw = current_yaw_;
                }

                // ---- 多段轨迹衔接：检测是否需要 CubicTransition ----
                const float connection_radius = target.trajectory_connection_radius;
                if (current_index_ + 1 < targets_.size() && connection_radius > 0.0f &&
                    (target.target_pos - current_pos_).norm() < static_cast<double>(connection_radius)) {
                    // 进入衔接半径：提前准备下一段方向，用 Hermite 曲线平滑过渡
                    const TargetPoint& next_target = targets_[current_index_ + 1];
                    const Eigen::Vector2d next_vec = next_target.target_pos - target.target_pos;
                    const double next_distance = next_vec.norm();
                    if (next_distance > kMinDistance) {
                        const Eigen::Vector2d next_direction = next_vec / next_distance;

                        // 过渡距离 = min(衔接半径, 到下一目标的一半距离)
                        const double blend_distance = std::min(static_cast<double>(connection_radius), next_distance * 0.5);
                        // 过渡速度 = max(当前速度, 下一段目标速度, 0.2) —— 保证过渡段不会因为速度太小而耗时过长
                        const double blend_speed = std::max({profile.velocity, static_cast<double>(next_target.target_vel), 0.2});

                        // 配置 CubicTransition 参数
                        transition_.active = true;
                        transition_.start_time = time;
                        transition_.start_pos = reference_pos;       // 过渡起点 = 当前参考位置
                        transition_.start_vel = reference_vel;       // 过渡起点速度 = 当前参考速度
                        transition_.end_pos = target.target_pos + next_direction * blend_distance;  // 向下一段方向延伸
                        transition_.end_vel = next_direction * std::min(
                            blend_speed,
                            static_cast<double>(next_target.max_velocity > 0.0f ? next_target.max_velocity : kDefaultVelocity));
                        // 过渡时长 = 距离 / 速度（至少 kMinDuration）
                        transition_.duration = std::max(
                            kMinDuration,
                            (transition_.end_pos - transition_.start_pos).norm() /
                                std::max(transition_.start_vel.norm(), 0.2));
                        // 朝向过渡：从当前参考朝向到下一段方向
                        transition_.start_yaw = static_cast<float>(reference_yaw);
                        transition_.end_yaw = next_target.constraint_target_yaw
                                                  ? next_target.target_yaw
                                                  : static_cast<float>(std::atan2(next_direction.y(), next_direction.x()));
                    }
                }

                // 规划时间耗尽且未进入过渡 → 完成当前目标
                if (elapsed >= profile.duration && !transition_.active) {
                    finish_current_target(time);
                    return cmd;
                }
            }

            // ============================================================
            // 最终控制律：前馈 + 反馈 → 机体系速度指令
            // ============================================================
            // velocity_body = world_to_body(ref_vel) + kp * world_to_body(ref_pos - current_pos)
            //
            // 即：规划速度（前馈） + P 反馈修正（在机体系下叠加）
            //
            // 为什么分两次 world_to_body？
            // - 前馈速度是规划出的世界系速度，旋转后得到"机器人应沿什么方向多快移动"
            // - 位置误差是世界系向量，旋转后得到"机器人视角的目标方向偏差"
            // - 两者在机体系下相加，避免世界系下不同方向的分量耦合问题

            const Eigen::Vector2d pos_error_body = world_to_body(reference_pos - current_pos_, current_yaw_);
            Eigen::Vector2d velocity_body = world_to_body(reference_vel, current_yaw_);  // 前馈项

            // 叠加 P 反馈（机体系）：v += kp * error
            velocity_body.x() += target.kp.x() * pos_error_body.x();
            velocity_body.y() += target.kp.y() * pos_error_body.y();

            // ---- 平移速度限幅与死区补偿 ----
            const Eigen::Vector2d target_error_body = world_to_body(target.target_pos - current_pos_, current_yaw_);
            clamp_translation(velocity_body, target.max_velocity);
            const bool translation_above_minimum = translation_is_above_minimum(velocity_body, target.adjust_min_vel);
            if (!translation_above_minimum) {
                // 速度太小但目标还没到 → 死区补偿，强制输出最小速度
                apply_translation_minimum(velocity_body, target_error_body, target.adjust_min_vel);
                clamp_translation(velocity_body, target.max_velocity);
            }

            // ---- 角速度：前馈 + 反馈 ----
            const double yaw_error = normalize_angle(reference_yaw - current_yaw_);
            double omega = reference_omega + target.kp.z() * yaw_error;  // 前馈 + P 反馈
            if (!translation_above_minimum) {
                // 平移速度很小时也对角速度做死区补偿
                apply_minimum(omega, yaw_error, target.adjust_min_omega);
            }
            omega = clamp_abs(omega, target.max_omega);

            // 组装最终指令
            cmd.vx = static_cast<float>(velocity_body.x());
            cmd.vy = static_cast<float>(velocity_body.y());
            cmd.vz = static_cast<float>(omega);
        }
    }  // 锁释放

    // 回调在锁外调用，避免死锁
    if (callback_to_call) {
        callback_to_call(1);  // success=1 表示正常完成
    }
    return cmd;
}
