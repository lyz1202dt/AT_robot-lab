#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/cmd.hpp>

/**
 * @brief Pilot —— 机器人轨迹规划与闭环控制器
 *
 * Pilot 是任务层（行为树/状态机）与底层控制（Cmd 话题）之间的中间层。
 * 它接收一组目标点序列（TargetPoint），在每一帧的 get_command() 中产出
 * 机器人速度指令（Cmd），驱动底盘沿规划轨迹运动。
 *
 * 核心能力：
 * 1. 轨迹规划 —— 梯形/三角速度曲线，多段轨迹之间用三次 Hermite 曲线平滑过渡。
 * 2. 闭环控制 —— 前馈（规划速度）+ 反馈（P 控制器），在机体系下叠加。
 * 3. 状态机 —— Idle → Running → Adjusting → Finished，支持暂停/恢复/代次管理。
 *
 * 坐标系约定：
 * - 世界系（world/map）：位置 target_pos、current_pos_ 均为世界坐标。
 * - 机体系（body）：Cmd.vx/vy 是机器人自身坐标系下的速度指令，x 前方，y 左方。
 *
 * 线程安全：
 * - set_state() 由控制定时器线程调用（高频），get_command() / start() / stop()
 *   可能由行为树线程调用，所有公共接口均加 mutex_ 保护。
 */
class Pilot {
public:
    /**
     * @brief 单个轨迹目标点的配置
     *
     * 一个 TargetPoint 描述机器人从当前位置运动到 target_pos 的全部运动约束。
     * 注意：虽然叫"目标点"，但 max_velocity/max_accelation 等参数在整段轨迹
     * 执行期间都生效，并非只在到达目标的瞬间才起作用。
     */
    struct TargetPoint {
        /// 目标点在世界坐标系下的位置 (x, y)，单位：米
        Eigen::Vector2d target_pos{Eigen::Vector2d::Zero()};
        /// 到达目标后机器人应朝向的偏航角，单位：弧度。仅在 constraint_target_yaw=true 时生效
        float target_yaw{0.0f};
        /// 是否约束最终朝向。true 时 Adjusting 阶段会原地转到位
        bool constraint_target_yaw{false};
        /// 到达目标点后期望保留的前进速度，单位：m/s。非零值可用于连续多段的运动衔接
        float target_vel{0.0f};
        /// 直线运动的最大速度上限，单位：m/s。轨迹规划峰值不会超过此值
        float max_velocity{0.7f};
        /// 直线运动的最大加速度上限，单位：m/s²。决定加减速段的斜率
        float max_accelation{0.25f};
        /// 旋转运动的最大角速度上限，单位：rad/s
        float max_omega{1.0f};
        /// P 控制器增益，(kp_x, kp_y, kp_yaw)。前两个作用于平移，第三个作用于旋转
        Eigen::Vector3d kp{0.2, 0.2, 0.5};
        /// 开始行走前允许的方向误差阈值，单位：弧度。
        /// allow_y_vel=false 时生效：机器人必须先原地转向到 path_yaw，误差小于此值后才开始前进
        float allow_start_dir_error{0.2f};
        /// 最终允许的方向误差阈值，单位：弧度。Adjusting 阶段 Yaw 子阶段以此判断是否对准
        float allow_final_dir_error{0.2f};
        /// 最终允许的位置误差阈值，单位：米。Adjusting 阶段 Position 子阶段以此判断是否到位
        float allow_final_pos_allow{0.2f};
        /// 微调阶段平移速度死区补偿值，单位：m/s。
        /// 当位置误差很小导致 P 输出低于此值时，强制以该速度输出，避免底盘因死区无法移动
        float adjust_min_vel{0.25f};
        /// 微调阶段角速度死区补偿值，单位：rad/s
        float adjust_min_omega{0.15f};
        /// 是否允许侧向（y 向）速度。
        /// - true：机器人边走边转，以当前位置→目标点的直线路径前进，yaw 会随进度线性插值到 target_yaw
        /// - false：先原地瞄准 path_yaw → 直线前进（保持朝向）→ Adjusting 阶段再原地对准 target_yaw
        bool allow_y_vel{false};
        /// 多段轨迹衔接半径，单位：米。
        /// 当机器人进入当前目标点该半径范围内且存在下一目标时，
        /// 用三次 Hermite 曲线平滑过渡到下一段方向，避免速度方向在连接点附近跳变
        float trajectory_connection_radius{0.0};
    };

    explicit Pilot(rclcpp::Node::SharedPtr node);
    ~Pilot();

    /**
     * @brief 启动轨迹执行
     * @param finished_cb 全部目标点到达后回调一次，参数 success=1
     * @param stop_when_finished true=完成后输出站立模式(Cmd.mode=1)，false=持续输出行走零速(Cmd.mode=2)
     * @return 有目标点且状态合法时返回 true
     *
     * 如果当前是 Paused 状态，则从暂停处恢复继续执行。
     * 如果当前是 Finished 或索引越界，则从头重新开始。
     */
    bool start(std::function<void(int success)> finished_cb, bool stop_when_finished = true);

    /**
     * @brief 停止执行，机器人进入位控站立
     * @return 总是返回 true
     * @note 和 reset_execution() 不同，stop() 会保存 resume_state_ 以支持 resume
     */
    bool stop();

    /**
     * @brief 带代次检查的停止，避免旧异步回调误停新轨迹
     * @param generation 调用方保存的代次编号，仅当与当前代次匹配时才执行停止
     * @return 代次匹配且实际执行了停止返回 true
     */
    bool stop_if_generation_matches(std::uint64_t generation);

    /**
     * @brief 带代次检查的"完成后站立"开关
     * @param generation 调用方保存的代次编号
     * @return 仅当已完成且代次匹配时才设为 true
     *
     * 用于异步定时器场景：轨迹已经 Finished，定时器触发后才切到站立模式。
     * 如果在此期间已经开启了新轨迹，则直接忽略，避免打断。
     */
    bool enable_stop_when_finished_if_generation_matches(std::uint64_t generation);

    /// 返回当前轨迹执行代次。每次 set_target/start/reset_execution 会使代次递增
    std::uint64_t generation() const;

    /// 设置多段目标轨迹点，会重置所有内部状态
    bool set_target(const std::vector<TargetPoint> &target);

    /// 设置单个目标点（内部包装为单元素 vector）
    bool set_target(const TargetPoint &target);

    /**
     * @brief 更新机器人当前位姿
     * @param pos 世界坐标系下的 (x, y) 位置
     * @param yaw 世界坐标系下的偏航角，单位：弧度
     * @note 由控制定时器线程高频调用（和 Cmd 同一周期），是闭环反馈的感测输入
     */
    void set_state(const Eigen::Vector2d &pos, const float &yaw);

    /**
     * @brief 每帧产出速度指令（核心入口）
     * @param time 当前高精度时间戳，用于轨迹按时间推进
     * @return 机器人速度指令 Cmd（mode, vx, vy, vz, wheel_vel）
     *
     * 内部流程：
     * 1. 检查状态（Idle/Paused → 站立；Finished → 按配置输出站立或行走零速）
     * 2. Running 阶段：梯形速度规划 + P 反馈 → 机体系速度
     * 3. 多段衔接时启用 CubicTransition 三次 Hermite 过渡
     * 4. 规划时间耗尽时切入 Adjusting 阶段
     * 5. Adjusting 阶段：Position → Yaw → FinalPosition 顺序微调
     * 6. 全部完成时分发 finished_cb 回调
     */
    robot_msgs::msg::Cmd get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time);

private:
    /**
     * @brief Pilot 内部状态机
     * - Idle:      初始/空闲状态，输出站立零速
     * - Running:   正沿轨迹运动，每帧做速度规划+P控制
     * - Adjusting: 已到达目标附近，进入最终微调阶段
     * - Paused:    用户暂停（stop() 或代次不匹配），输出站立零速
     * - Finished:  全部目标已完成，按 stop_when_finished_ 决定输出站立 or 行走零速
     */
    enum class PilotState {
        Idle,
        Running,
        Adjusting,
        Paused,
        Finished
    };

    /**
     * @brief Adjusting 阶段的三个子步骤
     * - Position:      先修正位置误差（平移逼近目标点）
     * - Yaw:           再原地修正方向误差（仅 constraint_target_yaw=true 时有效）
     * - FinalPosition: 最后复检位置并做最终修正，防止旋转造成的漂移
     */
    enum class AdjustPhase {
        Position,
        Yaw,
        FinalPosition
    };

    /**
     * @brief 三次 Hermite 曲线过渡参数
     *
     * 当机器人进入当前目标点的 connection_radius 范围且存在下一目标时，
     * 用此过渡平滑衔接两段轨迹，保证位置 C¹ 连续（位置和速度均连续）。
     *
     * 曲线形式：P(u) = h00*P0 + h10*T*V0 + h01*P1 + h11*T*V1
     * 其中 h00~h11 是三次 Hermite 基函数，u ∈ [0,1]，T = duration。
     */
    struct CubicTransition {
        bool active{false};   ///< 是否启用过渡
        std::chrono::time_point<std::chrono::high_resolution_clock> start_time{};
        double duration{0.0};
        Eigen::Vector2d start_pos{Eigen::Vector2d::Zero()};  ///< 过渡起点位置（世界系）
        Eigen::Vector2d end_pos{Eigen::Vector2d::Zero()};    ///< 过渡终点位置（世界系）
        Eigen::Vector2d start_vel{Eigen::Vector2d::Zero()};  ///< 过渡起点速度（世界系）
        Eigen::Vector2d end_vel{Eigen::Vector2d::Zero()};    ///< 过渡终点速度（世界系）
        float start_yaw{0.0f};   ///< 过渡起点朝向
        float end_yaw{0.0f};     ///< 过渡终点朝向
    };

    /// 角度归一化到 [-π, π] 范围
    static double normalize_angle(double angle);
    /// 按绝对值限幅：|value| ≤ |limit|，返回限幅后的值。limit≤0 返回 0
    static double clamp_abs(double value, double limit);
    /// 世界坐标系向量旋转到机器人机体坐标系（绕 z 轴旋转 -yaw）
    static Eigen::Vector2d world_to_body(const Eigen::Vector2d& vector, double yaw);

    /// 构造站立模式指令（mode=1, 全部速度为零）
    robot_msgs::msg::Cmd stand_command() const;
    /// 构造行走零速指令（mode=2, 全部速度为零），保持行走模式但不移动
    robot_msgs::msg::Cmd walk_zero_command() const;

    /// 重置所有执行状态：代次+1，索引归零，清空过渡数据
    void reset_execution();
    /// 以当前机器人位姿为起点，初始化当前轨迹段的时间基准和瞄准状态
    void begin_current_segment(std::chrono::time_point<std::chrono::high_resolution_clock> time, double start_speed);
    /// 完成当前目标点：推进到下一个目标，或进入 Adjusting 阶段
    void finish_current_target(std::chrono::time_point<std::chrono::high_resolution_clock> time);

    rclcpp::Node::SharedPtr node_;

    /// 互斥锁：set_state 由控制定时器线程更新，get_command/start/stop 可能由行为树线程调用
    mutable std::mutex mutex_;

    /// 目标点序列，set_target 写入
    std::vector<TargetPoint> targets_;
    /// 当前正执行的目标点在 targets_ 中的索引
    std::size_t current_index_{0};
    /// 当前状态机状态
    PilotState state_{PilotState::Idle};
    /// Adjusting 阶段当前子步骤
    AdjustPhase adjust_phase_{AdjustPhase::Position};
    /// 暂停前所处的状态，用于 resume 时恢复
    PilotState resume_state_{PilotState::Running};

    /// 机器人当前位置（世界坐标系），由 set_state 高频更新
    Eigen::Vector2d current_pos_{Eigen::Vector2d::Zero()};
    /// 机器人当前偏航角
    float current_yaw_{0.0f};

    /// 当前直线轨迹段的起点位置（世界系）。每次 begin_current_segment 或目标切换时更新
    Eigen::Vector2d segment_start_pos_{Eigen::Vector2d::Zero()};
    /// 当前直线轨迹段的起点朝向
    float segment_start_yaw_{0.0f};
    /// 当前直线轨迹段的起点速度大小
    double segment_start_speed_{0.0};
    /// 瞄准完成标志：allow_y_vel=false 时，true 表示已完成初始瞄准，可以开始直线前进
    bool aiming_done_{false};
    /// 180° 临界瞄准时锁定的旋转方向，避免 yaw 跨 ±π 后角速度反向
    bool aiming_pi_turn_latched_{false};
    double aiming_pi_turn_sign_{1.0};
    /// 当前直线轨迹段的起始时间戳，梯形速度规划以此计算 elapsed
    std::chrono::time_point<std::chrono::high_resolution_clock> segment_start_time_{};

    /// 多段轨迹衔接的三次 Hermite 过渡数据
    CubicTransition transition_;
    /// 全部目标完成时的回调函数
    std::function<void(int success)> finished_cb_;
    /// Finished 状态下是否输出站立模式
    bool stop_when_finished_{true};
    /// 当前执行代次，每次重置推进。用于防御旧异步回调误操作新轨迹
    std::uint64_t generation_{0};
};
