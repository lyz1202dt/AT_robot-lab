#pragma once
// Eigen
#include <Eigen/Dense>

// KDL
#include <kdl_parser/kdl_parser.hpp>
// #include <orocos_kdl/kdl/chain.hpp>

// // OSQP
// #include <OsqpEigen/OsqpEigen.h>

// // qpOASES
// #include <qpOASES.hpp>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp> // ← 你缺的就是它
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// C++ 标准库
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

class LegCalc;

// 腿部控制目标结构体 (替代 ROS 消息)
struct LegTarget {
    struct JointCommand {
        double rad{0.0};    // 关节角度
        double omega{0.0};  // 关节角速度
        double torque{0.0}; // 关节力矩
        double kp{0.0};     // 比例增益
        double kd{0.0};     // 微分增益
    };

    struct WheelCommand {
        double omega{0.0};  // 轮子角速度
        double torque{0.0}; // 轮子力矩
        double kd{0.0};     // 微分增益
    };

    JointCommand joints[3]; // 三个关节
    WheelCommand wheel;     // 轮子
};
struct RobotTarget
{
    LegTarget legs[4];
};

class Robot_t {
public:
    explicit Robot_t(const std::string& urdf_file_path);
    ~Robot_t() = default;



    static constexpr double WHEEL_RADIUS = 0.065;


    // 解算部分
    KDL::Tree tree;
    std::string urdf_xml;
    KDL::Chain lf_leg_chain;
    KDL::Chain rf_leg_chain;
    KDL::Chain lb_leg_chain;
    KDL::Chain rb_leg_chain;
    std::shared_ptr<LegCalc> lf_leg_calc;
    std::shared_ptr<LegCalc> rf_leg_calc;
    std::shared_ptr<LegCalc> lb_leg_calc;
    std::shared_ptr<LegCalc> rb_leg_calc;

    // 狗腿数据缓存
    Eigen::Vector3d lf_joint_pos{0.0, 0.0, 0.0}, lf_joint_vel{0.0, 0.0, 0.0}, lf_joint_torque{0.0, 0.0, 0.0}, lf_forward_torque;
    Eigen::Vector3d rf_joint_pos{0.0, 0.0, 0.0}, rf_joint_vel{0.0, 0.0, 0.0}, rf_joint_torque{0.0, 0.0, 0.0},
        rf_forward_torque{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_joint_pos{0.0, 0.0, 0.0}, lb_joint_vel{0.0, 0.0, 0.0}, lb_joint_torque{0.0, 0.0, 0.0},
        lb_forward_torque{0.0, 0.0, 0.0};
    Eigen::Vector3d rb_joint_pos{0.0, 0.0, 0.0}, rb_joint_vel{0.0, 0.0, 0.0}, rb_joint_torque{0.0, 0.0, 0.0},
        rb_forward_torque{0.0, 0.0, 0.0};
    double lf_wheel_omega{0.0}, rf_wheel_omega{0.0}, lb_wheel_omega{0.0}, rb_wheel_omega{0.0};
    double lf_wheel_torque{0.0}, rf_wheel_torque{0.0}, lb_wheel_torque{0.0}, rb_wheel_torque{0.0};


    Eigen::Vector3d lf_base_offset{0.0, 0.0, 0.0}, rf_base_offset{0.0, 0.0, 0.0}, lb_base_offset{0.0, 0.0, 0.0},
        rb_base_offset{0.0, 0.0, 0.0};
    double body_height{0.25};
    double robot_lf_grivate{0.0};
    double robot_rf_grivate{0.0};
    double robot_lb_grivate{0.0};
    double robot_rb_grivate{0.0};
    std::vector<double> kp, kd;
    double wheel_kd;
};



class LegCalc {
public:
    LegCalc(
    KDL::Chain& chain, const std::vector<double>& kp_list = {3.0, 2.8, 2.8}, const std::vector<double>& kd_list = {0.17, 0.14, 0.11},
    double wheel_kd_param = 0.5);
    ~LegCalc();

    void set_init_joint_pos(const Eigen::Vector3d init_joint_pos);
    Eigen::Vector3d joint_pos_setarray(const Eigen::Vector3d init_joint_pos);

    // int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
    Eigen::Vector3d joint_pos(const Eigen::Vector3d& foot_pos, int* result); // 稍后需要在线安装IK求解器（手推的解析求解器或者数值迭代器）


    Eigen::Vector3d foot_force(
        const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_torque,
        const Eigen::Vector3d& forward_torque = Eigen::Vector3d(0.0, 0.0, 0.0));

    Eigen::Vector3d joint_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_vel);

    Eigen::Vector3d
        joint_torque_dynamic(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega, const Eigen::Vector3d& foot_acc);

    Eigen::Vector3d
        joint_torque_foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_force); // 由足端期望力计算的关节力矩

    Eigen::Vector3d foot_pos(const Eigen::Vector3d& joint_rad);

    void set_joint_pd(int index, double kp, double kd);

    void get_joint_pd(int index, double& kp, double& kd);

    LegTarget signal_leg_calc(
        const Eigen::Vector3d& exp_cart_pos, const Eigen::Vector3d& exp_cart_vel, const Eigen::Vector3d& exp_cart_acc,
        const Eigen::Vector3d& exp_cart_force, Eigen::Vector3d* torque, const double wheel_vel = 0.0, const double wheel_force = 0.0);

    Eigen::Vector3d pos_offset;                                                                       // 足端位置到机器人中心的偏移
    double kp[3], kd[3];
    double wheel_kd;

private:
    Eigen::Vector3d joint_acc(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_vel, Eigen::Vector3d foot_acc);

    Eigen::Matrix<double, 3, 3> get_3x3_jacobian_(const KDL::Jacobian& full_jacobian); // 从KDL库中求出我们感兴趣的3*3位置雅可比矩阵

    const KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk_solver;                                         // 关节位置->足端位置
    KDL::ChainJntToJacSolver jacobain_solver;                                          // 求解雅可比矩阵
    KDL::ChainJntToJacDotSolver jdot_solver;                                           // 求解dJdq
    KDL::ChainIkSolverVel_pinv vel_solver;                                             //
    KDL::ChainIkSolverPos_LMA ik_pos_solver;                                           // 计算期望关节位置
    KDL::ChainDynParam dynamin_solver;                                                 // 关节运动状态->关节力矩

    // 计算数据缓存区
    KDL::JntSpaceInertiaMatrix M;
    KDL::JntArray C;
    KDL::JntArray G;
    KDL::Jacobian temp_jacobain;

    KDL::JntArray last_exp_joint_pos;

    KDL::JntArray _temp_joint3_array;
    KDL::JntArray _temp2_joint3_array;

    KDL::JntArrayVel _temp_joint3_vel_array;
    KDL::Twist _temp_jdot_qd;

    double wheel_radius{0.065};
};




class Cross_Step {
private:
    typedef struct {
        double a;
        double b;
        double c;
        double d;
        double e;
        double f;
    } QuinticLineParam_t;

    typedef struct {
        QuinticLineParam_t x;
        QuinticLineParam_t y;
        QuinticLineParam_t z;
        QuinticLineParam_t l1_z;
        QuinticLineParam_t l2_z;
        double time;
    } StepTrajectory_t;
    StepTrajectory_t traj;
    double T;
    static void set_quintic(QuinticLineParam_t& seg, double p0, double v0, double a0, double pT, double vT, double aT, double T) {
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T3 * T;
        double T5 = T4 * T;

        seg.a = p0;
        seg.b = v0;
        seg.c = 0.5 * a0;

        seg.d = (10 * (pT - p0) - (6 * v0 + 4 * vT) * T - (1.5 * a0 - 0.5 * aT) * T2) / T3;
        seg.e = (-15 * (pT - p0) + (8 * v0 + 7 * vT) * T + (1.5 * a0 - aT) * T2) / T4;
        seg.f = (6 * (pT - p0) - (3 * v0 + 3 * vT) * T - (0.5 * a0 - 0.5 * aT) * T2) / T5;
    }

    static inline double get_quintic_value(const QuinticLineParam_t& line, double t) {
        return line.a + line.b * t + line.c * t * t + line.d * t * t * t + line.e * t * t * t * t + line.f * t * t * t * t * t;
    }

    static inline double get_quintic_dt(const QuinticLineParam_t& line, double t) {
        return line.b + 2.0 * line.c * t + 3.0 * line.d * t * t + 4.0 * line.e * t * t * t + 5.0 * line.f * t * t * t * t;
    }

    static inline double get_quintic_dtdt(const QuinticLineParam_t& line, double t) {
        return 2.0 * line.c + 6.0 * line.d * t + 12.0 * line.e * t * t + 20.0 * line.f * t * t * t;
    }

public:
    bool flight_trajectory_is_available = false;
    bool support_trajectory_is_available = false;
    inline void update_support_trajectory(const Eigen::Vector3d& cur_pos, const Eigen::Vector3d final_pos, double time) {
        traj.time = time;
        flight_trajectory_is_available = false;
        support_trajectory_is_available = true;

        double v0 = 0.0;
        double vT = 0.0;
        double a0 = 0.0;
        double aT = 0.0;

        set_quintic(traj.x, cur_pos[0], v0, a0, final_pos[0], vT, aT, time);
        set_quintic(traj.y, cur_pos[1], v0, a0, final_pos[1], vT, aT, time);
        set_quintic(traj.z, cur_pos[2], v0, a0, final_pos[2], vT, aT, time);
    }
    inline void update_flight_trajectory(
        const Eigen::Vector3d& cur_pos, const Eigen::Vector3d& cur_vel, const Eigen::Vector3d& exp_pos, const Eigen::Vector3d& exp_vel, const double time, const double step_height) {

        traj.time = time;
        flight_trajectory_is_available = true;
        support_trajectory_is_available = false;

        set_quintic(
            traj.x, cur_pos[0], cur_vel[0], 0.0, // 起点
            exp_pos[0], -exp_vel[0], 0.0, time);               // 终点
        // y方向轨迹
        set_quintic(traj.y, cur_pos[1], cur_vel[1], 0.0, exp_pos[1], -exp_vel[1], 0.0, time);
        // z方向分为两段：抬腿 -> 落腿
        // 第一段：从当前z抬到最高点
        set_quintic(traj.l1_z, cur_pos[2], cur_vel[2], 0.0, step_height, 0.0, 0.0, time * 0.5);

        // 第二段：从最高点落到地面
        set_quintic(traj.l2_z, step_height, 0.0, 0.0, exp_pos[2], 0.0, 0.0, time * 0.5);
    }
    inline std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> get_target(double time, bool& success) {
        Eigen::Vector3d pos, vel, acc;

        if (time >= traj.time) {
            time    = traj.time;
            success = false;
        } else {
            success = true;
        }

        pos[0] = get_quintic_value(traj.x, time);
        vel[0] = get_quintic_dt(traj.x, time);
        acc[0] = get_quintic_dtdt(traj.x, time);

        pos[1] = get_quintic_value(traj.y, time);
        vel[1] = get_quintic_dt(traj.y, time);
        acc[1] = get_quintic_dtdt(traj.y, time);

        if (flight_trajectory_is_available) {
            const double half_time = traj.time * 0.5;
            if (time <= half_time) {
                pos[2] = get_quintic_value(traj.l1_z, time);
                vel[2] = get_quintic_dt(traj.l1_z, time);
                acc[2] = get_quintic_dtdt(traj.l1_z, time);
            } else {
                const double t2 = time - half_time;
                pos[2] = get_quintic_value(traj.l2_z, t2);
                vel[2] = get_quintic_dt(traj.l2_z, t2);
                acc[2] = get_quintic_dtdt(traj.l2_z, t2);
            }
        } else if (support_trajectory_is_available) {
            pos[2] = get_quintic_value(traj.z, time);
            vel[2] = get_quintic_dt(traj.z, time);
            acc[2] = get_quintic_dtdt(traj.z, time);
        } else {
            pos[2] = 0.0;
            vel[2] = 0.0;
            acc[2] = 0.0;
        }

        return {pos, vel, acc};
    }
};

struct DiagonalWalkTargets {
    Eigen::Vector3d lf_pos{0.0, 0.0, 0.0}, rf_pos{0.0, 0.0, 0.0}, lb_pos{0.0, 0.0, 0.0}, rb_pos{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_vel{0.0, 0.0, 0.0}, rf_vel{0.0, 0.0, 0.0}, lb_vel{0.0, 0.0, 0.0}, rb_vel{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_acc{0.0, 0.0, 0.0}, rf_acc{0.0, 0.0, 0.0}, lb_acc{0.0, 0.0, 0.0}, rb_acc{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_force{0.0, 0.0, 0.0}, rf_force{0.0, 0.0, 0.0}, lb_force{0.0, 0.0, 0.0}, rb_force{0.0, 0.0, 0.0};
};

class DiagonalWalkController {
public:
    void configure(
        double step_time, double step_height, double step_support_rate, double body_vx, double body_vy, double yaw_rate, double duration,
        double stance_z_offset = 0.0);
    void reset();
    void start(const std::shared_ptr<Robot_t>& robot);
    bool update(
        const std::shared_ptr<Robot_t>& robot, double mass, const Eigen::Vector2d& mass_center_pos, double now, DiagonalWalkTargets& targets);

private:
    static Eigen::Vector2d calc_leg_planar_vel(const Eigen::Vector3d& body_vel, const Eigen::Vector3d& omega, const Eigen::Vector3d& leg_offset);
    static Eigen::Vector3d make_support_target(const Eigen::Vector2d& exp_vel, double time, double stance_z_offset);
    static Eigen::Vector3d make_flight_target(const Eigen::Vector2d& exp_vel, double time, double stance_z_offset);
    void update_leg_velocity_targets(const std::shared_ptr<Robot_t>& robot);
    void solve_support_forces(const std::shared_ptr<Robot_t>& robot, double mass, const Eigen::Vector2d& mass_center_pos, DiagonalWalkTargets& targets) const;

    double step_time_{0.6};
    double step_height_{0.08};
    double step_support_rate_{0.6};
    double body_vx_{0.18};
    double body_vy_{0.0};
    double yaw_rate_{0.0};
    double duration_{3.0};
    double stance_z_offset_{0.0};

    double main_phase_start_{0.0};
    double slave_phase_start_{0.0};
    double slave_phase_stop_{0.0};

    bool initialized_{false};
    bool stop_requested_{false};
    bool step1_support_updated_{false};
    bool step2_support_updated_{false};
    bool step1_flight_updated_{false};
    bool step2_flight_updated_{false};

    Eigen::Vector2d lf_exp_vel_{0.0, 0.0}, rf_exp_vel_{0.0, 0.0}, lb_exp_vel_{0.0, 0.0}, rb_exp_vel_{0.0, 0.0};
    Cross_Step lf_step_, rf_step_, lb_step_, rb_step_;
};
