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

            set_quintic(
                traj.x, cur_pos[0], cur_vel[0], 0.0, // 起点
                exp_pos[0], -exp_vel[0], 0.0, time);               // 终点
            // y方向轨迹
            set_quintic(traj.y, cur_pos[1], cur_vel[1], 0.0, exp_pos[1], -exp_vel[1], 0.0, time);
            // z方向分为两段：抬腿 -> 落腿
            // 第一段：从当前z抬到最高点
            set_quintic(traj.l1_z, cur_pos[2], cur_vel[2], 0.0, step_height, 0.0, 0.0, time * 0.5f);

            // 第二段：从最高点落到地面
            set_quintic(traj.l2_z, step_height, 0.0, 0.0, exp_pos[2], 0.0, 0.0, time * 0.5f);
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

        pos[2] = get_quintic_value(traj.z, time);
        vel[2] = get_quintic_dt(traj.z, time);
        acc[2] = get_quintic_dtdt(traj.z, time);

        return {pos, vel, acc};
    }
};

class Cross_WallState  {

public:

    Cross_WallState(const std::string& urdf_file_path);
    void enter();
    RobotTarget update();
    std::tuple<Eigen::Vector3d, double> get_robot_mass_info(
    const Eigen::Vector3d& lf_joint_pos, const Eigen::Vector3d& rf_joint_pos, const Eigen::Vector3d& lb_joint_pos, const Eigen::Vector3d& rb_joint_pos);

    std::shared_ptr<Robot_t> robot;
    int cross_wall_stage{-1};
    std::chrono::steady_clock::time_point cross_wall_stage_time;

    Eigen::Vector3d wall_lf_foot_pos{0, 0, 0}, wall_rf_foot_pos{0, 0, 0}, wall_lb_foot_pos{0, 0, 0}, wall_rb_foot_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_pos{0, 0, 0}, rf_foot_exp_pos{0, 0, 0}, lb_foot_exp_pos{0, 0, 0}, rb_foot_exp_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_force{0, 0, 0}, rf_foot_exp_force{0, 0, 0}, lb_foot_exp_force{0, 0, 0}, rb_foot_exp_force{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_vel{0, 0, 0}, rf_foot_exp_vel{0, 0, 0}, lb_foot_exp_vel{0, 0, 0}, rb_foot_exp_vel{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_acc{0, 0, 0}, rf_foot_exp_acc{0, 0, 0}, lb_foot_exp_acc{0, 0, 0}, rb_foot_exp_acc{0, 0, 0};
    Eigen::Vector3d lf_forward_torque{0, 0, 0}, rf_forward_torque{0, 0, 0}, lb_forward_torque{0, 0, 0}, rb_forward_torque{0, 0, 0};

    // 使用关节角度
    Eigen::Vector3d lf_joint_exp_pos_{0, 0, 0}, rf_joint_exp_pos_{0, 0, 0}, lb_joint_exp_pos_{0, 0, 0}, rb_joint_exp_pos_{0, 0, 0};
    Eigen::Vector3d lf_joint_omega{0, 0, 0}, rf_joint_omega{0, 0, 0}, lb_joint_omega{0, 0, 0}, rb_joint_omega{0, 0, 0};
    Eigen::Vector3d lf_joint_torque{0, 0, 0}, rf_joint_torque{0, 0, 0}, lb_joint_torque{0, 0, 0}, rb_joint_torque{0, 0, 0};

    double lf_wheel_vel{0.0}, rf_wheel_vel{0.0}, lb_wheel_vel{0.0}, rb_wheel_vel{0.0};
    double lf_wheel_force{0.0}, rf_wheel_force{0.0}, lb_wheel_force{0.0}, rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.3; // 建议 0.3~0.6

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;

    double time_s{1.0};
    bool change_flag{true};
    bool allow_vel{true};

    float k_F{1.0f};

    // 力变量
    Eigen::Vector2d mass_center_pos;
    double mass;
    bool RL_walk_flag{false};
    bool Cross_wall_over{false};

    Cross_Step lf_step, rf_step, lb_step, rb_step;
};