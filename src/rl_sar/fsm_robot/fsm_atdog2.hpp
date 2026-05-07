/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATDOG2_FSM_HPP
#define ATDOG2_FSM_HPP

#include "../library/core/fsm/fsm.hpp"
#include "../library/core/rl_sdk/rl_sdk.hpp"
#include <eigen3/Eigen/Dense>
#include <chrono>
 #include <eigen3/Eigen/Core>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>   // ← 你缺的就是它
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <vector>
#include </opt/ros/humble/include/kdl_parser/kdl_parser/kdl_parser.hpp>

namespace atdog2_fsm
{

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.Get<int>("num_of_dofs"); ++i)
        {
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 8;
            fsm_command->motor_command.tau[i] = 0;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}


    float percent_pre_getup = 0.0f;
    float percent_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, -0.8, 0.0,
        0.00, 0.8, 0.0,
        0.00, -0.8, 0.0,
        0.00, 0.8, 0.0,
        0.00, 0.00, 0.00, 0.00
    };
    bool stand_from_passive = true;

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        percent_getup = 0.0f;
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStatePassive")
        {
            stand_from_passive = true;
        }
        else
        {
            stand_from_passive = false;
        }
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        std::cout<<"当前关节角:"<<rl.now_state.motor_state.q<<std::endl;
    }

    void Run() override
    {
        if(stand_from_passive)
        {

            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 1.0f, "Pre Getting up", true)) return;
            if (Interpolate(percent_getup, pre_running_pos, rl.params.Get<std::vector<float>>("default_dof_pos"), 2.0f, "Getting up", true)) return;
        }
        else
        {
            if (Interpolate(percent_getup, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 1.0f, "Getting up", true)) return;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        if (percent_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRLLocomotion";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    float percent_getdown = 0.0f;

    void Enter() override
    {
        percent_getdown = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        Interpolate(percent_getdown, rl.now_state.motor_state.q, rl.start_state.motor_state.q, 2.0f, "Getting down", true);
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || percent_getdown >= 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

/*
*
*
* 
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*/


class RLFSMStateCrosswall : public RLFSMState
{
    
    using Vector3D = Eigen::Vector3d;
    using Vector2D = Eigen::Vector2d;

public:


        bool use_limit_lf = false;
        bool use_limit_rf = false;
        bool use_limit_lb = false;
        bool use_limit_rb = false;
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
        double time;
    } StepTrajectory_t;

    class Cross_Step {
    private:
        StepTrajectory_t traj;
        double T;
        static void set_quintic(QuinticLineParam_t& seg,
                            double p0, double v0, double a0,
                            double pT, double vT, double aT,
                            double T)
        {
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

        static inline double get_quintic_value(const QuinticLineParam_t& line, double t)
        {
            return line.a
                + line.b * t
                + line.c * t * t
                + line.d * t * t * t
                + line.e * t * t * t * t
                + line.f * t * t * t * t * t;
        }

        static inline double get_quintic_dt(const QuinticLineParam_t& line, double t)
        {
            return line.b
                + 2.0 * line.c * t
                + 3.0 * line.d * t * t
                + 4.0 * line.e * t * t * t
                + 5.0 * line.f * t * t * t * t;
        }

        static inline double get_quintic_dtdt(const QuinticLineParam_t& line, double t)
        {
            return 2.0 * line.c
                + 6.0 * line.d * t
                + 12.0 * line.e * t * t
                + 20.0 * line.f * t * t * t;
        }

    public:

        inline void update_support_trajectory(const Eigen::Vector3d & cur_pos, const Eigen::Vector3d  final_pos, double time)
        {
            traj.time = time;
            T = time;

            for (int i = 0; i < 3; i++)
            {
                double p0 = cur_pos[i];
                double pT = final_pos[i];

                // ⭐ Quintic：直接保证平滑
                double v0 = 0.0;
                double vT = 0.0;
                double a0 = 0.0;
                double aT = 0.0;

                if (i == 0)
                    set_quintic(traj.x, p0, v0, a0, pT, vT, aT, time);
                else if (i == 1)
                    set_quintic(traj.y, p0, v0, a0, pT, vT, aT, time);
                else
                    set_quintic(traj.z, p0, v0, a0, pT, vT, aT, time);
            }

        }
        inline std::tuple<Eigen::Vector3d , Eigen::Vector3d , Eigen::Vector3d > get_target(double time,bool &success)
        {
            Eigen::Vector3d  pos, vel, acc;

            if (time >= T)
            {
                time = T;
                success = false;
            }
            else
            {
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


    class LegCalc {
private:
       
        KDL::Chain chain;
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
        
        Eigen::Vector3d joint_acc(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_vel, Eigen::Vector3d foot_acc){
    _temp_joint3_array.data          = joint_rad;
    _temp2_joint3_array.data         = joint_vel;
    _temp_joint3_vel_array.q.data    = joint_rad;
    _temp_joint3_vel_array.qdot.data = joint_vel;

    // 计算雅可比矩阵J
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    jdot_solver.JntToJacDot(_temp_joint3_vel_array, _temp_jdot_qd);

    Eigen::Matrix3d Jac = get_3x3_jacobian_(temp_jacobain);
    Vector3D jdot_dq_eigen;
    for (int i = 0; i < 3; i++) {
        jdot_dq_eigen[i] = _temp_jdot_qd(i);
    }
    return Jac.completeOrthogonalDecomposition().solve(foot_acc - jdot_dq_eigen);
}

        Eigen::Matrix<double, 3, 3> get_3x3_jacobian_(const KDL::Jacobian& full_jacobian)// 从KDL库中求出我们感兴趣的3*3位置雅可比矩阵
        {
            Eigen::Matrix<double, 3, 3> jacobian_3x3;
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    jacobian_3x3(i, j) = full_jacobian(i, j);
                }
            }
            return jacobian_3x3;
        }




    public:

        Eigen::Vector3d pos_offset;                                                                       // 足端位置到机器人中心的偏移
        double kp[3], kd[3];
        double wheel_kd;
        LegCalc(
            KDL::Chain& chain, const std::vector<double>& kp_list = {3.0, 2.8, 2.8},
            const std::vector<double>& kd_list = {0.17, 0.14, 0.11}, double wheel_kd_param = 0.5)
            : chain(chain)
            , fk_solver(chain)
            , jacobain_solver(chain)
            , jdot_solver(chain)
            , vel_solver(chain)
            , ik_pos_solver(chain, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10)
            , dynamin_solver(chain, KDL::Vector(0, 0, -9.81)) {
            _temp_joint3_array.resize(3); // 提前resize需要用到的KDL::JntArray防止运行时频繁申请/释放内存
            _temp2_joint3_array.resize(3);
            last_exp_joint_pos.resize(3);
            temp_jacobain.resize(3);
            _temp_joint3_vel_array.resize(3);

            C.resize(3);
            G.resize(3);
            M.resize(3);

            last_exp_joint_pos(0) = 0.0;
            last_exp_joint_pos(1) = 0.0;
            last_exp_joint_pos(2) = 0.0;

            // set_joint_pd(0,3.0,0.17);   //设置默认参数
            // set_joint_pd(1,2.8,0.14);
            // set_joint_pd(2,2.8,0.11);

            // // set_joint_pd(0,50.0,3.0);   //设置默认参数
            // // set_joint_pd(1,50.0,3.0);
            // // set_joint_pd(2,50.0,3.0);

            // set_joint_pd(3,0.0,0.5);

            if (kp_list.size() != 3 || kd_list.size() != 3) {
                throw std::runtime_error("PD param size must be 3");
            }


            for (int i = 0; i < 3; i++) {
                this->kp[i] = kp_list[i];
                this->kd[i] = kd_list[i];
            }

            this->wheel_kd = wheel_kd_param;
        }
        ~LegCalc();

        void set_init_joint_pos(const Eigen::Vector3d init_joint_pos)
        {
            last_exp_joint_pos(0)=init_joint_pos[0];
            last_exp_joint_pos(1)=init_joint_pos[1];
            last_exp_joint_pos(2)=init_joint_pos[2];
        }
        

        // int joint_pos(KDL::JntArray &joint_rad, KDL::Vector &foot_pos,KDL::JntArray &result);
        Eigen::Vector3d
            joint_pos(const Eigen::Vector3d& foot_pos, int* result)
             {
    KDL::Frame frame;
    Eigen::Vector3d temp = foot_pos + pos_offset;
    frame.p.x(temp[0]);
    frame.p.y(temp[1]);
    frame.p.z(temp[2]);
    frame.M = KDL::Rotation::Identity();

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint3_array);
    if (*result == 0)                                                                      // 缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos = _temp_joint3_array;
    return {_temp_joint3_array(0), _temp_joint3_array(1), _temp_joint3_array(2)};
} // 稍后需要在线安装IK求解器（手推的解析求解器或者数值迭代器）


        Eigen::Vector3d foot_force(
            const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_torque,
            const Eigen::Vector3d& forward_torque = Eigen::Vector3d::Zero()){
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    auto jacobian = get_3x3_jacobian_(temp_jacobain);

    return jacobian.transpose().inverse() * (joint_torque - forward_torque);
}




        Eigen::Vector3d foot_pos(const Eigen::Vector3d& joint_rad) {
    KDL::Frame frame;
    _temp_joint3_array(0) = joint_rad[0]; // 避免运行时动态分配内存，提高效率
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    int fk_result = fk_solver.JntToCart(_temp_joint3_array, frame);

                                          // 添加调试：检查 FK 计算结果
    std::cout << "[FK DEBUG] Joint angles: [" << joint_rad[0] << ", " << joint_rad[1] << ", " << joint_rad[2] << "]" << std::endl;
    std::cout << "[FK DEBUG] FK result code: " << fk_result << std::endl;
    std::cout << "[FK DEBUG] Frame position: [" << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "]" << std::endl;
    std::cout << "[FK DEBUG] pos_offset: [" << pos_offset[0] << ", " << pos_offset[1] << ", " << pos_offset[2] << "]" << std::endl;

    Eigen::Vector3d temp;
    temp[0] = frame.p.x();
    temp[1] = frame.p.y();
    temp[2] = frame.p.z();

    return temp - pos_offset; // temp是在机器人坐标系下的足端位置，要转换成支撑相中型点的坐标输出
}

        void set_joint_pd(int index, double kp, double kd);

        void get_joint_pd(int index, double& kp, double& kd);

        // robot_interfaces::msg::LegTarget signal_leg_calc(
        //     const Vector3D& exp_cart_pos, const Vector3D& exp_cart_vel, const Vector3D& exp_cart_acc, const Vector3D& exp_cart_force,
        //     Vector3D* torque, const double wheel_vel = 0.0, const double wheel_force = 0.0);

        // robot_interfaces::msg::LegTarget signal_leg_torque_calc(
        //     const Vector3D& cur_joint_pos, const Vector3D& exp_foot_force, const Vector3D& foot_vel, const Vector3D& foot_acc,
        //     double wheel_force = 0.0);




    };

    class Robot_t {
    public:

        static constexpr double WHEEL_RADIUS = 0.065;
            std::vector<double> kp, kd;
            double wheel_kd;

        std::vector<std::string> joint_names = {"lf_joint1", "lf_joint2", "lf_joint3", "lf_joint4", "rf_joint1", "rf_joint2",
                                                "rf_joint3", "rf_joint4", "lb_joint1", "lb_joint2", "lb_joint3", "lb_joint4",
                                                "rb_joint1", "rb_joint2", "rb_joint3", "rb_joint4"};
                                                
        // 解算部分
        // rclcpp::SyncParametersClient::SharedPtr robot_description_param_;
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

                Eigen::Vector3d lf_joint_pos, lf_joint_vel, 
                                lf_joint_torque, lf_forward_torque;
        
                Eigen::Vector3d rf_joint_pos, rf_joint_vel, 
                                rf_joint_torque, rf_forward_torque;
        
                Eigen::Vector3d lb_joint_pos, lb_joint_vel, 
                                lb_joint_torque, lb_forward_torque;
        
                Eigen::Vector3d rb_joint_pos, rb_joint_vel, 
                                rb_joint_torque, rb_forward_torque;

        double lf_wheel_omega{0.0},rf_wheel_omega{0.0},lb_wheel_omega{0.0},rb_wheel_omega{0.0};
        double lf_wheel_torque{0.0},rf_wheel_torque{0.0},lb_wheel_torque{0.0},rb_wheel_torque{0.0};


        // 共享参数
        Eigen::Vector3d lf_base_offset;
        Eigen::Vector3d rf_base_offset;
        Eigen::Vector3d lb_base_offset;
        Eigen::Vector3d rb_base_offset;

        double robot_lf_grivate{0.0};
        double robot_rf_grivate{0.0};
        double robot_lb_grivate{0.0};
        double robot_rb_grivate{0.0};

        Robot_t() {
            robot_lf_grivate = 25.86;
            robot_rf_grivate = 25.33;
            robot_lb_grivate = 25.86;
            robot_rb_grivate = 25.23;
            lf_base_offset   = Eigen::Vector3d(0.18433, 0.058364, 0.045534);
            rf_base_offset   = Eigen::Vector3d(0.18643, -0.06136, 0.03776);
            lb_base_offset   = Eigen::Vector3d(-0.20677, 0.058639, 0.03776);
            rb_base_offset   = Eigen::Vector3d(-0.20677, -0.06136, 0.037786);

            /*待改*/

            // robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");
            // auto params              = robot_description_param_->get_parameters({"robot_description"});
            // urdf_xml                 = params[0].as_string();
            // if (urdf_xml.empty()) {
            //     RCLCPP_ERROR(node_->get_logger(), "无法读取URDF文件，不能进行动力学计算");
            //     return;
            // }

            // 将四个轮子关节从continuous类型改为fixed类型，以便KDL能够求解
            std::vector<std::string> wheel_joints = {"lf_joint4", "rf_joint4", "lb_joint4", "rb_joint4"};
            for (const auto& joint_name : wheel_joints) {
                std::string search_pattern  = "<joint\n        name=\"" + joint_name + "\"\n        type=\"continuous\">";
                std::string replace_pattern = "<joint\n        name=\"" + joint_name + "\"\n        type=\"fixed\">";
                size_t pos                  = urdf_xml.find(search_pattern);
                if (pos != std::string::npos) {
                    urdf_xml.replace(pos, search_pattern.length(), replace_pattern);
                }
            }

            kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
            tree.getChain("body_link", "lf_link4", lf_leg_chain);
            tree.getChain("body_link", "rf_link4", rf_leg_chain);
            tree.getChain("body_link", "lb_link4", lb_leg_chain);
            tree.getChain("body_link", "rb_link4", rb_leg_chain);

            // 初始化狗腿解算器，定义足端中性点位置
            lf_leg_calc             = std::make_shared<LegCalc>(lf_leg_chain, kp, kd, wheel_kd);
            rf_leg_calc             = std::make_shared<LegCalc>(rf_leg_chain, kp, kd, wheel_kd);
            lb_leg_calc             = std::make_shared<LegCalc>(lb_leg_chain, kp, kd, wheel_kd);
            rb_leg_calc             = std::make_shared<LegCalc>(rb_leg_chain, kp, kd, wheel_kd);
            lf_leg_calc->pos_offset = lf_base_offset;
            rf_leg_calc->pos_offset = rf_base_offset;
            lb_leg_calc->pos_offset = lb_base_offset;
            rb_leg_calc->pos_offset = rb_base_offset;
        };
        ~Robot_t();

    };
    std::shared_ptr<Robot_t> robot_t;
    int cross_wall_stage{-1};

    Eigen::Vector3d  wall_lf_foot_pos, wall_rf_foot_pos, wall_lb_foot_pos, wall_rb_foot_pos;
    Eigen::Vector3d  lf_foot_exp_pos, rf_foot_exp_pos, lb_foot_exp_pos, rb_foot_exp_pos;
    Eigen::Vector3d  lf_foot_exp_force, rf_foot_exp_force, lb_foot_exp_force, rb_foot_exp_force;
    Eigen::Vector3d  lf_foot_exp_vel, rf_foot_exp_vel, lb_foot_exp_vel, rb_foot_exp_vel;
    Eigen::Vector3d  lf_foot_exp_acc, rf_foot_exp_acc, lb_foot_exp_acc, rb_foot_exp_acc;
    Eigen::Vector3d  lf_forward_torque, rf_forward_torque, lb_forward_torque, rb_forward_torque;

    //使用关节角度
    Eigen::Vector3d  lf_joint_exp_pos_,rf_joint_exp_pos_,
             lb_joint_exp_pos_,rb_joint_exp_pos_;
    Eigen::Vector3d  lf_joint_exp_omega, rf_joint_exp_omega,
             lb_joint_exp_omega,rb_joint_exp_omega;
    Eigen::Vector3d  lf_joint_exp_torque,rf_joint_exp_torque,
             lb_joint_exp_torque,rb_joint_exp_torque;

    double lf_wheel_vel{0.0},rf_wheel_vel{0.0},lb_wheel_vel{0.0},rb_wheel_vel{0.0};
    double lf_wheel_force{0.0},rf_wheel_force{0.0},lb_wheel_force{0.0},rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.3;   // 建议 0.3~0.6

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;
    
    double time_s{1.0};
    bool change_flag{true};
    bool allow_vel{true};
    std::chrono::steady_clock::time_point cross_wall_stage_time;
    float k_F{1.0f};

    //力变量
    Eigen::Vector2d mass_center_pos;


    Cross_Step lf_step, rf_step, lb_step, rb_step;





    RLFSMStateCrosswall(RL *rl) : RLFSMState(*rl, "RLFSMStateCrosswall") {

       mass_center_pos = Eigen::Vector2d(-0.012165, 0.00021263);
       robot_t = std::make_shared<Robot_t>();
       
    }

    
    

    void Enter() override
    {
        cross_wall_stage=-1;
        //change_flag = true;
        cross_wall_stage_time = std::chrono::steady_clock::now();
        rl.now_state = *fsm_state;
    }

    void Run() override
    {

        robot_t->lf_joint_pos = 
        lf_wheel_force = 0.0f;
        static int last_stage = -1;
        
        if (cross_wall_stage == -1)
        {
            last_stage = -1;
            double time = std::chrono::duration<double>(std::chrono::steady_clock::now() - cross_wall_stage_time).count();
            if(time > 0.5)
                cross_wall_stage = 0;
        }

        static int cnt = 0;
        cnt++;
        if(cnt>=250)
        {
            cnt = 0;
            std::cerr << "\033[31mchange_flag = " << change_flag << ", cross_wall_stage = " << cross_wall_stage << "\033[0m" << std::endl;
        }

        /*****要改******** */
        lf_foot_exp_force=Vector3D(0.0, 0.0, -robot_t->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0, 0.0, -robot_t->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0, 0.0, -robot_t->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0, 0.0, -robot_t->robot_rb_grivate);
        /*****要改******** */

        if(cross_wall_stage == 0  && change_flag == true){
                use_limit_lf = false;
                use_limit_lb = false;
                use_limit_rb = false;
                use_limit_rf = false;
                
             if(allow_vel == true)
             {
                lf_wheel_vel= 0.3;
                rf_wheel_vel=-0.3;
                lb_wheel_vel= 0.3;
                rb_wheel_vel=-0.3;

                double wheel_F = k_F * ((lf_wheel_vel / Robot_t::WHEEL_RADIUS) - (robot_t->lf_wheel_omega + robot_t->lb_wheel_omega - robot_t->rf_wheel_omega -robot_t->rb_wheel_omega) / 4.0f);
                lf_wheel_force = wheel_F;
                lb_wheel_force = wheel_F;
                rf_wheel_force = -wheel_F;
                rb_wheel_force = -wheel_F;

                allow_vel = false;
             }
            auto lf_cart_force = robot_t->lf_leg_calc->foot_force(robot_t->lf_joint_pos, robot_t->lf_joint_torque, robot_t->lf_forward_torque);
            auto rf_cart_force = robot_t->rf_leg_calc->foot_force(robot_t->rf_joint_pos, robot_t->rf_joint_torque, robot_t->rf_forward_torque);
            static int cnt = 0;
            cnt++;
            if(cnt>=25 && (!stopping))
            {
            RCLCPP_ERROR(robot_t->node_->get_logger(),
                "\033[31m这是碰撞前的数据:lf_cart_force[0] = %f, rf_cart_force[0] = %f\033[0m",
                        lf_cart_force[0], rf_cart_force[0]);
            }
            if ((lf_cart_force[0] > 10.0 || rf_cart_force[0] > 10.0) && (!stopping)){

                if(!stopping) stop_t = 0.0;
                stopping = true;
                

                // 记录初值
                lf_vel_start = lf_wheel_vel;
                rf_vel_start = rf_wheel_vel;
                lb_vel_start = lb_wheel_vel;
                rb_vel_start = rb_wheel_vel;

                lf_force_start = lf_wheel_force;
                rf_force_start = rf_wheel_force;
                lb_force_start = lb_wheel_force;
                rb_force_start = rb_wheel_force;


                // wall_lf_foot_pos=robot_t->lf_leg_calc->foot_pos(robot_t->lf_joint_pos);
                // wall_rf_foot_pos=robot_t->rf_leg_calc->foot_pos(robot_t->rf_joint_pos);
                // wall_lb_foot_pos=robot_t->lb_leg_calc->foot_pos(robot_t->lb_joint_pos);
                // wall_rb_foot_pos=robot_t->rb_leg_calc->foot_pos(robot_t->rb_joint_pos);

                // lf_leg_step.update_support_trajectory(wall_lf_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // rf_leg_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                // rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.08,0.0),2.0);
                
                // //change_flag=false;
                // cross_wall_stage=1;
            }
            // ================= 平滑减速 =================
            if (stopping)
            {
                static int cnt = 0;
                cnt++;
                if(cnt>=25)
                {
                RCLCPP_ERROR(robot_t->node_->get_logger(),
                    "\033[31m数据:lf_cart_force[0] = %f, rf_cart_force[0] = %f\033[0m",
                            lf_cart_force[0], rf_cart_force[0]);
                }
                stop_t += 0.004;

                double tau = stop_t / stop_T;
                if (tau > 1.0) tau = 1.0;


                double s = 10*pow(tau,3) - 15*pow(tau,4) + 6*pow(tau,5);

                // 速度快速降
                lf_wheel_vel = lf_vel_start * (1 - s);
                rf_wheel_vel = rf_vel_start * (1 - s);
                lb_wheel_vel = lb_vel_start * (1 - s);
                rb_wheel_vel = rb_vel_start * (1 - s);

                // ⚠️ 力慢一点降（关键！防翻）
                double s_force = pow(s, 2.0);

                lf_wheel_force = lf_force_start * (1 - s_force);
                rf_wheel_force = rf_force_start * (1 - s_force);
                lb_wheel_force = lb_force_start * (1 - s_force);
                rb_wheel_force = rb_force_start * (1 - s_force);
                // static int cnt = 0;
                // cnt++;
                // if(cnt>=25)
                // {
                // RCLCPP_ERROR(robot_t->node_->get_logger(),
                //     "\033[31mtau = %.2f,\n s = %.2f\n, s_force = %.2f\033[0m",
                //             tau, s,s_force);
                // }

                // ================= 停稳后再切阶段 =================
                if (tau >= 1.0)
                {
                    stopping = false;

                    lf_wheel_vel = 0.0;
                    rf_wheel_vel = 0.0;
                    lb_wheel_vel = 0.0;
                    rb_wheel_vel = 0.0;

                    // ⚠️ 力不要全清（可以留一点贴墙）
                    lf_wheel_force = 0.0;
                    rf_wheel_force = 0.0;
                    lb_wheel_force = 0.0;
                    rb_wheel_force = 0.0;

                    // ======= 这里才记录足端位置 =======
                    wall_lf_foot_pos = robot_t->lf_leg_calc->foot_pos(robot_t->lf_joint_pos);
                    wall_rf_foot_pos = robot_t->rf_leg_calc->foot_pos(robot_t->rf_joint_pos);
                    wall_lb_foot_pos = robot_t->lb_leg_calc->foot_pos(robot_t->lb_joint_pos);
                    wall_rb_foot_pos = robot_t->rb_leg_calc->foot_pos(robot_t->rb_joint_pos);

                    lf_step.update_support_trajectory(wall_lf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
                    rf_step.update_support_trajectory(wall_rf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
                    lb_step.update_support_trajectory(wall_lb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
                    rb_step.update_support_trajectory(wall_rb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
                    //change_flag=false;
                    cross_wall_stage = 1;
                }
            }

    }  //0:碰撞检测后,狗身向右倾斜，准备迈左后腿
      
        else if (cross_wall_stage == 1 && change_flag == true){        
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
                    lf_wheel_vel = 0.0;
                    rf_wheel_vel = 0.0;
                    lb_wheel_vel = 0.0;
                    rb_wheel_vel = 0.0;

                    // ⚠️ 力不要全清（可以留一点贴墙）
                    lf_wheel_force = 0.0;
                    rf_wheel_force = 0.0;
                    lb_wheel_force = 0.0;
                    rb_wheel_force = 0.0;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            auto lf_pos=(lf_foot_exp_pos+robot_t->lf_leg_calc->pos_offset).head(2);   //在简化的二维平面模型中，提供支撑力的位置
            auto rf_pos=(rf_foot_exp_pos+robot_t->rf_leg_calc->pos_offset).head(2);
            auto lb_pos=(lb_foot_exp_pos+robot_t->lb_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot_t->rb_leg_calc->pos_offset).head(2);
            
            // 使用Eigen求解每条腿的支撑力
            // 构建方程组: A * f = b
            // 约束条件:
            // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
            // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
            // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
            Eigen::Matrix<double, 3, 4> A;
            Eigen::Vector3d b;
            
            // 第一行: 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // lb
            A(0, 3) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 第二行: 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = lb_pos.x() - mass_center_pos.x();
            A(1, 3) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 第三行: 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = lb_pos.y() - mass_center_pos.y();
            A(2, 3) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 使用最小二乘法求解超定方程组
            Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;
            
            // 将求解的垂直力存入各腿的期望力向量（z分量）
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));
            
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(wall_lf_foot_pos,wall_lf_foot_pos,1.0);
                rf_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,1.0);
                rb_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,1.0);
                lb_leg_step.update_flight_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0), Vector3D(0.12,0.08,0.0), Eigen::Vector2d(0.0,0.0), 1.0,0.06);
                //change_flag=false;
                cross_wall_stage=2;     
            }
        }//1:其它保持不变,迈左后腿
        else if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
            auto lf_pos=(lf_foot_exp_pos+robot_t->lf_leg_calc->pos_offset).head(2);
            auto rf_pos=(rf_foot_exp_pos+robot_t->rf_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot_t->rb_leg_calc->pos_offset).head(2);
            
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            
            // 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 求解3x3方程组
            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
            
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot_t->lf_joint_pos,Vector3D(1.44,0.822,-0.165),2.0);
                
                //change_flag=false;
                cross_wall_stage=3;    
            }
        }
        else if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success); 
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot_t->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot_t->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot_t->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {
               
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(Vector3D(1.44,0.822,-0.165),Vector3D(1.44,-0.92,0.48),2.0);
                
                   
                //change_flag=false;
                cross_wall_stage=4;
            }
        }
        else if (cross_wall_stage == 4 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot_t->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot_t->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot_t->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {   
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                
                rf_step.update_support_trajectory(wall_rf_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);

                //change_flag=false;
                cross_wall_stage=5;
            }
        }
        else if(cross_wall_stage==5 && change_flag == true){

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            
            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot_t->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot_t->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot_t->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿
            
            if(!success)
            {
                // wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(Vector3D(1.44,-0.92,0.48),Vector3D(-0.12,-1.00,0.96),1.0);
                rf_step.update_support_trajectory(wall_rf_foot_pos,wall_rf_foot_pos,1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,wall_lb_foot_pos,1.0);
                rb_step.update_support_trajectory(wall_rb_foot_pos,wall_rb_foot_pos,1.0);
                //change_flag=false;
                cross_wall_stage=6;
            }
        }
        //右前腿规划：6-9
        else if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();  
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot_t->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot_t->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot_t->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿

            if(!success)
            {
                int result;
                robot_t->rf_leg_calc->set_init_joint_pos(Vector3D(-0.13,-0.42,-0.536));
                rf_joint_exp_pos_ = robot_t->rf_leg_calc->joint_pos(rf_foot_exp_pos,&result);
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                // lf_leg_step.update_support_trajectory(Vector3D(0.12,-1.10,0.48),Vector3D(0.12,-0.80,0.48),2.0);
                rf_step.update_support_trajectory(rf_joint_exp_pos_,Vector3D(-1.35,-0.827,-0.531),1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),1.0);
                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.07),1.0);
                //change_flag=false;
                cross_wall_stage=7;
            }
        }
        else if(cross_wall_stage == 7 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            // std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_leg_step.get_target(time, success);
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(Vector3D(-1.35,-0.827,-0.531),Vector3D(-1.5,1.08,-0.531),2.0);

                //change_flag=false;
                cross_wall_stage=8;
            }
       }
        else if(cross_wall_stage == 8 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_rf_foot_pos=rf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(Vector3D(-1.5,1.08,-0.531),Vector3D(-0.60,1.25,-0.531),2.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,-0.03),2.0);
                
                //change_flag=false;
                cross_wall_stage=9;
            }
       }
        else if(cross_wall_stage == 9 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
           
            if(!success)
            {
                
                wall_rf_foot_pos = rf_joint_exp_pos_;
                wall_lf_foot_pos = lf_joint_exp_pos_;
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;

                // rf_step.update_support_trajectory(Vector3D(-0.60,1.25,-0.531),Vector3D(-0.0267,-0.822,0.165),2.0); 
                // lf_step.update_support_trajectory(Vector3D(0.219,-1.19,0.488),Vector3D(0.0267,0.822,-0.165),2.0); 
                rf_step.update_support_trajectory(robot_t->rf_joint_pos,Vector3D(-0.0267,-0.822,0.165),2.0); 
                lf_step.update_support_trajectory(robot_t->lf_joint_pos,Vector3D(0.0267,0.822,-0.165),2.0); 
                //change_flag=false;               
                cross_wall_stage=10;     
            }       
        }
        else if(cross_wall_stage == 10 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {    
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                cross_wall_stage = 11;
                //change_flag=false;
            }
        }
        else if(cross_wall_stage == 11 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double s = 1.0f;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;
            
            lf_wheel_vel= 0.4;
            rf_wheel_vel=-0.4;
            lb_wheel_vel= 0.4;
            rb_wheel_vel=-0.4;

            double wheel_F = k_F * ((lb_wheel_vel / Robot::WHEEL_RADIUS) - ( robot_t->lf_wheel_omega+robot_t->lb_wheel_omega - robot_t->rb_wheel_omega-robot_t->rf_wheel_omega) / 4.0f);
            lf_wheel_force =  wheel_F;
            rf_wheel_force = -wheel_F;
            lb_wheel_force =  wheel_F;   
            rb_wheel_force = -wheel_F;
            if(time > s)
            { 
                lf_wheel_vel= 0.0;
                rf_wheel_vel= 0.0;
                lb_wheel_vel= 0.0;
                rb_wheel_vel= 0.0;
                lf_wheel_force = 0.0;
                rf_wheel_force = 0.0;
                lb_wheel_force = 0.0;   
                rb_wheel_force = 0.0;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.10,-0.21,0.40),1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.10,0.21,0.40),1.0);

                //change_flag=false;
                cross_wall_stage=12;
            }
            // auto lb_cart_force = robot_t->lb_leg_calc->foot_force(robot_t->lb_joint_pos, robot_t->lb_joint_torque, robot_t->lb_forward_torque);
            // auto rb_cart_force = robot_t->rb_leg_calc->foot_force(robot_t->rb_joint_pos, robot_t->rb_joint_torque, robot_t->rb_forward_torque);
            // if (abs((int)(lb_cart_force[0])) > 16.0 || abs((int)(rb_cart_force[0])) > 16.0)
            // {
            //     RCLCPP_INFO(robot_t->node_->get_logger(),"HELLO");
            //     lf_wheel_vel= 0.0;
            //     rf_wheel_vel= 0.0;
            //     lb_wheel_vel= 0.0;
            //     rb_wheel_vel= 0.0;
                      
            //     lb_wheel_force = 0.0;
            //     rb_wheel_force = 0.0;
            //     lf_wheel_force =  0;
            //     rf_wheel_force =  0;
            //     change_flag=false;
            // }
            
        }
        else if(cross_wall_stage == 12 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();

            lf_wheel_vel= 0.0;
            rf_wheel_vel= 0.0;
            lb_wheel_vel= 0.0;
            rb_wheel_vel= 0.0;

            lf_wheel_force = 0;
            rf_wheel_force = 0;
            lb_wheel_force = 0;   
            rb_wheel_force = 0;

            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.25,-0.08,0.4),1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.25, 0.08,0.4),1.0);

                //change_flag=false;
                cross_wall_stage=13;
            }
        }
        // if(cross_wall_stage == 13 && change_flag == true)
        // {
        //     if (cross_wall_stage != last_stage)
        //     {
        //         cross_wall_stage_time = robot_t->node_->get_clock()->now();
        //         last_stage = cross_wall_stage;
        //     }
        //     bool success=false;
        //     double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
        //     std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_leg_step.get_target(time, success);
        //     std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_leg_step.get_target(time, success);
        //     if(!success)
        //     {
        //         wall_lb_foot_pos=lb_foot_exp_pos;
        //         wall_rb_foot_pos=rb_foot_exp_pos;

        //         rb_leg_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.1,-0.15,0.30),2.0);
        //         lb_leg_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.1,0.15,0.30),2.0);

        //         change_flag=false;
        //         //cross_wall_stage=18;
        //     }
        // }
        else if(cross_wall_stage == 13 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {

                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;


                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(-0.2,0.0,-0.05),5.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(-0.2,0.0,-0.05),5.0);

                //change_flag=false;
                cross_wall_stage=14;
            }
        }
        else if(cross_wall_stage == 14 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            { 
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),1.0);
              
                //change_flag=false;
                cross_wall_stage=15;
            }
        }
        else if(cross_wall_stage==15 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = robot_t->node_->get_clock()->now();
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=(robot_t->node_->get_clock()->now()-cross_wall_stage_time).seconds();
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);

            if(!success)
            {
               
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                rb_step.update_support_trajectory(wall_rb_foot_pos,Vector3D(0.0,0.0,0.0),0.5);
                lb_step.update_support_trajectory(wall_lb_foot_pos,Vector3D(0.0,0.0,0.0),0.5);

              
                //cross_wall_stage=21;
                change_flag=false;
                auto lf_cart_pos = robot_t->lf_leg_calc->foot_pos(robot_t->lf_joint_pos);
                auto rf_cart_pos = robot_t->rf_leg_calc->foot_pos(robot_t->rf_joint_pos);
                auto lb_cart_pos = robot_t->lb_leg_calc->foot_pos(robot_t->lb_joint_pos);
                auto rb_cart_pos = robot_t->rb_leg_calc->foot_pos(robot_t->rb_joint_pos);
                robot_t->lf_z_vmc->reset(lf_cart_pos.z(), 0.0);
                robot_t->rf_z_vmc->reset(rf_cart_pos.z(), 0.0);
                robot_t->lb_z_vmc->reset(lb_cart_pos.z(), 0.0);
                robot_t->rb_z_vmc->reset(rb_cart_pos.z(), 0.0);

                return "stop";
            }
        }



/*******************************lf**********************************/
        if(use_limit_lf)
        {

            for(int i=0;i<3;i++)
            {
                joints_target.legs[0].joints[i].kp   = (float)robot_t->lf_leg_calc->kp[i];
                joints_target.legs[0].joints[i].kd   = (float)robot_t->lf_leg_calc->kd[i];
                joints_target.legs[0].joints[i].rad    = (float)lf_joint_exp_pos_[i];
            }
            robot_t->lf_leg_calc->joint_pos_setarray(robot_t->lf_joint_pos);
            lf_foot_exp_pos = robot_t->lf_leg_calc->foot_pos(robot_t->lf_joint_pos);
            // static int cnt_ = 0;
            // cnt_++;
            // if(cnt_>=1)
            // {
            //     cnt_ = 0;
            //     RCLCPP_ERROR(robot_t->node_->get_logger(),
            //         "\033[31mlf_joint_pos = (%.2f, %.2f %.2f), lf_joint_exp_pos_ = (%.2f, %.2f, %.2f)\033[0m",
            //                 robot_t->lf_joint_pos[0], robot_t->lf_joint_pos[1],robot_t->lf_joint_pos[2],lf_joint_exp_pos_[0],lf_joint_exp_pos_[1],lf_joint_exp_pos_[2]);
            // }   
        }
        else
        {
            joints_target.legs[0] =
                robot_t->lf_leg_calc->signal_leg_calc(lf_foot_exp_pos, lf_foot_exp_vel, 
                    lf_foot_exp_acc, lf_foot_exp_force,  
                    &robot_t->lf_forward_torque,lf_wheel_vel,lf_wheel_force);
        }

/*******************************rf**********************************/
        if(use_limit_rf)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[1].joints[i].rad =(float)rf_joint_exp_pos_[i];
            joints_target.legs[1].joints[i].kp = (float)robot_t->rf_leg_calc->kp[i];
            joints_target.legs[1].joints[i].kd = (float)robot_t->rf_leg_calc->kd[i];
            }
            robot_t->rf_leg_calc->joint_pos_setarray(robot_t->rf_joint_pos);
            rf_foot_exp_pos = robot_t->rf_leg_calc->foot_pos(robot_t->rf_joint_pos);
        }
        else
        {
        joints_target.legs[1] =
            robot_t->rf_leg_calc->signal_leg_calc(rf_foot_exp_pos, rf_foot_exp_vel, 
            rf_foot_exp_acc, rf_foot_exp_force,  
            &robot_t->rf_forward_torque,rf_wheel_vel,rf_wheel_force);
        }
/*******************************lb**********************************/
        if(use_limit_lb)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[2].joints[i].kp = (float)robot_t->lb_leg_calc->kp[i];
            joints_target.legs[2].joints[i].kd = (float)robot_t->lb_leg_calc->kd[i];
            joints_target.legs[2].joints[i].rad =(float)lb_joint_exp_pos_[i];
            }
            robot_t->lb_leg_calc->joint_pos_setarray(robot_t->lb_joint_pos);
            lb_foot_exp_pos = robot_t->lb_leg_calc->foot_pos(robot_t->lb_joint_pos);
        }
        else
        {
            joints_target.legs[2] =
            robot_t->lb_leg_calc->signal_leg_calc(lb_foot_exp_pos, lb_foot_exp_vel, 
            lb_foot_exp_acc, lb_foot_exp_force,  
            &robot_t->lb_forward_torque,lb_wheel_vel,lb_wheel_force);
        }
/*******************************rb**********************************/
        if(use_limit_rb)
        {
            for(int i=0;i<3;i++)
            {
            joints_target.legs[3].joints[i].kp = (float)robot_t->rb_leg_calc->kp[i];
            joints_target.legs[3].joints[i].kd = (float)robot_t->rb_leg_calc->kd[i];
            joints_target.legs[3].joints[i].rad =(float)rb_joint_exp_pos_[i];
            }
            robot_t->rb_leg_calc->joint_pos_setarray(robot_t->rb_joint_pos);
            rb_foot_exp_pos = robot_t->rb_leg_calc->foot_pos(robot_t->rb_joint_pos);
        }else
        {
            joints_target.legs[3] =
            robot_t->rb_leg_calc->signal_leg_calc(rb_foot_exp_pos, rb_foot_exp_vel, 
            rb_foot_exp_acc, rb_foot_exp_force,  
            &robot_t->rb_forward_torque,rb_wheel_vel,rb_wheel_force);
        }

    }

    void Exit() override {}

    std::string CheckChange() override
    {
        
        return state_name_;
    }



};
/*
*
*
* 
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*
*/

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab";
        std::string robot_config_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            rl.InitRL(robot_config_path);
            rl.now_state = *fsm_state;
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            rl.fsm.RequestStateChange("RLFSMStatePassive");
        }
    }

    void Run() override
    {
        // position transition from last default_dof_pos to current default_dof_pos
        // if (Interpolate(percent_transition, rl.now_state.motor_state.q, rl.params.Get<std::vector<float>>("default_dof_pos"), 0.5f, "Policy transition", true)) return;

        if (!rl.rl_init_done) rl.rl_init_done = true;

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller [" << rl.config_name << "] x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        RLControl();
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
        {
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRLLocomotion";
        }
        return state_name_;
    }
};

} // namespace atdog2_fsm

class ATDog2FSMFactory : public FSMFactory
{
public:
    ATDog2FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<atdog2_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<atdog2_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<atdog2_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<atdog2_fsm::RLFSMStateRLLocomotion>(rl);
        else if (state_name == "RLFSMStateCrosswall")
            return std::make_shared<atdog2_fsm::RLFSMStateCrosswall>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "atdog2"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion",
            "RLFSMStateCrosswall"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(ATDog2FSMFactory, "RLFSMStatePassive")

#endif // ATDOG2_FSM_HPP
