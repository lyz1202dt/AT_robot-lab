/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATDOG2_FSM_HPP
#define ATDOG2_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <Eigen/Dense>

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

class RLFSMStateCrosswall : public RLFSMState
{
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
    
public:
    RLFSMStateCrosswall(RL *rl) : RLFSMState(*rl, "RLFSMStateCrosswall") {}

    int cross_wall_stage{-1};

    Eigen::Vector3d  wall_lf_foot_pos{0,0,0}, wall_rf_foot_pos{0,0,0}, wall_lb_foot_pos{0,0,0}, wall_rb_foot_pos{0,0,0};
    Eigen::Vector3d  lf_foot_exp_pos{0,0,0}, rf_foot_exp_pos{0,0,0}, lb_foot_exp_pos{0,0,0}, rb_foot_exp_pos{0,0,0};
    Eigen::Vector3d  lf_foot_exp_force{0,0,0}, rf_foot_exp_force{0,0,0}, lb_foot_exp_force{0,0,0}, rb_foot_exp_force{0,0,0};
    Eigen::Vector3d  lf_foot_exp_vel{0,0,0}, rf_foot_exp_vel{0,0,0}, lb_foot_exp_vel{0,0,0}, rb_foot_exp_vel{0,0,0};
    Eigen::Vector3d  lf_foot_exp_acc{0,0,0}, rf_foot_exp_acc{0,0,0}, lb_foot_exp_acc{0,0,0}, rb_foot_exp_acc{0,0,0};
    Eigen::Vector3d  lf_forward_torque{0,0,0}, rf_forward_torque{0,0,0}, lb_forward_torque{0,0,0}, rb_forward_torque{0,0,0};

    //使用关节角度
    Eigen::Vector3d  lf_joint_exp_pos_{0,0,0},rf_joint_exp_pos_{0,0,0},
             lb_joint_exp_pos_{0,0,0},rb_joint_exp_pos_{0,0,0};
    Eigen::Vector3d  lf_joint_omega{0,0,0}, rf_joint_omega{0,0,0},
             lb_joint_omega{0,0,0},rb_joint_omega{0,0,0};
    Eigen::Vector3d  lf_joint_torque{0,0,0},rf_joint_torque{0,0,0},
             lb_joint_torque{0,0,0},rb_joint_torque{0,0,0};

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

    float k_F{1.0f};

    //力变量
    Eigen::Vector2d mass_center_pos;
    double mass;

    Cross_Step lf_step, rf_step, lb_step, rb_step;
    

    void Enter() override
    {
        
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        
        return state_name_;
    }



};

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
