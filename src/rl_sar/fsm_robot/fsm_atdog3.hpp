/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ATDOG3_FSM_HPP
#define ATDOG3_FSM_HPP

#include "fsm.hpp"
#include "rl_sdk.hpp"
#include <Eigen/Dense>
#include <cmath>  // for std::isnan, std::isinf
#include "cross_wall_atdog3.hpp"
namespace atdog3_fsm
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
        if(rl.control.mode==1)  //检查切换到位控站立
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
        if(rl.fsm.previous_state_->GetStateName()=="RLFSMStateCrosswall")
        {
            return "RLFSMStateRLLocomotion";
        }
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
            else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::LB_DPadDown)
            {
                return "RLFSMStateCrosswall";
            }
            else if(rl.control.current_keyboard == Input::Keyboard::Num7)
            {
                return "RLFSMStateCheck";
            }

            //std::cout<<"检查切换\n";
            if(rl.control.mode==2)   //转到位控站立状态
            {
                std::cout<<"切入普通行走状态\n";
                return "RLFSMStateRLLocomotion";
            }
            else if(rl.control.mode==3)     //切换到爬台阶状态
            {
                return "RLFSMStateRLStairs";
            }
            else if(rl.control.mode==4)     //切换到沙石地行走状态
            {
                return "RLFSMStateRLSand";
            }
            else if(rl.control.mode==5)     //切换到限高杆状态
            {
                return "RLFSMStateRLBar";
            }
            else if(rl.control.mode==6)     //切换到斜坡状态
            {
                return "RLFSMStateRLSlope";
            }
            else if(rl.control.mode==7)     //切换到木桥状态
            {
                return "RLFSMStateRLBridge";
            }
            else if(rl.control.mode==8)     //切换到木桥状态
            {
                return "RLFSMStateCrosswall";
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

        if (rl.control.mode==1)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateCrosswall : public RLFSMState {


public:
    int num_dofs;
    std::shared_ptr<CrossWallStateAtdog3> cross_wall_state;
    RLFSMStateCrosswall(RL *rl) : RLFSMState(*rl, "RLFSMStateCrosswall") {
        std::string urdf_path = "src/rl_sar_zoo/" + rl->robot_name + "_description/urdf/dog3.urdf";
        cross_wall_state = std::make_shared<CrossWallStateAtdog3>(urdf_path);
    }
    
    

    void Enter() override
    {
        rl.now_state = *fsm_state;
        num_dofs = rl.params.Get<int>("num_of_dofs");
        cross_wall_state->enter();
        
    }

    void Run() override
    {
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;    // ✅ atdog2 是 3 关节/腿
            int jonit_index = i % 3;  // ✅ 索引范围 0-2
            // 获取当前电机状态
            switch (leg_index) 
            {
                case 0:
                    cross_wall_state->robot->rf_joint_pos[jonit_index] = fsm_state->motor_state.q[i];      // 位置
                    cross_wall_state->robot->rf_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];    // 速度
                break;
                case 1:
                    cross_wall_state->robot->lf_joint_pos[jonit_index] = fsm_state->motor_state.q[i];      
                    cross_wall_state->robot->lf_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                case 3:
                    cross_wall_state->robot->lb_joint_pos[jonit_index] = fsm_state->motor_state.q[i];
                    cross_wall_state->robot->lb_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                case 2:
                    cross_wall_state->robot->rb_joint_pos[jonit_index] = fsm_state->motor_state.q[i];
                    cross_wall_state->robot->rb_joint_vel[jonit_index] = fsm_state->motor_state.dq[i];
                break;
                default:
                    break;
            }
            
        }
        RobotTarget joints_target;
        joints_target = cross_wall_state->update();
        for (int i = 0; i < num_dofs; ++i)
        {
            int leg_index = i / 3;    // ✅ atdog2 是 3 关节/腿
            int jonit_index = i % 3;  // ✅ 索引范围 0-2
            
            // 防止 NaN 或 Inf 值
            float q_val = joints_target.legs[leg_index].joints[jonit_index].rad;
            float dq_val = joints_target.legs[leg_index].joints[jonit_index].omega;
            float kp_val = joints_target.legs[leg_index].joints[jonit_index].kp;
            float kd_val = joints_target.legs[leg_index].joints[jonit_index].kd;

            
            // 检查并修正无效值
            // if (std::isnan(q_val) || std::isinf(q_val)) q_val = 0.0f;
            // if (std::isnan(dq_val) || std::isinf(dq_val)) dq_val = 0.0f;
            // if (std::isnan(kp_val) || std::isinf(kp_val)) kp_val = 0.0f;
            // if (std::isnan(kd_val) || std::isinf(kd_val)) kd_val = 0.0f;
            
            fsm_command->motor_command.q[i] = q_val;
            fsm_command->motor_command.dq[i] = dq_val;
            fsm_command->motor_command.kp[i] = kp_val;
            fsm_command->motor_command.kd[i] = kd_val;
            fsm_command->motor_command.tau[i] = joints_target.legs[leg_index].joints[jonit_index].torque;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if(cross_wall_state->RL_walk_flag == true)
        {
            cross_wall_state->RL_walk_flag = false;
            cross_wall_state->cross_wall_stage = 13;
            return "RLFSMStateRLLocomotion";
        }
        if(cross_wall_state->Cross_wall_over == true)
        {
            cross_wall_state->Cross_wall_over = false;
            cross_wall_state->cross_wall_stage = -1;
            return "RLFSMStateGetUp";
        }
        
        return state_name_;
    }
};

class RLFSMStateRLLocomotion : public RLFSMState
{
public:
    RLFSMStateRLLocomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRLLocomotion") {}

    float percent_transition = 0.0f;
    std::chrono::steady_clock::time_point cross_enter_time = std::chrono::steady_clock::now();
    bool RL_to_Cross = false;

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
        if (rl.fsm.previous_state_->GetStateName() == "RLFSMStateCrosswall")
        {
            cross_enter_time = std::chrono::steady_clock::now();
            RL_to_Cross = true;
            rl.control.setVel(0.4f, 0.0f, 0.0f);
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
        if((std::chrono::steady_clock::now() - cross_enter_time > std::chrono::milliseconds(3000)) && RL_to_Cross)
        {
            RL_to_Cross = false;
            rl.control.setVel(0.0f, 0.0f, 0.0f);
            return "RLFSMStateCrosswall";
        }
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
        else if (rl.control.current_keyboard == Input::Keyboard::Num5 || rl.control.current_gamepad == Input::Gamepad::RB_DPadDown)
        {
            return "RLFSMStateCrosswall";
        }

        //遥控器切换
        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==3)     //切换到爬台阶状态
        {
            return "RLFSMStateRLStairs";
        }
        else if(rl.control.mode==4)     //切换到沙石地行走状态
        {
            return "RLFSMStateRLSand";
        }
        else if(rl.control.mode==6)     //切换到斜坡状态
        {
            return "RLFSMStateRLSlope";
        }
        else if(rl.control.mode==5)     //切换到限高杆状态
        {
            return "RLFSMStateRLBar";
        }
        else if(rl.control.mode==7)     //切换到木桥状态
        {
            return "RLFSMStateRLBridge";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};


class RLFSMStateCheck : public RLFSMState
{
public:
    RLFSMStateCheck(RL *rl) : RLFSMState(*rl, "RLFSMStateCheck") {}


    float percent_pre_getup = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00, -1.0, 0.3,
        0.00, 1.0, -0.3,
        0.00, -1.0, 0.3,
        0.00, 1.0, -0.3,
    };

    void Enter() override
    {
        percent_pre_getup = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        std::cout<<"当前关节角:"<<rl.now_state.motor_state.q<<std::endl;
    }

    void Run() override
    {
        std::cout<<"当前关节角:"<<percent_pre_getup<<std::endl;
            if (Interpolate(percent_pre_getup, rl.now_state.motor_state.q, pre_running_pos, 3.0f, "RLFSMStateCheck", true)) return;
    }

    void Exit() override {}

    std::string CheckChange() override
    {

        if (percent_pre_getup >= 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::Num9 || rl.control.current_gamepad == Input::Gamepad::B)
            {
                return "RLFSMStateGetDown";
            }
            else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
            {
                return "RLFSMStateGetUp";
            }

            //std::cout<<"检查切换\n";
            if(rl.control.mode==1)   //转到位控站立状态
            {
                return "RLFSMStateGetUp";
            }

        }
        
        return state_name_;
    }
};


class RLFSMStateRLStairs : public RLFSMState
{
public:
    RLFSMStateRLStairs(RL *rl) : RLFSMState(*rl, "RLFSMStateRLStairs") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_stairs";
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

        //遥控器切换
        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==2)     //切换到普通行走模式
        {
            return "RLFSMStateRLLocomotion";
        }
        else if(rl.control.mode==4)     //切换到沙石地行走状态
        {
            return "RLFSMStateRLSand";
        }
        else if(rl.control.mode==6)     //切换到斜坡状态
        {
            return "RLFSMStateRLSlope";
        }
        else if(rl.control.mode==5)     //切换到限高杆状态
        {
            return "RLFSMStateRLBar";
        }
        else if(rl.control.mode==7)     //切换到木桥状态
        {
            return "RLFSMStateRLBridge";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};

class RLFSMStateRLSand : public RLFSMState
{
public:
    RLFSMStateRLSand(RL *rl) : RLFSMState(*rl, "RLFSMStateRLSand") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_sand";
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

        //遥控器切换
        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==2)     //切换到普通行走模式
        {
            return "RLFSMStateRLLocomotion";
        }
        else if(rl.control.mode==3)     //切换到爬台阶状态
        {
            return "RLFSMStateRLStairs";
        }
        else if(rl.control.mode==6)     //切换到斜坡状态
        {
            return "RLFSMStateRLSlope";
        }
        else if(rl.control.mode==5)     //切换到限高杆状态
        {
            return "RLFSMStateRLBar";
        }
        else if(rl.control.mode==7)     //切换到木桥状态
        {
            return "RLFSMStateRLBridge";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};

class RLFSMStateRLSlope : public RLFSMState
{
public:
    RLFSMStateRLSlope(RL *rl) : RLFSMState(*rl, "RLFSMStateRLSlope") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_slope";
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

        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==2)     //切换到普通行走模式
        {
            return "RLFSMStateRLLocomotion";
        }
        else if(rl.control.mode==3)     //切换到爬台阶状态
        {
            return "RLFSMStateRLStairs";
        }
        else if(rl.control.mode==4)     //切换到沙石地行走状态
        {
            return "RLFSMStateRLSand";
        }
        else if(rl.control.mode==5)     //切换到限高杆状态
        {
            return "RLFSMStateRLBar";
        }
        else if(rl.control.mode==7)     //切换到木桥状态
        {
            return "RLFSMStateRLBridge";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};

class RLFSMStateRLBar : public RLFSMState
{
public:
    RLFSMStateRLBar(RL *rl) : RLFSMState(*rl, "RLFSMStateRLBar") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_bar";
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

        //遥控器切换
        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==2)     //切换到普通行走模式
        {
            return "RLFSMStateRLLocomotion";
        }
        else if(rl.control.mode==3)     //切换到爬台阶状态
        {
            return "RLFSMStateRLStairs";
        }
        else if(rl.control.mode==4)     //切换到沙石地行走状态
        {
            return "RLFSMStateRLSand";
        }
        else if(rl.control.mode==6)     //切换到斜坡状态
        {
            return "RLFSMStateRLSlope";
        }
        else if(rl.control.mode==7)     //切换到木桥状态
        {
            return "RLFSMStateRLBridge";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};

class RLFSMStateRLBridge : public RLFSMState
{
public:
    RLFSMStateRLBridge(RL *rl) : RLFSMState(*rl, "RLFSMStateRLBridge") {}

    float percent_transition = 0.0f;

    void Enter() override
    {
        percent_transition = 0.0f;
        rl.episode_length_buf = 0;

        // read params from yaml
        rl.config_name = "robot_lab_bridge";
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

        if(rl.control.mode==1)   //转到位控站立状态
        {
            return "RLFSMStateGetUp";
        }
        else if(rl.control.mode==2)     //切换到普通行走模式
        {
            return "RLFSMStateRLLocomotion";
        }
        else if(rl.control.mode==3)     //切换到爬台阶状态
        {
            return "RLFSMStateRLStairs";
        }
        else if(rl.control.mode==4)     //切换到沙石地行走状态
        {
            return "RLFSMStateRLSand";
        }
        else if(rl.control.mode==5)     //切换到限高杆状态
        {
            return "RLFSMStateRLBar";
        }
        else if(rl.control.mode==6)     //切换到斜坡状态
        {
            return "RLFSMStateRLSlope";
        }
        else if(rl.control.mode==8)     //切换到木桥状态
        {
            return "RLFSMStateCrosswall";
        }
        return state_name_;
    }
};

} // namespace atdog3_fsm

class ATDog3FSMFactory : public FSMFactory
{
public:
    ATDog3FSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<atdog3_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<atdog3_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<atdog3_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<atdog3_fsm::RLFSMStateRLLocomotion>(rl);
        else if (state_name == "RLFSMStateRLStairs")
            return std::make_shared<atdog3_fsm::RLFSMStateRLStairs>(rl);
        else if (state_name == "RLFSMStateRLSand")
            return std::make_shared<atdog3_fsm::RLFSMStateRLSand>(rl);
        else if (state_name == "RLFSMStateCrosswall")
            return std::make_shared<atdog3_fsm::RLFSMStateCrosswall>(rl);
        else if (state_name == "RLFSMStateRLSlope")
            return std::make_shared<atdog3_fsm::RLFSMStateRLSlope>(rl);
        else if (state_name == "RLFSMStateRLBar")
            return std::make_shared<atdog3_fsm::RLFSMStateRLBar>(rl);
        else if (state_name == "RLFSMStateRLBridge")
            return std::make_shared<atdog3_fsm::RLFSMStateRLBridge>(rl);
        else if (state_name == "RLFSMStateCheck")
            return std::make_shared<atdog3_fsm::RLFSMStateCheck>(rl);
        
        return nullptr;
    }
    std::string GetType() const override { return "atdog3"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion",
            "RLFSMStateRLStairs",
            "RLFSMStateRLSand",
            "RLFSMStateCrosswall",
            "RLFSMStateRLSlope",
            "RLFSMStateRLBar",
            "RLFSMStateRLBridge",
            "RLFSMStateCheck"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(ATDog3FSMFactory, "RLFSMStatePassive")

#endif // ATDOG3_FSM_HPP
