/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_real_atdog2.hpp"

RL_Real::RL_Real(int argc, char **argv)
{
    this->robot_name="atdog2";
    this->ReadYaml("atdog2", "base.yaml");

    //创建状态机
    this->fsm =*FSMManager::GetInstance().CreateFSM("atdog2", this);

    //初始化机器人
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    // Shut down motion control-related service

    //键盘控制、底层控制、策略推理循环
    this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    this->loop_keyboard->start();
    this->loop_control->start();
    this->loop_rl->start();
}

RL_Real::~RL_Real()
{
    this->loop_keyboard->shutdown();
    this->loop_control->shutdown();
    this->loop_rl->shutdown();
    std::cout << LOGGER::INFO << "RL_Real exit" << std::endl;
}

void RL_Real::GetState(RobotState<float> *state)
{
    // if (this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::A);
    // if (this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::B);
    // if (this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::X);
    // if (this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::Y);
    // if (this->unitree_joy.components.L1) this->control.SetGamepad(Input::Gamepad::LB);
    // if (this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::RB);
    // if (this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LStick);
    // if (this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RStick);
    // if (this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::DPadUp);
    // if (this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::DPadDown);
    // if (this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    // if (this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::DPadRight);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::LB_A);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::LB_B);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::LB_X);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::LB_Y);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.A) this->control.SetGamepad(Input::Gamepad::RB_A);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.B) this->control.SetGamepad(Input::Gamepad::RB_B);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.X) this->control.SetGamepad(Input::Gamepad::RB_X);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.Y) this->control.SetGamepad(Input::Gamepad::RB_Y);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.F1) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.F2) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.up) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.down) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.left) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    // if (this->unitree_joy.components.R1 && this->unitree_joy.components.right) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    // if (this->unitree_joy.components.L1 && this->unitree_joy.components.R1) this->control.SetGamepad(Input::Gamepad::LB_RB);

    // this->control.x = this->joystick.ly();
    // this->control.y = -this->joystick.lx();
    // this->control.yaw = -this->joystick.rx();

    // state->imu.quaternion[0] = this->unitree_low_state.imu_state().quaternion()[0]; // w
    // state->imu.quaternion[1] = this->unitree_low_state.imu_state().quaternion()[1]; // x
    // state->imu.quaternion[2] = this->unitree_low_state.imu_state().quaternion()[2]; // y
    // state->imu.quaternion[3] = this->unitree_low_state.imu_state().quaternion()[3]; // z

    // for (int i = 0; i < 3; ++i)
    // {
    //     state->imu.gyroscope[i] = this->unitree_low_state.imu_state().gyroscope()[i];
    // }
    // for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    // {
    //     state->motor_state.q[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q();
    //     state->motor_state.dq[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq();
    //     state->motor_state.tau_est[i] = this->unitree_low_state.motor_state()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau_est();
    // }
}

void RL_Real::RobotControl()
{
    //获取各个传感器数据，遥控器期望，填写到robot_state中
    this->GetState(&this->robot_state);     

    //执行状态机，送入state，输出command
    this->StateController(&this->robot_state, &this->robot_command);    

    //清空上一次遥控器的输入
    this->control.ClearInput();     

    //将命令发送到电机
    this->SetCommand(&this->robot_command);     
}

void RL_Real::SetCommand(const RobotCommand<float> *command)
{
//     unitree_go::msg::dds_::LowCmd_ dds_low_command;
//     dds_low_command.head()[0] = 0xFE;
//     dds_low_command.head()[1] = 0xEF;
//     dds_low_command.level_flag() = 0xFF;
//     dds_low_command.gpio() = 0;

//     for (int i = 0; i < 20; ++i)
//     {
//         dds_low_command.motor_cmd()[i].mode() = 0x01;
//         dds_low_command.motor_cmd()[i].q() = PosStopF;
//         dds_low_command.motor_cmd()[i].kp() = 0;
//         dds_low_command.motor_cmd()[i].dq() = VelStopF;
//         dds_low_command.motor_cmd()[i].kd() = 0;
//         dds_low_command.motor_cmd()[i].tau() = 0;
//     }

//     for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
//     {
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].mode() = 0x01;
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].q() = command->motor_command.q[i];
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].dq() = command->motor_command.dq[i];
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kp() = command->motor_command.kp[i];
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].kd() = command->motor_command.kd[i];
//         dds_low_command.motor_cmd()[this->params.Get<std::vector<int>>("joint_mapping")[i]].tau() = command->motor_command.tau[i];
//     }

//     dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
//     lowcmd_publisher->Write(dds_low_command);

}

void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE) && defined(USE_ROS)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};

        }
#endif
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);
    }
}

std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}


volatile sig_atomic_t g_shutdown_requested = 0;
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", shutting down..." << std::endl;
    g_shutdown_requested = 1;
}


int main(int argc, char **argv)
{
    RL_Real rl_sar(argc, argv);
    while (!g_shutdown_requested) { sleep(1); }
    std::cout << LOGGER::INFO << "Exiting..." << std::endl;

    return 0;
}
