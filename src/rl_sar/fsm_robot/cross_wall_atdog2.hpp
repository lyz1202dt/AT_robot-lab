#pragma once

#include "cross_wall.hpp"

class CrossWallStateAtdog2 {
public:
    CrossWallStateAtdog2(const std::string& urdf_file_path);
    void enter();
    RobotTarget update();
    std::tuple<Eigen::Vector3d, double> get_robot_mass_info(
        const Eigen::Vector3d& lf_joint_pos, const Eigen::Vector3d& rf_joint_pos,
        const Eigen::Vector3d& lb_joint_pos, const Eigen::Vector3d& rb_joint_pos);

    std::shared_ptr<Robot_t> robot;
    int cross_wall_stage{-1};
    std::chrono::steady_clock::time_point cross_wall_stage_time;

    Eigen::Vector3d wall_lf_foot_pos{0, 0, 0}, wall_rf_foot_pos{0, 0, 0}, wall_lb_foot_pos{0, 0, 0}, wall_rb_foot_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_pos{0, 0, 0}, rf_foot_exp_pos{0, 0, 0}, lb_foot_exp_pos{0, 0, 0}, rb_foot_exp_pos{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_force{0, 0, 0}, rf_foot_exp_force{0, 0, 0}, lb_foot_exp_force{0, 0, 0}, rb_foot_exp_force{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_vel{0, 0, 0}, rf_foot_exp_vel{0, 0, 0}, lb_foot_exp_vel{0, 0, 0}, rb_foot_exp_vel{0, 0, 0};
    Eigen::Vector3d lf_foot_exp_acc{0, 0, 0}, rf_foot_exp_acc{0, 0, 0}, lb_foot_exp_acc{0, 0, 0}, rb_foot_exp_acc{0, 0, 0};
    Eigen::Vector3d lf_forward_torque{0, 0, 0}, rf_forward_torque{0, 0, 0}, lb_forward_torque{0, 0, 0}, rb_forward_torque{0, 0, 0};

    Eigen::Vector3d lf_joint_exp_pos_{0, 0, 0}, rf_joint_exp_pos_{0, 0, 0}, lb_joint_exp_pos_{0, 0, 0}, rb_joint_exp_pos_{0, 0, 0};
    Eigen::Vector3d lf_joint_omega{0, 0, 0}, rf_joint_omega{0, 0, 0}, lb_joint_omega{0, 0, 0}, rb_joint_omega{0, 0, 0};
    Eigen::Vector3d lf_joint_torque{0, 0, 0}, rf_joint_torque{0, 0, 0}, lb_joint_torque{0, 0, 0}, rb_joint_torque{0, 0, 0};

    double lf_wheel_vel{0.0}, rf_wheel_vel{0.0}, lb_wheel_vel{0.0}, rb_wheel_vel{0.0};
    double lf_wheel_force{0.0}, rf_wheel_force{0.0}, lb_wheel_force{0.0}, rb_wheel_force{0.0};

    bool stopping = false;
    double stop_t = 0.0;
    double stop_T = 0.3;

    double lf_vel_start, rf_vel_start, lb_vel_start, rb_vel_start;
    double lf_force_start, rf_force_start, lb_force_start, rb_force_start;

    double time_s{1.0};
    bool change_flag{true};
    bool allow_vel{true};

    float k_F{1.0f};

    Eigen::Vector2d mass_center_pos;
    double mass;
    bool RL_walk_flag{false};
    bool Cross_wall_over{false};

    Cross_Step lf_step, rf_step, lb_step, rb_step;
};
