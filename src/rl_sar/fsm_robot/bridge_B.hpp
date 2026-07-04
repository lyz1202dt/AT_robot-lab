#pragma once

#include "cross_wall.hpp"

class BridgeBStateAtdog2 {
public:
    explicit BridgeBStateAtdog2(const std::string& urdf_file_path);
    void configure(
        double crouch_time = 1.0, double step_height = 0.25, double step_forward_distance = 0.2, double bridge_surface_z = 0.4,
        double exit_stance_z = -0.02,
        double stance_z_offset = 0.0);
    void enter();
    RobotTarget update();
    std::tuple<Eigen::Vector3d, double> get_robot_mass_info(
        const Eigen::Vector3d& lf_joint_pos, const Eigen::Vector3d& rf_joint_pos,
        const Eigen::Vector3d& lb_joint_pos, const Eigen::Vector3d& rb_joint_pos);

    std::shared_ptr<Robot_t> robot;
    int bridge_stage{-1};
    std::chrono::steady_clock::time_point bridge_stage_time;
    std::chrono::steady_clock::time_point last_update_time;

    Eigen::Vector3d lf_foot_exp_pos{0.0, 0.0, 0.0}, rf_foot_exp_pos{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_foot_exp_pos{0.0, 0.0, 0.0}, rb_foot_exp_pos{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_foot_exp_force{0.0, 0.0, 0.0}, rf_foot_exp_force{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_foot_exp_force{0.0, 0.0, 0.0}, rb_foot_exp_force{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_foot_exp_vel{0.0, 0.0, 0.0}, rf_foot_exp_vel{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_foot_exp_vel{0.0, 0.0, 0.0}, rb_foot_exp_vel{0.0, 0.0, 0.0};
    Eigen::Vector3d lf_foot_exp_acc{0.0, 0.0, 0.0}, rf_foot_exp_acc{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_foot_exp_acc{0.0, 0.0, 0.0}, rb_foot_exp_acc{0.0, 0.0, 0.0};

    Eigen::Vector3d lf_init_joint_pos{0.0, 0.0, 0.0}, rf_init_joint_pos{0.0, 0.0, 0.0};
    Eigen::Vector3d lb_init_joint_pos{0.0, 0.0, 0.0}, rb_init_joint_pos{0.0, 0.0, 0.0};


    double lf_wheel_vel{0.0}, rf_wheel_vel{0.0}, lb_wheel_vel{0.0}, rb_wheel_vel{0.0};
    double lf_wheel_force{0.0}, rf_wheel_force{0.0}, lb_wheel_force{0.0}, rb_wheel_force{0.0};

    double crouch_time{1.0};
    double drive_time{2.0};
    double stand_time{1.0};
    double target_distance{0.55};
    double target_drive_velocity{0.30};
    double total_distance{0.0};
    double step_height{0.25};
    double step_forward_distance{0.2};
    double bridge_surface_z{0.2};
    double exit_stance_z{-0.02};
    double stance_z_offset{0.0};
    double gait_step_time{0.6};
    double gait_support_rate{0.6};
    double bridge_board_width{0.40};
    double bridge_board_gap{0.15};
    double bridge_swing_clearance{0.05};
    double bridge_swing_lateral_offset{0.07};
    double bridge_support_scrape_depth{0.01};
    int bridge_board_count{4};
    int bridge_cycle_limit{100};
    int bridge_pair_phase{0};
    int bridge_pair_phase_total{8};
    bool bridge_pair_phase_active{false};
    bool bridge_active_move_lf_rb{true};
    bool bridge_lateral_targets_initialized{false};
    double bridge_lf_target_y{0.0};
    double bridge_rf_target_y{0.0};
    double bridge_lb_target_y{0.0};
    double bridge_rb_target_y{0.0};
    double bridge_level_shank_blend_time{0.12};

    bool RL_walk_flag{false};
    bool bridge_over{false};
    bool diagonal_walk_started{false};

    Eigen::Vector2d mass_center_pos{0.0, 0.0};
    double mass{0.0};

    Cross_Step lf_step, rf_step, lb_step, rb_step;
    DiagonalWalkController diagonal_walk;

private:
    double low_pass_filter(double input, int wheel_idx);
    double get_elapsed_time() const;
    void reset_targets_to_current_pose();
    RobotTarget build_robot_target();

    double last_filtered_wheel_vel[4]{0.0, 0.0, 0.0, 0.0};
};

using BridgeBState = BridgeBStateAtdog2;
