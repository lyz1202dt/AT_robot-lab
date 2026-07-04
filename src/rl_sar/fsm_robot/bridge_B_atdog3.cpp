#include "bridge_B_atdog3.hpp"

#include <algorithm>
#include <cmath>

using Vector3D = Eigen::Vector3d;

BridgeBStateAtdog3::BridgeBStateAtdog3(const std::string& urdf_file_path)
{
    robot = std::make_shared<Robot_t>(urdf_file_path);
    bridge_stage = -1;
    bridge_stage_time = std::chrono::steady_clock::now();
    last_update_time = bridge_stage_time;
}

std::tuple<Vector3D, double> BridgeBStateAtdog3::get_robot_mass_info(
    const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos)
{
    auto accumulate_chain_com = [](const KDL::Chain& chain, const Vector3D& q_eigen, double& mass_sum, KDL::Vector& com_sum) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints() && i < 3; ++i) {
            q(i) = q_eigen[i];
        }

        KDL::Frame T = KDL::Frame::Identity();
        unsigned int joint_idx = 0;

        for (unsigned int seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
            const auto& seg = chain.getSegment(seg_idx);
            if (seg.getJoint().getType() != KDL::Joint::None) {
                T = T * seg.pose(q(joint_idx));
                joint_idx++;
            } else {
                T = T * seg.pose(0.0);
            }

            const auto& rbi = seg.getInertia();
            const double m = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            const KDL::Vector c_base = T * rbi.getCOG();
            mass_sum += m;
            com_sum = com_sum + c_base * m;
        }
    };

    double mass_sum = 0.0;
    KDL::Vector com_sum(0.0, 0.0, 0.0);

    accumulate_chain_com(robot->lf_leg_chain, lf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rf_leg_chain, rf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->lb_leg_chain, lb_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rb_leg_chain, rb_joint_pos, mass_sum, com_sum);

    const auto it = robot->tree.getSegment("body_link");
    if (it != robot->tree.getSegments().end()) {
        const auto& rbi = it->second.segment.getInertia();
        const double m = rbi.getMass();
        if (m > 0.0) {
            mass_sum += m;
            com_sum = com_sum + rbi.getCOG() * m;
        }
    }

    if (mass_sum <= 1e-9) {
        return std::make_tuple(Vector3D::Zero(), 0.0);
    }

    const KDL::Vector com = com_sum / mass_sum;
    return std::make_tuple(Vector3D(com.x(), com.y(), com.z()), mass_sum);
}

void BridgeBStateAtdog3::configure(
    double crouch_time_param, double step_height_param, double step_forward_distance_param, double bridge_surface_z_param,
    double exit_stance_z_param, double stance_z_offset_param)
{
    crouch_time = crouch_time_param;
    step_height = step_height_param;
    step_forward_distance = step_forward_distance_param;
    bridge_surface_z = bridge_surface_z_param;
    exit_stance_z = exit_stance_z_param;
    stance_z_offset = stance_z_offset_param;
}

void BridgeBStateAtdog3::enter()
{
    robot->lf_leg_calc->set_init_joint_pos(robot->lf_joint_pos);
    robot->rf_leg_calc->set_init_joint_pos(robot->rf_joint_pos);
    robot->lb_leg_calc->set_init_joint_pos(robot->lb_joint_pos);
    robot->rb_leg_calc->set_init_joint_pos(robot->rb_joint_pos);

    robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
    robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
    robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
    robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);

    lf_init_joint_pos = robot->lf_joint_pos;
    rf_init_joint_pos = robot->rf_joint_pos;
    lb_init_joint_pos = robot->lb_joint_pos;
    rb_init_joint_pos = robot->rb_joint_pos;

    bridge_stage = -1;
    bridge_stage_time = std::chrono::steady_clock::now();
    last_update_time = bridge_stage_time;
    total_distance = 0.0;
    bridge_over = false;
    RL_walk_flag = false;
    diagonal_walk_started = false;
    diagonal_walk.reset();
    bridge_pair_phase = 0;
    bridge_pair_phase_active = false;
    bridge_active_move_lf_rb = true;
    bridge_pair_phase_total = std::max(1, bridge_cycle_limit);
    bridge_lateral_targets_initialized = false;
    bridge_lf_target_y = 0.0;
    bridge_rf_target_y = 0.0;
    bridge_lb_target_y = 0.0;
    bridge_rb_target_y = 0.0;

    for (double& value : last_filtered_wheel_vel) {
        value = 0.0;
    }

    Vector3D com_3d;
    std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
    mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());

    reset_targets_to_current_pose();
}

double BridgeBStateAtdog3::low_pass_filter(double input, int wheel_idx)
{
    constexpr double alpha = 0.1;
    const double filtered = alpha * input + (1.0 - alpha) * last_filtered_wheel_vel[wheel_idx];
    last_filtered_wheel_vel[wheel_idx] = filtered;
    return filtered;
}

double BridgeBStateAtdog3::get_elapsed_time() const
{
    const auto now = std::chrono::steady_clock::now();
    const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - bridge_stage_time);
    return duration.count() / 1000.0;
}

void BridgeBStateAtdog3::reset_targets_to_current_pose()
{
    lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

    lf_foot_exp_vel = Vector3D::Zero();
    rf_foot_exp_vel = Vector3D::Zero();
    lb_foot_exp_vel = Vector3D::Zero();
    rb_foot_exp_vel = Vector3D::Zero();

    lf_foot_exp_acc = Vector3D::Zero();
    rf_foot_exp_acc = Vector3D::Zero();
    lb_foot_exp_acc = Vector3D::Zero();
    rb_foot_exp_acc = Vector3D::Zero();

    lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
    rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate);
    lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate);
    rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate);

    lf_wheel_vel = 0.0;
    rf_wheel_vel = 0.0;
    lb_wheel_vel = 0.0;
    rb_wheel_vel = 0.0;

    lf_wheel_force = 0.0;
    rf_wheel_force = 0.0;
    lb_wheel_force = 0.0;
    rb_wheel_force = 0.0;
}

RobotTarget BridgeBStateAtdog3::build_robot_target()
{
    RobotTarget joints_target;
    joints_target.legs[1] = robot->lf_leg_calc->signal_leg_calc(
        lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    joints_target.legs[0] = robot->rf_leg_calc->signal_leg_calc(
        rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    joints_target.legs[3] = robot->lb_leg_calc->signal_leg_calc(
        lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    joints_target.legs[2] = robot->rb_leg_calc->signal_leg_calc(
        rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);

    if (bridge_stage == 2) {
        double blend = 1.0;
        if (bridge_pair_phase_active && bridge_level_shank_blend_time > 1e-6) {
            const double phase_time = std::min(get_elapsed_time() / bridge_level_shank_blend_time, 1.0);
            blend = phase_time * phase_time * (3.0 - 2.0 * phase_time);
        }

        auto enforce_level_shank = [blend](LegTarget& leg) {
            // dog3 的 sagittal 链上，小腿水平约束可写成 q_thigh + q_calf = 0。
            const double level_rad = -leg.joints[1].rad;
            const double level_omega = -leg.joints[1].omega;
            leg.joints[2].rad += (level_rad - leg.joints[2].rad) * blend;
            leg.joints[2].omega += (level_omega - leg.joints[2].omega) * blend;
        };

        if (bridge_active_move_lf_rb) {
            enforce_level_shank(joints_target.legs[1]);
            enforce_level_shank(joints_target.legs[2]);
        } else {
            enforce_level_shank(joints_target.legs[0]);
            enforce_level_shank(joints_target.legs[3]);
        }
    }

    return joints_target;
}

RobotTarget BridgeBStateAtdog3::update()
{
    const auto now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(now - last_update_time).count();
    last_update_time = now;

    reset_targets_to_current_pose();

    const double raw_drive_omega =
        (robot->lf_wheel_omega - robot->rf_wheel_omega + robot->lb_wheel_omega - robot->rb_wheel_omega) / 4.0;
    const double filtered_drive_omega = low_pass_filter(raw_drive_omega, 0);
    total_distance += std::abs(filtered_drive_omega * Robot_t::WHEEL_RADIUS * dt);

    auto switch_stage = [this]() {
        bridge_stage_time = std::chrono::steady_clock::now();
    };
    const double bridge_target_z = bridge_surface_z + stance_z_offset;
    const double exit_target_z = exit_stance_z + stance_z_offset;
    const Vector3D bridge_support_target(0.0, 0.0, bridge_target_z);
    const Vector3D exit_support_target(0.0, 0.0, exit_target_z);
    auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
    auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    lf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lf_grivate);
    rf_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rf_grivate);
    lb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_lb_grivate);
    rb_foot_exp_force = Vector3D(0.0, 0.0, -robot->robot_rb_grivate);

    if (bridge_stage == -1) {
        if (get_elapsed_time() > 0.2) {
            bridge_stage = 0;
            switch_stage();
        }
    } else if (bridge_stage == 0) {

        lf_step.update_support_trajectory(lf_cart_pos, bridge_support_target, crouch_time);
        rf_step.update_support_trajectory(rf_cart_pos, bridge_support_target, crouch_time);
        lb_step.update_support_trajectory(lb_cart_pos, bridge_support_target, crouch_time);
        rb_step.update_support_trajectory(rb_cart_pos, bridge_support_target, crouch_time);

        bridge_stage = 1;
        switch_stage();
    } else if (bridge_stage == 1) {
        bool success = false;
        const double time = get_elapsed_time();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(time, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(time, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(time, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(time, success);

        if (!success) {
            bridge_stage = 2;
            switch_stage();
        }
    } else if (bridge_stage == 2) {
        Vector3D com_3d;
        std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
        mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());

        const double bridge_pitch = bridge_board_width + bridge_board_gap;
        const double half_bridge_pitch = bridge_pitch * 0.45;
        const double move_time = std::max(0.5, gait_step_time);
        const double swing_peak_z = bridge_target_z + bridge_swing_clearance;
        const double support_scrape_z = bridge_target_z - bridge_support_scrape_depth;
        const bool move_lf_rb = (bridge_pair_phase % 2 == 0);

        if (!bridge_lateral_targets_initialized) {
            bridge_lf_target_y = lf_cart_pos.y() + bridge_swing_lateral_offset;
            bridge_lb_target_y = lb_cart_pos.y() + bridge_swing_lateral_offset;
            bridge_rf_target_y = rf_cart_pos.y() - bridge_swing_lateral_offset;
            bridge_rb_target_y = rb_cart_pos.y() - bridge_swing_lateral_offset;
            bridge_lateral_targets_initialized = true;
        }

        auto lock_bridge_support_height = [support_scrape_z](Vector3D& pos, Vector3D& vel, Vector3D& acc) {
            pos.z() = support_scrape_z;
            vel.z() = 0.0;
            acc.z() = 0.0;
        };

        auto split_bridge_support_force = [this](bool lf_support, bool rf_support, bool lb_support, bool rb_support) {
            int support_leg_count = 0;
            support_leg_count += lf_support ? 1 : 0;
            support_leg_count += rf_support ? 1 : 0;
            support_leg_count += lb_support ? 1 : 0;
            support_leg_count += rb_support ? 1 : 0;

            lf_foot_exp_force.setZero();
            rf_foot_exp_force.setZero();
            lb_foot_exp_force.setZero();
            rb_foot_exp_force.setZero();

            if (support_leg_count <= 0) {
                return;
            }

            const double support_force = -mass * 9.8 / static_cast<double>(support_leg_count);
            if (lf_support) {
                lf_foot_exp_force.z() = support_force;
            }
            if (rf_support) {
                rf_foot_exp_force.z() = support_force;
            }
            if (lb_support) {
                lb_foot_exp_force.z() = support_force;
            }
            if (rb_support) {
                rb_foot_exp_force.z() = support_force;
            }
        };

        if (!bridge_pair_phase_active) {
            bridge_active_move_lf_rb = move_lf_rb;
            const Vector3D lf_support_target(-half_bridge_pitch, lf_cart_pos.y(), support_scrape_z);
            const Vector3D rf_support_target(-half_bridge_pitch, rf_cart_pos.y(), support_scrape_z);
            const Vector3D lb_support_target(-half_bridge_pitch, lb_cart_pos.y(), support_scrape_z);
            const Vector3D rb_support_target(-half_bridge_pitch, rb_cart_pos.y(), support_scrape_z);
            const Vector3D lf_flight_target(half_bridge_pitch, bridge_lf_target_y, bridge_target_z);
            const Vector3D rf_flight_target(half_bridge_pitch, bridge_rf_target_y, bridge_target_z);
            const Vector3D lb_flight_target(half_bridge_pitch, bridge_lb_target_y, bridge_target_z);
            const Vector3D rb_flight_target(half_bridge_pitch, bridge_rb_target_y, bridge_target_z);

            if (move_lf_rb) {
                lf_step.update_flight_trajectory(
                    lf_cart_pos, Vector3D::Zero(), lf_flight_target, Vector3D::Zero(), move_time, swing_peak_z);
                rb_step.update_flight_trajectory(
                    rb_cart_pos, Vector3D::Zero(), rb_flight_target, Vector3D::Zero(), move_time, swing_peak_z);
                rf_step.update_support_trajectory(rf_cart_pos, rf_support_target, move_time);
                lb_step.update_support_trajectory(lb_cart_pos, lb_support_target, move_time);
            } else {
                rf_step.update_flight_trajectory(
                    rf_cart_pos, Vector3D::Zero(), rf_flight_target, Vector3D::Zero(), move_time, swing_peak_z);
                lb_step.update_flight_trajectory(
                    lb_cart_pos, Vector3D::Zero(), lb_flight_target, Vector3D::Zero(), move_time, swing_peak_z);
                lf_step.update_support_trajectory(lf_cart_pos, lf_support_target, move_time);
                rb_step.update_support_trajectory(rb_cart_pos, rb_support_target, move_time);
            }

            bridge_pair_phase_active = true;
            switch_stage();
        }

        bool success = false;
        const double time = get_elapsed_time();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(time, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(time, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(time, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(time, success);

        if (move_lf_rb) {
            lock_bridge_support_height(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc);
            lock_bridge_support_height(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc);
        } else {
            lock_bridge_support_height(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc);
            lock_bridge_support_height(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc);
        }

        if (move_lf_rb) {
            split_bridge_support_force(false, true, true, false);
        } else {
            split_bridge_support_force(true, false, false, true);
        }

        if (!success) {
            bridge_pair_phase_active = false;
            bridge_pair_phase++;

            if (bridge_pair_phase >= bridge_pair_phase_total) {
                diagonal_walk_started = false;
                diagonal_walk.reset();
                bridge_stage = 3;
                switch_stage();
                lf_step.update_support_trajectory(lf_foot_exp_pos, exit_support_target, stand_time);
                rf_step.update_support_trajectory(rf_foot_exp_pos, exit_support_target, stand_time);
                lb_step.update_support_trajectory(lb_foot_exp_pos, exit_support_target, stand_time);
                rb_step.update_support_trajectory(rb_foot_exp_pos, exit_support_target, stand_time);
            }
        }
    } else if (bridge_stage == 3) {
        bool success = false;
        const double time = get_elapsed_time();
        std::tie(lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc) = lf_step.get_target(time, success);
        std::tie(rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc) = rf_step.get_target(time, success);
        std::tie(lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc) = lb_step.get_target(time, success);
        std::tie(rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc) = rb_step.get_target(time, success);

        if (!success) {
            bridge_stage = 4;
            bridge_over = true;
            switch_stage();
        }
    }

    std::cout << "bridge_stage:" << bridge_stage << std::endl;
    return build_robot_target();
}
