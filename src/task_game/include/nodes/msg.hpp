#pragma once

#include <array>
#include <vector>
#include <Eigen/Dense>

struct TargetPoint {
    bool constraint_target_yaw{false};
    float target_vel{0.0f};
    float max_velocity{0.7f};
    float max_accelation{0.25f};
    float max_omega{1.0f};
    Eigen::Vector3d kp{0.2, 0.2, 0.5};
    float allow_start_dir_error{0.2f};
    float allow_final_dir_error{0.2f};
    float allow_final_pos_allow{0.2f};
    float adjust_min_vel{0.25f};
    float adjust_min_omega{0.15f};
    bool allow_y_vel{false};
    float trajectory_connection_radius{0.0f};
};

struct TrajectoryPlan {
    std::vector<std::array<float, 3>> trajectory;
    std::vector<TargetPoint> target_points;
};

struct BoxMoveTask {
    TrajectoryPlan to_box;
    TrajectoryPlan to_dst;
    std::array<float, 2> pick_box_pos{};
    std::array<float, 2> place_box_pos{};
    bool place_at_second_floor{false};
    int box_id{-1};
    int line{-1};
    int col{-1};
};

struct MoveBoxPlan {
    BoxMoveTask box0;
    BoxMoveTask box1;
    std::array<float, 3> dst2_pos{};
    TargetPoint dst0_to_dst2;
};

enum class BoxSlot {
    Box0,
    Box1,
};
