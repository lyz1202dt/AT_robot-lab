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

    // 以下为激光重规划标记：当 box0 平板放置失败（平板被卡死）时，
    // 剩余箱子改为“单吸到手上再从手上放置”，并在最后回头重试卡住的平板箱。
    // replan_after_box1_place：box1 放置后直接按 box0 平板失败重规划，跳过 box0 放置链。
    bool replan_after_box1_place{true};
    // hand_only_plan：本轮是单吸手放计划，真实箱子放在 box1 槽，box0 槽被跳过。
    bool hand_only_plan{false};
    // plate_retry_plan：本轮是“最后回头重试卡住的平板箱”计划，只走 box0 平板放置。
    bool plate_retry_plan{false};
    // finish_after_box0_place：本轮 box0 放完后直接结束行为树（退让 dst2 并进手动）。
    bool finish_after_box0_place{false};
};

enum class BoxSlot {
    Box0,
    Box1,
};
