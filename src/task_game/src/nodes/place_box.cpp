#include "nodes/place_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <atomic>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

Pilot::TargetPoint make_retreat_target(const MoveBoxPlan& plan)
{
    Pilot::TargetPoint target;
    target.target_pos = Eigen::Vector2d(plan.dst2_pos[0], plan.dst2_pos[1]);
    target.target_yaw = plan.dst2_pos[2];
    target.constraint_target_yaw = plan.dst_to_dst2.constraint_target_yaw;
    target.target_vel = plan.dst_to_dst2.target_vel;
    target.max_velocity = plan.dst_to_dst2.max_velocity;
    target.max_accelation = plan.dst_to_dst2.max_accelation;
    target.max_omega = plan.dst_to_dst2.max_omega;
    target.kp = plan.dst_to_dst2.kp;
    target.allow_start_dir_error = plan.dst_to_dst2.allow_start_dir_error;
    target.allow_final_dir_error = plan.dst_to_dst2.allow_final_dir_error;
    target.allow_final_pos_allow = plan.dst_to_dst2.allow_final_pos_allow;
    target.adjust_min_vel = plan.dst_to_dst2.adjust_min_vel;
    target.adjust_min_omega = plan.dst_to_dst2.adjust_min_omega;
    target.allow_y_vel = plan.dst_to_dst2.allow_y_vel;
    target.trajectory_connection_radius = 0.0f;
    return target;
}

bool retreat_to_dst2(Robot* context, const MoveBoxPlan& plan)
{
    const auto target = make_retreat_target(plan);
    if (!context->pilot->set_target(target)) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 设置退让目标失败");
        return false;
    }

    bool finished = false;
    bool success = false;
    if (!context->pilot->start([&finished, &success](int result) {
            success = (result != 0);
            finished = true;
        }, true)) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 启动退让到 dst2 失败");
        return false;
    }

    RCLCPP_INFO(
        context->node_->get_logger(),
        "PlaceBoxAction: 全部搬箱完成，退让到 dst2=(%.2f, %.2f, %.2f)",
        plan.dst2_pos[0],
        plan.dst2_pos[1],
        plan.dst2_pos[2]);

    while (rclcpp::ok() && context->auto_pilot_enabled.load() && !finished) {
        std::this_thread::sleep_for(50ms);
    }

    if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
        context->pilot->stop();
        return false;
    }

    if (!success) {
        RCLCPP_ERROR(context->node_->get_logger(), "PlaceBoxAction: 退让到 dst2 失败");
        context->pilot->stop();
        return false;
    }

    context->enter_manual_mode();
    RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 已退让到 dst2 并切入手动模式");
    return true;
}

} // namespace


PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}

void PlaceBoxAction::arm_place_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) { arm_state_ = msg->data; }


BT::Status PlaceBoxAction::execute(BT& tree) {

    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    arm_state_ = 0;

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂放置完成");

    if (!subscriptions_ready_) {
        arm_cmd_pub_ = context->node_->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);

        arm_state_sub = context->node_->create_subscription<std_msgs::msg::Int32>(
            "arm_cmd_state", 10, std::bind(&PlaceBoxAction::arm_place_cmd_callback, this, std::placeholders::_1));
        
        subscriptions_ready_ = true;
    }

    if (!wait_for_stage(context, Robot::kTreePlaceBox)) {
        context->pilot->stop();
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;

    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }

    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    // =========================================================
    // 8. 当前计划
    // =========================================================
    const auto& plan = move_plan[plan_index];
    place_at_second_floor_ = plan.place_at_second_floor;

    if (!place_at_second_floor_) {
        std_msgs::msg::Int32 msg;
        msg.data = 2;
        arm_cmd_pub_->publish(msg);
    } else {

        std_msgs::msg::Int32 msg;
        msg.data = 3;
        arm_cmd_pub_->publish(msg);
    }
    // =========================================================
    // 等待执行结果
    // =========================================================
    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {

        if (arm_state_ == 1) {
            RCLCPP_INFO(context->node_->get_logger(), "放置完成");
            arm_state_ = 0;
            break;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");

            if (plan_index + 1 < static_cast<int>(move_plan.size())) {
                tree.write_msg("plan_index", plan_index + 1);
            } else {
                RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 全部搬箱计划执行完成");
                if (!retreat_to_dst2(context, plan)) {
                    return BT::FAILED;
                }
            }

            if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
                context->advance_tree_stage();
            }

            return BT::SUCCESS;
        }

        std::this_thread::sleep_for(10ms);
    }

    // std::this_thread::sleep_for(6s);

    if (plan_index + 1 < static_cast<int>(move_plan.size())) {
        tree.write_msg("plan_index", plan_index + 1);
    } else {
        RCLCPP_INFO(context->node_->get_logger(), "PlaceBoxAction: 全部搬箱计划执行完成");
        if (!retreat_to_dst2(context, plan)) {
            return BT::FAILED;
        }
    }

    if (!context->is_tree_debug_mode() && context->auto_pilot_enabled.load()) {
        context->advance_tree_stage();
    }

    return BT::SUCCESS;
}
