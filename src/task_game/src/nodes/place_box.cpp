#include "nodes/place_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

using namespace std::chrono_literals;

namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() &&
           context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

bool wait_with_interrupt(Robot* context, const std::chrono::milliseconds duration) {
    auto remaining = duration;

    while (remaining.count() > 0) {
        if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
            return false;
        }

        const auto step = std::min(remaining, std::chrono::milliseconds(50));
        std::this_thread::sleep_for(step);

        remaining -= step;
    }

    return true;
}



} // namespace


PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {}

void PlaceBoxAction::arm_place_cmd_callback(
    const robot_msgs::msg::Armmode::SharedPtr msg)
{
    arm_state_ = msg->mode;
}


BT::Status PlaceBoxAction::execute(BT& tree) {

    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂放置完成");

   
    arm_cmd_pub_ =
        context->node_->create_publisher<robot_msgs::msg::Armmode>("arm_cmd", 10);

    arm_state_sub =
        context->node_->create_subscription<robot_msgs::msg::Armmode>(
            "arm_cmd_place_state",
            10,
            std::bind(&PlaceBoxAction::arm_place_cmd_callback,
                      this,
                      std::placeholders::_1));

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

    dst_box_pos_ = plan.dst_box_pos;
    place_at_second_floor_ = plan.place_at_second_floor;

   

    // if (!place_at_second_floor_) {
      
    //      robot_msgs::msg::Armmode msg_Armmode;
    // msg_Armmode.mode = 2;
    // arm_cmd_pub_->publish(msg_Armmode);
    // } else {
      
    //     robot_msgs::msg::Armmode msg_Armmode;
    // msg_Armmode.mode = 3;
    // arm_cmd_pub_->publish(msg_Armmode);
    // }

    

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
            return BT::SUCCESS;
        }

        std::this_thread::sleep_for(10ms);
    }

    std::this_thread::sleep_for(6s);

    if (plan_index + 1 < static_cast<int>(move_plan.size())) {
        tree.write_msg("plan_index", plan_index + 1);
    } else {
        RCLCPP_INFO(context->node_->get_logger(),
            "PlaceBoxAction: 全部搬箱计划执行完成");
    }

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    return BT::SUCCESS;
}