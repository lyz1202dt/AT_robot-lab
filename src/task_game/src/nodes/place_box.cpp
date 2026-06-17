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

float distance_sq(const std::array<float, 2>& a, const std::array<float, 2>& b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    return dx * dx + dy * dy;
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

    place_pos_up_pub =
        context->node_->template create_publisher<robot_msgs::msg::Vis>("place_position_up", 10);
    place_pos_down_pub =
        context->node_->template create_publisher<robot_msgs::msg::Vis>("place_position_down", 10);
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

    // =========================================================
    // ✔ TF 获取：map -> arm_base_link
    // =========================================================
    geometry_msgs::msg::TransformStamped tf_msg;

    try {
        tf_msg = context->tf_buffer_->lookupTransform(
            "map",
            "arm_base_link",
            tf2::TimePointZero,
            tf2::durationFromSec(0.05));
    }
    catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(context->node_->get_logger(),
            "TF获取失败(map->arm_base_link): %s", ex.what());
        return BT::FAILED;
    }

    // =========================================================
    // ✔ 直接得到“狗/机器人全局位置”
    // =========================================================
    std::array<float, 2> dog_global_pos = {
        static_cast<float>(tf_msg.transform.translation.x),
        static_cast<float>(tf_msg.transform.translation.y)
    };

    // =========================================================
    // ✔ 核心修正：全局目标 - 机器人全局位置
    // =========================================================
    std::array<float, 2> relative_target = {
        dst_box_pos_[0] - dog_global_pos[0],
        dst_box_pos_[1] - dog_global_pos[1]
    };

    robot_msgs::msg::Vis msg_Vis;
    msg_Vis.x = relative_target[0];
    msg_Vis.y = relative_target[1];

   

    if (!place_at_second_floor_) {
        place_pos_down_pub->publish(msg_Vis);
         robot_msgs::msg::Armmode msg_Armmode;
    msg_Armmode.mode = 2;
    arm_cmd_pub_->publish(msg_Armmode);
    } else {
        place_pos_up_pub->publish(msg_Vis);
        robot_msgs::msg::Armmode msg_Armmode;
    msg_Armmode.mode = 3;
    arm_cmd_pub_->publish(msg_Armmode);
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
            return BT::FAILED;
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