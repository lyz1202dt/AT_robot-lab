#include "nodes/catch_box.hpp"

#include "core/robot.hpp"
#include "nodes/msg.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;
namespace {

// 机械臂协议：box0 抓完放平板，box1 抓完保持吸在机械臂上。
constexpr int kArmCatchToPlate = 1;
constexpr int kArmCatchToHand = 2;

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

int32_t stage_for_slot(BoxSlot slot) {
    return slot == BoxSlot::Box0 ? Robot::kTreeCatchBox0 : Robot::kTreeCatchBox1;
}

int arm_command_for_slot(BoxSlot slot) {
    return slot == BoxSlot::Box0 ? kArmCatchToPlate : kArmCatchToHand;
}

const char* slot_name(BoxSlot slot) {
    return slot == BoxSlot::Box0 ? "box0" : "box1";
}

} // namespace

CatchBoxAction::CatchBoxAction(BoxSlot slot)
    : BT::ActionNode(slot == BoxSlot::Box0 ? "catch_box0_action" : "catch_box1_action"), slot_(slot) {}

void CatchBoxAction::arm_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("logger"), "接收到反馈");
    arm_state_ = msg->data;
}

BT::Status CatchBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }

    arm_state_ = 0;

    if (!subscriptions_ready_) {
        arm_cmd_pub_ = context->node_->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);

        arm_state_sub_ = context->node_->create_subscription<std_msgs::msg::Int32>(
            "arm_cmd_state", 10, std::bind(&CatchBoxAction::arm_cmd_callback, this, std::placeholders::_1));

        subscriptions_ready_ = true;
    }

    if (!wait_for_stage(context, stage_for_slot(slot_))) {
        context->pilot->stop();
        return BT::FAILED;
    }

    std::vector<MoveBoxPlan> move_plan;
    int plan_index = 0;

    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }

    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }

    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }

    std_msgs::msg::Int32 msg;
    msg.data = arm_command_for_slot(slot_);
    arm_cmd_pub_->publish(msg);

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂抓取 %s 完成，arm_cmd=%d", slot_name(slot_), msg.data);

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        if (arm_state_ == 1) {
            RCLCPP_INFO(context->node_->get_logger(), "抓取 %s 成功", slot_name(slot_));
            arm_state_ = 0;
            if (!context->is_tree_debug_mode()) {
                context->advance_tree_stage();
            }
            return BT::SUCCESS;
        }

        if (arm_state_ == -1) {
            RCLCPP_ERROR(context->node_->get_logger(), "抓取 %s 失败", slot_name(slot_));
            arm_state_ = 0;
            if (!context->is_tree_debug_mode()) {
                context->advance_tree_stage();
            }
            return BT::SUCCESS;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");
            if (!context->is_tree_debug_mode()) {
                context->advance_tree_stage();
            }
            return BT::SUCCESS;
        }

        std::this_thread::sleep_for(5ms);
    }

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    RCLCPP_INFO(context->node_->get_logger(), "机械臂抓取 %s 完成", slot_name(slot_));
    return BT::SUCCESS;
}
