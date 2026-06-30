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

// 取某槽位在计划里的默认 box_id（来自 box_id_grid），用作 pnp 失败时的兜底。
int task_for_slot_id(const MoveBoxPlan& plan, BoxSlot slot) {
    return slot == BoxSlot::Box0 ? plan.box0.box_id : plan.box1.box_id;
}

} // namespace

CatchBoxAction::CatchBoxAction(BoxSlot slot)
    : BT::ActionNode(slot == BoxSlot::Box0 ? "catch_box0_action" : "catch_box1_action"), slot_(slot) {}

void CatchBoxAction::arm_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("logger"), "接收到反馈");
    arm_state_ = msg->data;
}

void CatchBoxAction::pnp_box_index_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    // first-wins：只锁存抓取窗口内的首条 ID，避免上一个箱子的 ID 串进来。
    std::lock_guard<std::mutex> lock(pnp_mutex_);
    if (!pnp_latched_) {
        pnp_value_ = msg->data;
        pnp_latched_ = true;
    }
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

        // 抓取时通过 pnp_box_index 得到被抓箱子的放置区 ID。
        pnp_box_index_sub_ = context->node_->create_subscription<std_msgs::msg::Int32>(
            "pnp_box_index", 10, std::bind(&CatchBoxAction::pnp_box_index_callback, this, std::placeholders::_1));

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

    // 激光重规划计划：单吸计划的 box0 抓取、以及末轮平板重试的抓取都跳过。
    const auto& current_plan = move_plan[plan_index];
    if (current_plan.plate_retry_plan || (slot_ == BoxSlot::Box0 && current_plan.hand_only_plan)) {
        if (!context->is_tree_debug_mode()) {
            context->advance_tree_stage();
        }
        RCLCPP_INFO(context->node_->get_logger(), "CatchBoxAction: 重规划计划跳过 %s 抓取阶段", slot_name(slot_));
        return BT::SUCCESS;
    }

    // 抓取开始前清空 pnp 锁存，确保收到的是本次抓取窗口内的 ID。
    {
        std::lock_guard<std::mutex> lock(pnp_mutex_);
        pnp_value_ = -1;
        pnp_latched_ = false;
    }

    // 机械臂抓取指令照常发送，与是否收到 pnp_box_index 无关。
    std_msgs::msg::Int32 msg;
    msg.data = arm_command_for_slot(slot_);
    arm_cmd_pub_->publish(msg);

    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂抓取 %s 完成，arm_cmd=%d", slot_name(slot_), msg.data);

    auto start = std::chrono::steady_clock::now();
    bool arm_done = false;

    while (rclcpp::ok()) {
        if (arm_state_ == 1) {
            RCLCPP_INFO(context->node_->get_logger(), "抓取 %s 成功", slot_name(slot_));
            arm_state_ = 0;
            arm_done = true;
            break;
        }

        if (arm_state_ == -1) {
            RCLCPP_ERROR(context->node_->get_logger(), "抓取 %s 失败", slot_name(slot_));
            arm_state_ = 0;
            arm_done = true;
            break;
        }

        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");
            arm_done = true;
            break;
        }

        std::this_thread::sleep_for(5ms);
    }

    if (!rclcpp::ok() || !arm_done) {
        if (!context->is_tree_debug_mode()) {
            context->advance_tree_stage();
        }
        return BT::SUCCESS;
    }

    // 解析被抓箱子的放置区 ID：优先用 pnp_box_index，识别失败回退到 box_id_grid 默认值。
    int resolved_id = task_for_slot_id(current_plan, slot_);
    {
        std::lock_guard<std::mutex> lock(pnp_mutex_);
        if (pnp_latched_ && pnp_value_ >= 0 && pnp_value_ < 4) {
            resolved_id = pnp_value_;
            RCLCPP_INFO(context->node_->get_logger(), "CatchBoxAction: %s 由 pnp_box_index 得到放置区 ID=%d", slot_name(slot_), resolved_id);
        } else {
            RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: %s 未收到有效 pnp_box_index，回退到 grid 默认 ID=%d", slot_name(slot_), resolved_id);
        }
    }

    // 用解析到的 ID 查表，patch 黑板 move_plan：放置导航位末点 + 机械臂放置位 + box_id。
    std::array<std::array<float, 3>, 4> place_table{};
    std::array<std::array<float, 2>, 4> arm_place_table{};
    if (!tree.read_msg("place_table", place_table) || !tree.read_msg("arm_place_table", arm_place_table)) {
        RCLCPP_WARN(context->node_->get_logger(), "CatchBoxAction: 缺少 place_table/arm_place_table 查表");
    } else if (resolved_id >= 0 && resolved_id < 4) {
        BoxMoveTask& task = (slot_ == BoxSlot::Box0) ? move_plan[plan_index].box0 : move_plan[plan_index].box1;
        task.box_id = resolved_id;
        task.place_box_pos = arm_place_table[resolved_id];
        if (!task.to_dst.trajectory.empty()) {
            // 覆盖放置点导航位（最后一个轨迹点），target_points 不变。
            task.to_dst.trajectory.back() = place_table[resolved_id];
        }
        tree.write_msg("move_plan", move_plan);
    }

    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    RCLCPP_INFO(context->node_->get_logger(), "机械臂抓取 %s 完成", slot_name(slot_));
    return BT::SUCCESS;
}
