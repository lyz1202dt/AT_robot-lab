#include "nodes/catch_box.hpp"

#include "core/robot.hpp"
#include "nodes/msg.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

using namespace std::chrono_literals;
namespace {

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

} // namespace

CatchBoxAction::CatchBoxAction()
    : BT::ActionNode("catch_box_action") {}

void CatchBoxAction::arm_cmd_callback(const robot_msgs::msg::Armmode::SharedPtr msg) { RCLCPP_INFO(rclcpp::get_logger("logger"),"接收到反馈");
    arm_state_ = msg->mode; }

BT::Status CatchBoxAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }

    if (!subscriptions_ready_) {
        
        arm_cmd_pub_ = context->node_->create_publisher<robot_msgs::msg::Armmode>("arm_cmd", 10);

        arm_state_sub_ = context->node_->create_subscription<robot_msgs::msg::Armmode>(
            "arm_cmd_state", 10, std::bind(&CatchBoxAction::arm_cmd_callback, this, std::placeholders::_1));
    
        subscriptions_ready_ = true;
    }

        if (!wait_for_stage(context, Robot::kTreeCatchBox)) {
            context->pilot->stop();
            return BT::FAILED;
        }
    


    // 4. 准备读取搬箱计划
    std::vector<MoveBoxPlan> move_plan;

    // 当前执行到第几个搬箱计划
    int plan_index = 0;


    // 5. 从行为树黑板读取 move_plan
    if (!tree.read_msg("move_plan", move_plan)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 move_plan");
        return BT::FAILED;
    }


    // 6. 从行为树黑板读取当前执行索引
    if (!tree.read_msg("plan_index", plan_index)) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: 缺少 plan_index");
        return BT::FAILED;
    }


    // 7. 检查索引是否合法，防止数组越界
    if (plan_index < 0 || plan_index >= static_cast<int>(move_plan.size())) {
        RCLCPP_WARN(context->node_->get_logger(), "PlaceBoxAction: plan_index=%d 越界", plan_index);
        return BT::FAILED;
    }


    // 8. 取出当前要执行的搬箱计划
    const auto& plan = move_plan[plan_index];

    // 发送抓取命令
    robot_msgs::msg::Armmode msg;
    msg.mode = 1;

    arm_cmd_pub_->publish(msg);


    RCLCPP_INFO(context->node_->get_logger(), "等待机械臂抓取完成");

    auto start = std::chrono::steady_clock::now();

    while (rclcpp::ok()) {
        

        // 成功
        if (arm_state_ == 1) {

            RCLCPP_INFO(context->node_->get_logger(), "抓取成功");

            arm_state_ = 0;
    if (!context->is_tree_debug_mode()) {
            context->advance_tree_stage();
        }
            return BT::SUCCESS;
        }

        // 失败
        if (arm_state_ == -1) {

            RCLCPP_ERROR(context->node_->get_logger(), "抓取失败");

            arm_state_ = 0;
    if (!context->is_tree_debug_mode()) {
            context->advance_tree_stage();
        }
            return BT::SUCCESS;
        }

        // 超时
        if (std::chrono::steady_clock::now() - start > 20s) {
            RCLCPP_ERROR(context->node_->get_logger(), "机械臂任务超时");
                if (!context->is_tree_debug_mode()) {
            context->advance_tree_stage();
        }
            return BT::SUCCESS;
        }

        std::this_thread::sleep_for(5ms);
    }



    // 10. 等待放置动作完成（最长10秒）
    // std::this_thread::sleep_for(6s);



    RCLCPP_INFO(context->node_->get_logger(), "机械臂抓取完成");
    return BT::SUCCESS;
}