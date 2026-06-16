#include "nodes/place_box.hpp"
#include "core/robot.hpp"
#include "nodes/msg.hpp"
#include <rclcpp/logging.hpp>
#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <atomic>

using namespace std::chrono_literals;

namespace {

/// 等待行为树进入指定阶段
/// 比如当前 PlaceBoxAction 要等到 Robot::kTreePlaceBox 才开始执行
bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ROS正常运行 且 自动模式仍开启，返回true
    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

/// 等待指定时间，但允许中途被打断
/// 如果自动模式关闭或者节点退出，会立即返回 false
bool wait_with_interrupt(Robot* context, const std::chrono::milliseconds duration) {
    auto remaining = duration;

    while (remaining.count() > 0) {

        // 如果程序退出或自动模式关闭，中断等待
        if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
            return false;
        }

        // 每50ms检查一次是否需要中断
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


/// PlaceBoxAction 构造函数
/// 注册一个行为树动作节点，名字叫 place_box_action
PlaceBoxAction::PlaceBoxAction()
    : BT::ActionNode("place_box_action") {

    
}

void PlaceBoxAction::arm_place_cmd_callback(
    const robot_msgs::msg::Armmode::SharedPtr msg)
{
    arm_state_ = msg->mode;
}


/// 行为树执行到这个节点时会调用 execute()
BT::Status PlaceBoxAction::execute(BT& tree) {

    // 1. 获取机器人上下文（共享控制对象）
    auto* context = tree.get_context<Robot>();

    if (!context) {
        return BT::FAILED;
    }


    RCLCPP_INFO(
        context->node_->get_logger(),
        "等待机械臂放置完成");

    //  // 初始化话题通信
    // place_pos_up_pub   = context->node_->template create_publisher<robot_msgs::msg::Vis>("place_position_up", 10);
    // place_pos_down_pub = context->node_->template create_publisher<robot_msgs::msg::Vis>("place_position_down", 10);
    // arm_cmd_pub_       = context->node_->create_publisher<robot_msgs::msg::Armmode>("arm_cmd", 10);
    // arm_state_sub =
    //         context->node_->create_subscription<
    //             robot_msgs::msg::Armmode>(
    //                 "arm_cmd_place_state",
    //                 10,
    //                 std::bind(
    //                     &PlaceBoxAction::arm_place_cmd_callback,
    //                     this,
    //                     std::placeholders::_1));


    // // 2. 等待进入“放箱子阶段”
    if (!wait_for_stage(context, Robot::kTreePlaceBox)) {
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

    // const std::array<float, 2>* target_pos = nullptr;


    // // 保存当前狗的位置位置 (x, y)
    // dst_box_pos_ = plan.dst_box_pos;

    // // 保存是否放到第二层
    // place_at_second_floor_ = plan.place_at_second_floor;

    // // ===============================
    // // 找离当前狗身位置最近的固定放置点
    // // ===============================

    // const std::array<float, 2>* fixed_points[] = {&kBox1Pos, &kBox2Pos, &kBox3Pos, &kBox4Pos};

    // // 默认最近的是第一个
    // const std::array<float, 2>* nearest_pos = fixed_points[0];
    // float min_dist                          = distance_sq(dst_box_pos_, *nearest_pos);

    // // 遍历找最近点
    // for (int i = 1; i < 4; ++i) {
    //     float dist = distance_sq(dst_box_pos_, *fixed_points[i]);

    //     if (dist < min_dist) {
    //         min_dist    = dist;
    //         nearest_pos = fixed_points[i];
    //     }
    // }

    // // 当前目标放置偏移（相对狗身）
    // std::array<float, 2> relative_target = {(*nearest_pos)[0] - dst_box_pos_[0], (*nearest_pos)[1] - dst_box_pos_[1]};

    // if (!place_at_second_floor_) {
    //     relative_target[0] = relative_target[0] + 0.0f;  
    //     relative_target[1] = relative_target[1] + 0.0f;

    //     robot_msgs::msg::Vis msg_Vis;
    //     msg_Vis.x = relative_target[0];
    //     msg_Vis.y = relative_target[1];
    //     place_pos_down_pub->publish(msg_Vis);

    //     robot_msgs::msg::Armmode msg_Armmode;
    //     msg_Armmode.mode = 2;
    //     arm_cmd_pub_->publish(msg_Armmode);

    // } else if (place_at_second_floor_) {
    //     relative_target[0] = relative_target[0] + 0.0f;  
    //     relative_target[1] = relative_target[1] + 0.0f;

    //     robot_msgs::msg::Vis msg_Vis;
    //     msg_Vis.x = relative_target[0];
    //     msg_Vis.y = relative_target[1];
    //     place_pos_up_pub->publish(msg_Vis);

    //     robot_msgs::msg::Armmode msg_Armmode;
    //     msg_Armmode.mode = 2;
    //     arm_cmd_pub_->publish(msg_Armmode);

    // } 

    // auto start = std::chrono::steady_clock::now();

    // while (rclcpp::ok()) {

    //     // 成功
    //     if (arm_state_ == 1) {

    //         RCLCPP_INFO(
    //             context->node_->get_logger(),
    //             "放置完成");

    //         arm_state_ = 0;

    //         break;
    //     }


    //     // 超时
    //     if (std::chrono::steady_clock::now() - start
    //         > 20s)
    //     {
    //         RCLCPP_ERROR(
    //             context->node_->get_logger(),
    //             "机械臂任务超时");

    //         return BT::FAILED;
    //     }

    //     std::this_thread::sleep_for(10ms);

    // }



    // // 打印提取结果，方便调试
    // RCLCPP_INFO(
    //     context->node_->get_logger(), "提取目标位置=(%.2f, %.2f), second_floor=%s", dst_box_pos_[0], dst_box_pos_[1],
    //     place_at_second_floor_ ? "true" : "false");


    // 9. 打印当前放置任务信息
    // RCLCPP_INFO(
    //     context->node_->get_logger(), "PlaceBoxAction: 执行放置轨迹，轨迹点数量=%zu，目标位置=(%.2f, %.2f)", plan.place_trajectory.size(),
    //     plan.dst_box_pos[0], plan.dst_box_pos[1]);


    // 10. 等待放置动作完成（最长10秒）
    if (!wait_with_interrupt(context, std::chrono::seconds(6))) {
        //return BT::FAILED;
    }

        // 11. 如果还有下一个搬箱计划
    if (plan_index + 1 < static_cast<int>(move_plan.size())) {

        // 更新索引，准备下次执行下一条计划
        tree.write_msg("plan_index", plan_index + 1);
    

    } else {

        // 所有计划执行完成
        RCLCPP_INFO(
            context->node_->get_logger(),
            "PlaceBoxAction: 全部搬箱计划执行完成"
        );
    }

    // 12. 如果不是调试模式，推进到下一行为树阶段
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }

    // 13. 返回执行成功
    return BT::SUCCESS;
    RCLCPP_INFO(
        context->node_->get_logger(),
        "机械臂放置完成");
}