#include <core/robot.hpp>
#include <robot_msgs/msg/detail/cmd__struct.hpp>
#include <robot_msgs/msg/detail/remote__struct.hpp>
#include <chrono>

using namespace std::chrono_literals;


Robot::Robot(const std::shared_ptr<rclcpp::Node> node):node_(node)
{
    node_->declare_parameter<std::string>("scene_name");

    //机器人底层控制指令发布
    cmd_pub_=node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    //机器人遥控器指令订阅
    remote_sub_=node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg){
        //TODO:处理并发布遥控器数据
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            //TODO:处理参数更新
        }
        return result;
    });

    control_timer=node_->create_wall_timer(50ms, [this](){
        
    });
}

