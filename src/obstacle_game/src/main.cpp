#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "core/robot.hpp"


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("robot_calc_node");
    auto robot_calc=std::make_shared<Robot>(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
