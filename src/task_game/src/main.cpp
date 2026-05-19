#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <core/robot.hpp>


int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("robot_calc_node");
    Robot robot_calc(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
