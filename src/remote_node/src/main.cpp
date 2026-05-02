#include <rclcpp/rclcpp.hpp>
#include "remote_node/remote_node.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<RemoteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}