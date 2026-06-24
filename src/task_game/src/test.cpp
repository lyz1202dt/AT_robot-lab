#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node {
public:
    TestPublisher()
        : Node("test_publisher") {
        grid_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("box_id_grid", 10);
        timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });
    }

private:
    void publish_once() {
        // 测试节点只发布普通箱子矩阵，规划节点会按双箱流程生成搬运计划。
        auto grid_msg = std_msgs::msg::Int32MultiArray();
        grid_msg.data = {
            3, 2, 1, 0,
            0, 1, 2, 3
        };
        grid_pub_->publish(grid_msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
