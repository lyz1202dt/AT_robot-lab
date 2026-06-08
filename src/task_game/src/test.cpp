#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_msgs/msg/box_id_grid.hpp>
#include <robot_msgs/msg/int.hpp>

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node {
public:
    TestPublisher()
        : Node("test_publisher") {
        vip_pub_ = this->create_publisher<robot_msgs::msg::Int>("vip_box_id", 10);
        grid_pub_ = this->create_publisher<robot_msgs::msg::BoxIdGrid>("box_id_grid", 10);

        timer_ = this->create_wall_timer(1s, [this]() { publish_once(); });
    }

private:
    void publish_once() {
        //timer_->cancel();

        // --- 发布 vip_box_id ---
        auto vip_msg = robot_msgs::msg::Int();
        vip_msg.data = 2;  // VIP 箱子 ID
        vip_pub_->publish(vip_msg);
        RCLCPP_INFO(this->get_logger(), "发布 vip_box_id: %d", vip_msg.data);

        // --- 发布 box_id_grid (2行×4列, 展平为 int32[8]) ---
        auto grid_msg = robot_msgs::msg::BoxIdGrid();
        // 示例：两排各4个箱子, data[0~3] 第一排, data[4~7] 第二排
        // 同一个 ID 可出现多次（如两个箱子属于同一 ID 的放置列）
        grid_msg.data = {
            0, 1, 3, 2,   // 第一排: box_id = 0,1,3,2
            2, 0, 1, 3    // 第二排: box_id = 2,0,1,3
        };
        grid_pub_->publish(grid_msg);
        RCLCPP_INFO(this->get_logger(), "发布 box_id_grid");
    }

    rclcpp::Publisher<robot_msgs::msg::Int>::SharedPtr vip_pub_;
    rclcpp::Publisher<robot_msgs::msg::BoxIdGrid>::SharedPtr grid_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}