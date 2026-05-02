#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <serial/serial.h>
#include <robot_msgs/msg/cmd.hpp>
#include <thread>
#include <atomic>
#include <mutex>
#include "remote_node/remote_comm.hpp"

// 远程控制器数据包命令类型
constexpr uint8_t CMD_REMOTE_CONTROL = 0x01;  // 遥控器数据包命令

class RemoteNode : public rclcpp::Node
{
public:
    RemoteNode();
    ~RemoteNode();

private:
    // ROS2 Publishers
    rclcpp::Publisher<robot_msgs::msg::Cmd>::SharedPtr move_cmd_pub;

    // 串口通信
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<std::thread> serial_recv_thread_;
    std::unique_ptr<std::thread> watchdog_thread_;  // 看门狗线程
    
    // 通信协议处理器
    std::unique_ptr<RemoteComm> remote_comm_;

    // 回调ID（用于取消注册）
    uint32_t remote_control_cb_id_;
    
    // 看门狗相关
    std::atomic<uint64_t> watchdog_heartbeat_{0};  // 心跳计数器
    std::atomic<bool> thread_running_{true};  // 线程运行标志
    std::mutex watchdog_mutex_;

    // 串口接收线程函数
    void serial_recv_task();

    // 串口发送线程函数
    void serial_send_task();
    
    // 看门狗监视线程函数
    void watchdog_task();

    // 遥控器数据接收回调
    static void on_remote_control_data(const uint8_t* data, uint16_t size, void* user_data);

    // 错误包处理回调
    static void on_bad_packet(uint32_t error_type);
};
