#include "remote_node/remote_node.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <robot_msgs/msg/detail/remote__struct.hpp>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <atomic>
#include <filesystem>
#include <vector>

RemoteNode::RemoteNode()
    : Node("remote_node"), remote_control_cb_id_(0)
{
    this->declare_parameter("remote_dev_port","/dev/ttyCH341USB0");
    // 创建ROS2发布器
    remote_pub = this->create_publisher<robot_msgs::msg::Remote>(
            "remote", 10);
    
    RCLCPP_INFO(this->get_logger(), "遥控器数据发布器已创建");

    if (!init_serial()) {
        return;
    }
    
    // 初始化通信协议处理器
    remote_comm_ = std::make_unique<RemoteComm>();
    remote_comm_->init([this](uint32_t error_type) {
        on_bad_packet(error_type);
    });

    // 注册遥控器数据接收回调
    remote_control_cb_id_ = remote_comm_->register_recv_cb(
        [this](const uint8_t* data, uint16_t size, void* user_data) {
            on_remote_control_data(data, size, user_data);
        },
        CMD_REMOTE_CONTROL,
        this
    );

    // 启动串口接收线程
    serial_recv_thread_ = std::make_unique<std::thread>([this]() { serial_recv_task(); });
    
    // 启动看门狗监视线程
    watchdog_thread_ = std::make_unique<std::thread>([this]() { watchdog_task(); });
    
    RCLCPP_INFO(this->get_logger(), "看门狗监视线程已启动");
}

bool RemoteNode::init_serial()
{
    const std::string configured_port = this->get_parameter("remote_dev_port").as_string();
    const std::string port = resolve_serial_port(configured_port);
    constexpr int baudrate = 115200;

    if (port.empty()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "未找到可用串口。请检查遥控器是否已连接，或通过 --ros-args -p remote_dev_port:=/dev/ttyUSB0 指定正确设备。");
        return false;
    }

    try {
        // 使用较短超时以更快响应输入变化。
        serial_ = std::make_unique<serial::Serial>(
            port, baudrate, serial::Timeout::simpleTimeout(10));
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "打开串口失败: %s。异常信息: %s",
            port.c_str(),
            e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            this->get_logger(),
            "初始化串口时发生异常: %s。异常信息: %s",
            port.c_str(),
            e.what());
        return false;
    }

    if (!serial_ || !serial_->isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "打开设备失败: %s", port.c_str());
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "成功打开设备:%s,波特率为%d", port.c_str(), baudrate);
    return true;
}

std::string RemoteNode::resolve_serial_port(const std::string& configured_port) const
{
    namespace fs = std::filesystem;

    if (fs::exists(configured_port)) {
        return configured_port;
    }

    const std::vector<std::string> fallback_ports = {
        "/dev/ttyUSB0",
        "/dev/ttyUSB1",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    };

    for (const auto& candidate : fallback_ports) {
        if (candidate == configured_port) {
            continue;
        }
        if (fs::exists(candidate)) {
            RCLCPP_WARN(
                this->get_logger(),
                "配置的串口 %s 不存在，自动改用检测到的设备 %s",
                configured_port.c_str(),
                candidate.c_str());
            return candidate;
        }
    }

    std::ostringstream oss;
    bool has_any_device = false;
    for (const auto& candidate : fallback_ports) {
        if (fs::exists(candidate)) {
            if (has_any_device) {
                oss << ", ";
            }
            oss << candidate;
            has_any_device = true;
        }
    }

    if (has_any_device) {
        RCLCPP_ERROR(
            this->get_logger(),
            "配置的串口 %s 不存在，可检测到的串口有: %s",
            configured_port.c_str(),
            oss.str().c_str());
    } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "配置的串口 %s 不存在，当前也未检测到常见串口设备 (/dev/ttyUSB* 或 /dev/ttyACM*)",
            configured_port.c_str());
    }

    return "";
}

RemoteNode::~RemoteNode()
{
    // 先停止看门狗线程
    thread_running_ = false;
    
    if (remote_comm_) {
        remote_comm_->unregister_recv_cb(remote_control_cb_id_);
    }
    
    if (serial_ && serial_->isOpen())
        serial_->close();
    
    if (watchdog_thread_) {
        if (watchdog_thread_->joinable())
            watchdog_thread_->join();
    }
    
    if (serial_recv_thread_) {
        if (serial_recv_thread_->joinable())
            serial_recv_thread_->join();
    }
}

void RemoteNode::serial_recv_task()
{
    int error_count = 0;
    const int MAX_ERRORS = 5;
    uint8_t buffer[512];  // 更大的批量读取缓冲区
    auto last_print_time = std::chrono::steady_clock::now();
    
    while (rclcpp::ok()) {
        if (!serial_ || !serial_->isOpen()) {
            break;
        }

        try {
            // 批量读取多个字节以提高性能
            size_t bytes_available = serial_->available();
            if (bytes_available > 0) {
                size_t bytes_to_read = std::min(bytes_available, size_t(512));
                size_t bytes_read = serial_->read(buffer, bytes_to_read);
                
                if (bytes_read > 0) {
                    error_count = 0;
                    // 逐字节传入通信协议处理器，每接收到一批数据立即处理
                    for (size_t i = 0; i < bytes_read; i++) {
                        remote_comm_->process_recv_byte(buffer[i]);
                        // RCLCPP_INFO(this->get_logger(), "接收到数据: 0x%02X", buffer[i]);
                    }
                    last_print_time = std::chrono::steady_clock::now();
                    
                    // 更新看门狗心跳
                    watchdog_heartbeat_++;
                }
            } else {
                // 无数据时，加入极短的睡眠(100微秒)避免CPU 100%占用导致系统卡死
                // 这个延迟相比690ms的消息间隔可以忽略不计
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            
            // 定期检查线程是否响应（看门狗机制）
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print_time);
            if (elapsed.count() > 30) {
                RCLCPP_WARN(this->get_logger(), "[看门狗] 30秒无新数据，检查遥控器连接");
                last_print_time = now;
            }
        } catch (const std::exception& e) {
            error_count++;
            if (error_count == 1) {
                RCLCPP_ERROR(this->get_logger(), "[异常] 串口读取错误: %s", e.what());
            }
            if (error_count >= MAX_ERRORS) {
                RCLCPP_ERROR(this->get_logger(), "[严重错误] 串口连续出错%d次，可能串口断开，退出接收线程", MAX_ERRORS);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void RemoteNode::on_remote_control_data(const uint8_t* data, uint16_t size, void* user_data)
{
    RemoteNode* node = reinterpret_cast<RemoteNode*>(user_data);

    // data 应包含遥控器的控制数据
    // 格式：float[4] (rocker0-3) + uint32_t (key) = 20 字节
    if (size < 20) {
        RCLCPP_WARN(node->get_logger(), 
                   "遥控器数据长度错误：期望>=20 字节，实际%u 字节", size);
        return;
    }
    
    robot_msgs::msg::Remote remote;
    
    memcpy(&remote.lx, data + 0, sizeof(float));
    memcpy(&remote.ly, data + 4, sizeof(float));
    memcpy(&remote.rx, data + 8, sizeof(float));
    memcpy(&remote.ry, data + 12, sizeof(float));
    memcpy(&remote.key, data + 16, sizeof(uint32_t));

    node->remote_pub->publish(remote);
}



void RemoteNode::on_bad_packet(uint32_t error_type)
{
    switch (error_type) {
        case 1:  // BAD_CHECKSUM
            RCLCPP_INFO(rclcpp::get_logger("remote_node"), "校验和错误");
            break;
        case 2:  // BAD_LENGTH
            RCLCPP_INFO(rclcpp::get_logger("remote_node"), "包长度错误");
            break;
        case 3:  // BAD_HEAD
            RCLCPP_INFO(rclcpp::get_logger("remote_node"), "包头错误");
            break;
        case 4:  // BAD_ACK
            RCLCPP_INFO(rclcpp::get_logger("remote_node"), "ACK包错误");
            break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("remote_node"), "未知错误类型: %u", error_type);
    }
}

void RemoteNode::watchdog_task()
{
    // 看门狗线程 - 定期检查系统健康状态
    uint64_t last_heartbeat = 0;
    int heartbeat_miss_count = 0;
    
    while (thread_running_ && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));  // 每2秒检查一次
        
        uint64_t current_heartbeat = watchdog_heartbeat_.load();
        
        if (current_heartbeat == last_heartbeat) {
            // 没有收到新的心跳，可能线程卡死了
            heartbeat_miss_count++;
            
            if (heartbeat_miss_count == 1) {
                RCLCPP_WARN(this->get_logger(), 
                    "[看门狗] 未接收到新数据 (心跳停止)，系统可能卡死。心跳值=%lu", 
                    current_heartbeat);
            } else if (heartbeat_miss_count == 3) {
                RCLCPP_ERROR(this->get_logger(), 
                    "[看门狗] 6秒无心跳更新！系统已卡死。建议重启节点。心跳值=%lu", 
                    current_heartbeat);
            }
        } else {
            // 收到新心跳，系统正常
            if (heartbeat_miss_count > 0) {
                RCLCPP_INFO(this->get_logger(), 
                    "[看门狗] 系统已恢复正常，心跳已更新。心跳值=%lu", 
                    current_heartbeat);
            }
            heartbeat_miss_count = 0;
        }
        
        last_heartbeat = current_heartbeat;
    }
    
    RCLCPP_INFO(this->get_logger(), "[看门狗] 监视线程已退出");
}
