#include "remote_node/remote_node.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <robot_msgs/msg/detail/remote__struct.hpp>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <atomic>
#include <filesystem>
#include <vector>

RemoteNode::RemoteNode()
    : Node("remote_node"), remote_control_cb_id_(0)
{
    this->declare_parameter("remote_dev_port","/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0");
    // 创建ROS2发布器
    remote_pub = this->create_publisher<robot_msgs::msg::Remote>(
            "remote", 10);
    
    RCLCPP_INFO(this->get_logger(), "遥控器数据发布器已创建");
    
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

    if (!init_serial()) {
        RCLCPP_WARN(this->get_logger(), "启动时未连接到遥控器串口，接收线程将持续尝试重连");
    }

    // 启动串口接收线程
    serial_recv_thread_ = std::make_unique<std::thread>([this]() { serial_recv_task(); });
    
    // 启动看门狗监视线程
    watchdog_thread_ = std::make_unique<std::thread>([this]() { watchdog_task(); });
    
    RCLCPP_INFO(this->get_logger(), "看门狗监视线程已启动");
}

bool RemoteNode::init_serial()
{
    const std::string port = this->get_parameter("remote_dev_port").as_string();
    constexpr int baudrate = 115200;

    if (port.empty()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "未找到可用串口。请检查遥控器是否已连接，或通过 --ros-args -p remote_dev_port:=/dev/ttyUSB0 指定正确设备。");
        return false;
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);

    try {
        // 使用较短超时以更快响应输入变化。
        if (serial_ && serial_->isOpen()) {
            serial_->close();
        }
        serial_.reset();

        auto serial = std::make_unique<serial::Serial>(
            port, baudrate, serial::Timeout::simpleTimeout(10));
        serial_ = std::move(serial);
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

void RemoteNode::close_serial()
{
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
    serial_.reset();
}

RemoteNode::~RemoteNode()
{
    thread_running_ = false;

    if (serial_recv_thread_) {
        if (serial_recv_thread_->joinable()) {
            serial_recv_thread_->join();
        }
    }

    close_serial();

    if (remote_comm_) {
        remote_comm_->unregister_recv_cb(remote_control_cb_id_);
    }

    if (watchdog_thread_) {
        if (watchdog_thread_->joinable()) {
            watchdog_thread_->join();
        }
    }
}

void RemoteNode::serial_recv_task()
{
    constexpr int MAX_ERRORS_BEFORE_RECONNECT = 3;
    constexpr auto IDLE_SLEEP = std::chrono::milliseconds(1);
    constexpr auto RECONNECT_DELAY = std::chrono::seconds(1);
    constexpr auto NO_DATA_RECONNECT_TIMEOUT = std::chrono::seconds(1);

    int error_count = 0;
    uint8_t buffer[512];  // 更大的批量读取缓冲区
    auto last_data_time = std::chrono::steady_clock::now();
    
    while (thread_running_ && rclcpp::ok()) {
        bool serial_ready = false;
        {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            serial_ready = serial_ && serial_->isOpen();
        }

        if (!serial_ready) {
            if (!init_serial()) {
                std::this_thread::sleep_for(RECONNECT_DELAY);
                continue;
            }
            if (has_connected_once_) {
                should_mark_reconnect_ = true;
            } else {
                has_connected_once_ = true;
            }
            error_count = 0;
            last_data_time = std::chrono::steady_clock::now();
        }

        try {
            size_t bytes_read = 0;
            {
                std::lock_guard<std::mutex> lock(serial_mutex_);
                if (!serial_ || !serial_->isOpen()) {
                    throw std::runtime_error("串口未打开");
                }

                size_t bytes_available = serial_->available();
                if (bytes_available > 0) {
                    size_t bytes_to_read = std::min(bytes_available, size_t(512));
                    bytes_read = serial_->read(buffer, bytes_to_read);
                }
            }

            // 批量读取多个字节以提高性能
            if (bytes_read > 0) {
                error_count = 0;
                // 逐字节传入通信协议处理器，每接收到一批数据立即处理
                for (size_t i = 0; i < bytes_read; i++) {
                    remote_comm_->process_recv_byte(buffer[i]);
                }

                last_data_time = std::chrono::steady_clock::now();

                // 更新看门狗心跳
                watchdog_heartbeat_++;
            } else {
                auto now = std::chrono::steady_clock::now();
                if (now - last_data_time >= NO_DATA_RECONNECT_TIMEOUT) {
                    RCLCPP_WARN(this->get_logger(), "超过6秒未收到遥控器数据，正在重新初始化串口");
                    close_serial();
                    error_count = 0;
                    std::this_thread::sleep_for(RECONNECT_DELAY);
                    continue;
                }

                std::this_thread::sleep_for(IDLE_SLEEP);
            }
        } catch (const std::exception& e) {
            error_count++;
            close_serial();

            if (error_count == 1 || error_count % MAX_ERRORS_BEFORE_RECONNECT == 0) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "串口读取异常: %s。正在尝试重新连接遥控器 (连续错误次数=%d)",
                    e.what(),
                    error_count);
            }

            std::this_thread::sleep_for(RECONNECT_DELAY);
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
    remote.just_reconnected = node->should_mark_reconnect_.exchange(false);

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
            // 没有收到新的心跳，可能暂时没有遥控器数据或正在重连
            heartbeat_miss_count++;
            
            if (heartbeat_miss_count == 1) {
                RCLCPP_WARN(this->get_logger(), 
                    "[看门狗] 未接收到新遥控器数据，接收线程将继续重连。心跳值=%lu", 
                    current_heartbeat);
            } else if (heartbeat_miss_count == 3) {
                RCLCPP_ERROR(this->get_logger(), 
                    "[看门狗] 6秒无新数据，当前仍在自动恢复流程中。心跳值=%lu", 
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
