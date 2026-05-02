#include "remote_node/remote_node.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <atomic>

RemoteNode::RemoteNode()
    : Node("remote_node"), remote_control_cb_id_(0)
{
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    
    // 打开串口 - 使用极短的超时时间（10ms）以提高实时性
    // 这样可以更快地响应按键数据变化
    serial_ = std::make_unique<serial::Serial>(port, baudrate, serial::Timeout::simpleTimeout(10));
    if (!serial_->isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "打开设备失败:%s", port.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "成功打开设备:%s,波特率为%d", port.c_str(), baudrate);
    
    // 创建ROS2发布器
    move_cmd_pub = this->create_publisher<robot_msgs::msg::Cmd>(
            "robot_move_cmd", 10);
    
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

    // 启动串口接收线程
    serial_recv_thread_ = std::make_unique<std::thread>([this]() { serial_recv_task(); });
    
    // 启动看门狗监视线程
    watchdog_thread_ = std::make_unique<std::thread>([this]() { watchdog_task(); });
    
    RCLCPP_INFO(this->get_logger(), "看门狗监视线程已启动");
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
    
    float rocker0, rocker1, rocker2, rocker3;
    uint32_t key_data;
    
    memcpy(&rocker0, data + 0, sizeof(float));
    memcpy(&rocker1, data + 4, sizeof(float));
    memcpy(&rocker2, data + 8, sizeof(float));
    memcpy(&rocker3, data + 12, sizeof(float));
    memcpy(&key_data, data + 16, sizeof(uint32_t));
    
    // 修正符号：rocker2 和 rocker3 的符号与遥控器显示相反，需要取反
    rocker2 = -rocker2;
    rocker3 = -rocker3;

    // 严格的按键值白名单验证 - 只允许指定的按键值
    // 允许的值：0x0, 0x8, 0x10, 0x20, 0x40, 0x800, 0x1000, 0x2000, 0x4000
    static const uint32_t VALID_KEY_VALUES[] = {
         0x0,0x8, 0x10, 0x20, 0x40, 0x800, 0x1000, 0x2000, 0x4000
    };
    static const size_t VALID_KEY_COUNT = sizeof(VALID_KEY_VALUES) / sizeof(VALID_KEY_VALUES[0]);
    
    bool key_valid = false;
    for (size_t i = 0; i < VALID_KEY_COUNT; i++) {
        if (key_data == VALID_KEY_VALUES[i]) {
            key_valid = true;
            break;
        }
    }
    
    if (!key_valid) {
        RCLCPP_WARN(node->get_logger(),
                   "[数据过滤] 非法按键值 0x%04X，已拒绝此包", key_data);
        return;  // 丢弃此包，不发送
    }
    
    // 发布 ROS 消息
    static uint32_t last_mode = 0;
    auto msg = robot_msgs::msg::Cmd();
    if(key_data == 0x8)        //idle
    {
        msg.mode = 0;      
        last_mode = 0;     
    }    
    else if(key_data == 0x10)    //stop
    {
        msg.mode = 1;
        last_mode = 1;
    }
    else if(key_data == 0x20)    //walk
    {
        msg.mode = 2;     
        last_mode = 2;
    }   
    else if(key_data == 0x40)    //climb_steps
    {
        msg.mode = 3;     
        last_mode = 3;
    }
    else if(key_data == 0x800)   //climb_steps
    {
        msg.mode = 4;      
        last_mode = 4;
    }
    else if(key_data == 0x1000)  
    {
        msg.mode = 5;       //cross_wall
        last_mode = 5;
    }
    else if(key_data == 0x2000)  
    {
        msg.mode = 6;       //jump
        last_mode = 6;
    }
    else if(key_data == 0x4000)  
    {
        msg.mode = 7;      //amble
        last_mode = 7;
    }
    else if(key_data == 0x0)     
    {
        msg.mode = last_mode;  //保持上次模式
    }

    if(std::abs(rocker0) < 100)
        rocker0 = 0.0f;
    if(std::abs(rocker1) < 100)
        rocker1 = 0.0f;
    if(std::abs(rocker2) < 100)
        rocker2 = 0.0f;
    if(std::abs(rocker3) < 100)
        rocker3 = 0.0f;
    msg.vy = std::clamp(rocker0 / 1950.0f, -0.5f, 0.5f);
    msg.vx = std::clamp(rocker1 / 1950.0f, -1.0f, 1.0f);
    msg.vz = std::clamp(rocker2 / 1950.0f, -1.0f, 1.0f);
    msg.wheel_vel = 0 * std::clamp(rocker3 / 1950.0f, -1.0f, 1.0f);

    node->move_cmd_pub->publish(msg);

    
    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        100,
        "vx=%.3f vy=%.3f vz=%.3f wheel=%.3f",
        msg.vx, msg.vy, msg.vz, msg.wheel_vel
    );
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

