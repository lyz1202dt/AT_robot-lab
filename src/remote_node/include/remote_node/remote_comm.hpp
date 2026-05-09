#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <functional>
#include <map>
#include <memory>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

// 协议常量定义
constexpr uint8_t PACK_HEAD = 0x5A;                    // 标准帧包头
constexpr uint8_t ACK_HEAD = 0xAA;                     // ACK帧包头
constexpr uint8_t PACK_NEED_ACK = 0x80;                // 需要ACK类型标志
constexpr uint8_t PACK_TYPE_NAK = 0x00;                // 不需要ACK类型
constexpr uint8_t PACK_CMD_MASK = 0x7F;                // 命令掩码（低7位）
constexpr uint16_t PACK_MAX_SIZE = 256;                // 最大包大小
// len字段 = cmd(1) + id(4) + data(n)，所以PACK_OVERHEAD是cmd和id的总大小
constexpr uint8_t PACK_OVERHEAD = 5;                   // 包开销(cmd+id)= 1+4=5字节

// 包结构体
struct DataPacket {
    uint8_t cmd;                                        // 命令字段（高位标记ACK需求）
    uint32_t pack_id;                                   // 包ID
    std::vector<uint8_t> data;                          // 数据体
};

// ACK包结构体
struct AckPacket {
    uint32_t pack_id;                                   // 被确认的包ID
};

// 待确认包信息
struct PendingAckBlock {
    uint32_t pack_id;                                   // 包ID
    uint8_t cmd;                                        // 原始命令
    std::vector<uint8_t> data;                          // 原始数据
    std::chrono::steady_clock::time_point send_time;    // 发送时间
    uint32_t timeout_ms;                                // 超时时间
    uint8_t retry_count;                                // 剩余重试次数
    std::function<void(bool)> callback;                 // 完成回调
};

// 协议处理类
class RemoteComm {
public:
    // 接收回调函数类型
    using RecvCallback = std::function<void(const uint8_t*, uint16_t, void*)>;
    using SendCallback = std::function<void(void*, bool)>;
    using BadPackCallback = std::function<void(uint32_t)>;

    RemoteComm();
    ~RemoteComm();

    // 初始化通信模块
    void init(BadPackCallback bad_pack_cb = nullptr);

    // 注册接收回调（针对特定命令）
    uint32_t register_recv_cb(RecvCallback callback, uint8_t cmd, void* user_data);

    // 取消注册接收回调
    bool unregister_recv_cb(uint32_t cb_id);

    // 异步发送NAK包（不需要ACK）
    bool send_packet_nak(const uint8_t* data, uint8_t cmd, uint16_t size);

    // 异步发送需要ACK的包
    bool send_packet_ack(const uint8_t* data, uint8_t cmd, uint16_t size,
                        SendCallback callback, void* user_data,
                        uint32_t timeout_ms, uint8_t max_retry = 3);

    // 阻塞式发送需要ACK的包
    bool send_packet_ack_blocking(const uint8_t* data, uint8_t cmd, uint16_t size,
                                 uint32_t timeout_ms, uint8_t max_retry = 3);

    // 处理收到的数据字节（外部调用，逐字节供给）
    void process_recv_byte(uint8_t byte);

    // 获取待发送的数据（用于实际发送）
    bool get_send_data(std::vector<uint8_t>& buffer);

    // 通知ACK已发送
    void notify_ack_sent();

private:
    // 内部状态机
    enum class RecvState {
        WAIT_HEADER,        // 等待包头
        RECV_LENGTH,        // 接收长度
        RECV_CMD,           // 接收命令
        RECV_ID,            // 接收包ID
        RECV_DATA,          // 接收数据
        RECV_CHECKSUM      // 接收校验和
    };

    // 校验和计算
    static uint8_t calc_checksum(const uint8_t* data, size_t len);

    // 接收状态处理
    void on_header_received(uint8_t byte);
    bool on_data_received();

    // ACK超时检测和重试
    void ack_timeout_check_task();

    // 发送队列处理
    void send_queue_task();

    // 内部数据成员
    RecvState recv_state_;
    std::vector<uint8_t> recv_buffer_;
    size_t recv_expected_len_;

    uint32_t next_pack_id_;
    uint32_t next_callback_id_;

    // 接收回调注册
    struct CallbackEntry {
        uint32_t id;
        uint8_t cmd;
        RecvCallback callback;
        void* user_data;
    };
    std::vector<CallbackEntry> recv_callbacks_;
    std::mutex recv_callbacks_mutex_;

    // 待确认包管理
    std::map<uint32_t, PendingAckBlock> pending_acked_packets_;
    std::mutex pending_ack_mutex_;

    // 发送队列
    struct SendQueueItem {
        std::vector<uint8_t> data;
        SendCallback callback;
        void* user_data;
    };
    std::queue<SendQueueItem> send_queue_;
    std::mutex send_queue_mutex_;
    std::condition_variable send_queue_cv_;
    SendQueueItem pending_ack_send_;      // 当前待发送的ACK包

    // 待发送的ACK包队列
    std::queue<std::vector<uint8_t>> pending_acks_;
    std::mutex pending_acks_mutex_;

    // 线程管理
    std::unique_ptr<std::thread> ack_check_thread_;
    std::unique_ptr<std::thread> send_thread_;
    bool running_;

    // 错误回调
    BadPackCallback bad_pack_callback_;

    // 错误类型枚举
    enum BadPackType {
        BAD_CHECKSUM = 1,
        BAD_LENGTH = 2,
        BAD_HEAD = 3,
        BAD_ACK = 4
    };
};
