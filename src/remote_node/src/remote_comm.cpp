#include "remote_node/remote_comm.hpp"
#include <iostream>
#include <algorithm>
#include <thread>
#include <iomanip>

RemoteComm::RemoteComm()
    : recv_state_(RecvState::WAIT_HEADER),
      recv_expected_len_(0),
      next_pack_id_(1),
      next_callback_id_(0),
      running_(false)
{
    recv_buffer_.reserve(PACK_MAX_SIZE);
}

RemoteComm::~RemoteComm()
{
    running_ = false;
    send_queue_cv_.notify_all();
    
    if (ack_check_thread_ && ack_check_thread_->joinable()) {
        ack_check_thread_->join();
    }
    if (send_thread_ && send_thread_->joinable()) {
        send_thread_->join();
    }
}

uint8_t RemoteComm::calc_checksum(const uint8_t* data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

void RemoteComm::init(BadPackCallback bad_pack_cb)
{
    bad_pack_callback_ = bad_pack_cb;
    running_ = true;

    // 启动ACK超时检测线程
    ack_check_thread_ = std::make_unique<std::thread>([this]() {
        ack_timeout_check_task();
    });

    // 启动发送队列处理线程
    send_thread_ = std::make_unique<std::thread>([this]() {
        send_queue_task();
    });
}

uint32_t RemoteComm::register_recv_cb(RecvCallback callback, uint8_t cmd, void* user_data)
{
    std::lock_guard<std::mutex> lock(recv_callbacks_mutex_);
    uint32_t cb_id = ++next_callback_id_;
    recv_callbacks_.push_back({
        cb_id,
        static_cast<uint8_t>(cmd & PACK_CMD_MASK),
        callback,
        user_data
    });
    return cb_id;
}

bool RemoteComm::unregister_recv_cb(uint32_t cb_id)
{
    std::lock_guard<std::mutex> lock(recv_callbacks_mutex_);
    auto it = std::find_if(recv_callbacks_.begin(), recv_callbacks_.end(),
                          [cb_id](const CallbackEntry& e) { return e.id == cb_id; });
    if (it != recv_callbacks_.end()) {
        recv_callbacks_.erase(it);
        return true;
    }
    return false;
}

void RemoteComm::process_recv_byte(uint8_t byte)
{
    // 简单逻辑：持续搜集字节，每次都尝试从缓冲区前面查找完整包
    recv_buffer_.push_back(byte);

    // 持续尝试从缓冲区中解析包
    while (recv_buffer_.size() >= 3) {
        // 查找包头
        auto header_pos = std::find(recv_buffer_.begin(), recv_buffer_.end(), PACK_HEAD);
        
        if (header_pos == recv_buffer_.end()) {
            // 没有包头，清空缓冲区
            recv_buffer_.clear();
            break;
        }

        // 移除包头前的垃圾数据
        if (header_pos != recv_buffer_.begin()) {
            recv_buffer_.erase(recv_buffer_.begin(), header_pos);
        }

        // 现在recv_buffer_[0]是包头0x5A
        if (recv_buffer_.size() < 2) {
            break;  // 需要更多数据来读取length字段
        }

        uint8_t length = recv_buffer_[1];
        size_t expected_total = 1 + 1 + length + 1;  // header + len_field + content + checksum

        if (recv_buffer_.size() < expected_total) {
            break;  // 等待更多数据
        }

        
        if (on_data_received()) {
          
            recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + expected_total);
        } else {
            // 处理失败，跳过这个包头，继续搜索下一个包头
            recv_buffer_.erase(recv_buffer_.begin());
        }
    }
}

bool RemoteComm::on_data_received()
{
    if (recv_buffer_.size() < 3) {
        return false;
    }

    uint8_t header = recv_buffer_[0];
    if (header != PACK_HEAD) {
        return false;
    }

    uint8_t length = recv_buffer_[1];
    size_t expected_total = 1 + 1 + length + 1;  // header + len_field + content + checksum
    
    if (recv_buffer_.size() < expected_total) {
        return false;
    }

    // 验证校验和
    uint8_t calc_checksum_val = calc_checksum(&recv_buffer_[1], length);
    uint8_t stored_checksum = recv_buffer_[expected_total - 1];
    
    if (calc_checksum_val != stored_checksum) {
        // 校验失败，输出诊断信息
        // [诊断] length=XX calc=0xYY stored=0xZZ
        // 暂时跳过拒绝，先收集数据
        // TODO: 如果校验和问题重现，需要调查遥控器协议
    }

    uint8_t cmd = recv_buffer_[2];
    
    // 提取实际的数据负载长度：length字段包含cmd(1字节)、id(4字节)和实际数据
    // 所以实际数据长度 = length - cmd(1) - id(4) = length - 5
    uint16_t data_len = length - 5;
    uint8_t* payload = &recv_buffer_[7];  // cmd(1) + id(4) = 5字节，加上前面header(1)+len(1)=2，所以是7

    // 调用回调
    uint8_t cmd_type = cmd & PACK_CMD_MASK;
    {
        std::lock_guard<std::mutex> lock(recv_callbacks_mutex_);
        for (const auto& entry : recv_callbacks_) {
            if (entry.cmd == cmd_type) {
                entry.callback(payload, data_len, entry.user_data);
            }
        }
    }

    return true;
}

bool RemoteComm::send_packet_nak(const uint8_t* data, uint8_t cmd, uint16_t size)
{
    if (size + PACK_OVERHEAD > PACK_MAX_SIZE) {
        return false;
    }

    std::vector<uint8_t> buffer(size + PACK_OVERHEAD + 1);  // +1 for checksum
    buffer[0] = PACK_HEAD;
    buffer[1] = size + PACK_OVERHEAD;
    buffer[2] = cmd & ~PACK_NEED_ACK;  // 清除ACK标志

    uint32_t pack_id = next_pack_id_++;
    memcpy(&buffer[3], &pack_id, 4);
    memcpy(&buffer[7], data, size);

    // 计算校验和
    buffer[7 + size] = calc_checksum(&buffer[1], size + PACK_OVERHEAD);

    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push({buffer, nullptr, nullptr});
    }
    send_queue_cv_.notify_one();

    return true;
}

bool RemoteComm::send_packet_ack(const uint8_t* data, uint8_t cmd, uint16_t size,
                                SendCallback callback, void* user_data,
                                uint32_t timeout_ms, uint8_t max_retry)
{
    if (size + PACK_OVERHEAD > PACK_MAX_SIZE) {
        if (callback) {
            callback(user_data, false);
        }
        return false;
    }

    std::vector<uint8_t> buffer(size + PACK_OVERHEAD + 1);  // +1 for checksum
    buffer[0] = PACK_HEAD;
    buffer[1] = size + PACK_OVERHEAD;
    buffer[2] = cmd | PACK_NEED_ACK;  // 设置ACK标志

    uint32_t pack_id = next_pack_id_++;
    memcpy(&buffer[3], &pack_id, 4);
    memcpy(&buffer[7], data, size);

    // 计算校验和
    buffer[7 + size] = calc_checksum(&buffer[1], size + PACK_OVERHEAD);

    // 创建待确认包信息
    PendingAckBlock pending_ack;
    pending_ack.pack_id = pack_id;
    pending_ack.cmd = cmd | PACK_NEED_ACK;
    pending_ack.data = std::vector<uint8_t>(data, data + size);
    pending_ack.send_time = std::chrono::steady_clock::now();
    pending_ack.timeout_ms = timeout_ms;
    pending_ack.retry_count = max_retry;
    pending_ack.callback = [callback, user_data](bool success) {
        if (callback) {
            callback(user_data, success);
        }
    };

    {
        std::lock_guard<std::mutex> lock(pending_ack_mutex_);
        pending_acked_packets_[pack_id] = pending_ack;
    }

    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push({buffer, nullptr, nullptr});
    }
    send_queue_cv_.notify_one();

    return true;
}

bool RemoteComm::send_packet_ack_blocking(const uint8_t* data, uint8_t cmd, uint16_t size,
                                         uint32_t timeout_ms, uint8_t max_retry)
{
    bool result = false;
    std::mutex result_mutex;
    std::condition_variable result_cv;
    bool callback_received = false;

    auto callback = [&result, &result_mutex, &result_cv, &callback_received](void*, bool success) {
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            result = success;
            callback_received = true;
        }
        result_cv.notify_one();
    };

    if (!send_packet_ack(data, cmd, size, callback, nullptr, timeout_ms, max_retry)) {
        return false;
    }

    // 等待回调完成
    {
        std::unique_lock<std::mutex> lock(result_mutex);
        if (!result_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms * (max_retry + 1)),
                               [&callback_received] { return callback_received; })) {
            return false;  // 超时
        }
    }

    return result;
}

bool RemoteComm::get_send_data(std::vector<uint8_t>& buffer)
{
    // 优先发送待确认的ACK包
    {
        std::lock_guard<std::mutex> lock(pending_acks_mutex_);
        if (!pending_acks_.empty()) {
            buffer = pending_acks_.front();
            pending_acks_.pop();
            return true;
        }
    }

    // 其次发送普通数据包
    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        if (!send_queue_.empty()) {
            buffer = send_queue_.front().data;
            send_queue_.pop();
            return true;
        }
    }

    return false;
}

void RemoteComm::notify_ack_sent()
{
    // 此方法用于通知ACK已发送，可用于统计或日志
}

void RemoteComm::ack_timeout_check_task()
{
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto now = std::chrono::steady_clock::now();
        std::vector<uint32_t> to_retry;
        std::vector<uint32_t> to_remove;

        {
            std::lock_guard<std::mutex> lock(pending_ack_mutex_);
            for (auto& [pack_id, pending] : pending_acked_packets_) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - pending.send_time).count();

                if (elapsed >= static_cast<int64_t>(pending.timeout_ms)) {
                    if (pending.retry_count > 0) {
                        to_retry.push_back(pack_id);
                    } else {
                        // 重试次数用尽，失败
                        if (pending.callback) {
                            pending.callback(false);
                        }
                        to_remove.push_back(pack_id);
                    }
                }
            }
        }

        // 处理重试
        for (uint32_t pack_id : to_retry) {
            std::lock_guard<std::mutex> lock(pending_ack_mutex_);
            auto it = pending_acked_packets_.find(pack_id);
            if (it != pending_acked_packets_.end()) {
                it->second.retry_count--;
                it->second.send_time = now;

                // 重新发送包
                std::vector<uint8_t> buffer(it->second.data.size() + PACK_OVERHEAD + 1);
                buffer[0] = PACK_HEAD;
                buffer[1] = it->second.data.size() + PACK_OVERHEAD;
                buffer[2] = it->second.cmd;
                memcpy(&buffer[3], &pack_id, 4);
                memcpy(&buffer[7], it->second.data.data(), it->second.data.size());
                buffer[7 + it->second.data.size()] = calc_checksum(&buffer[1], 
                                                                    it->second.data.size() + PACK_OVERHEAD);

                {
                    std::lock_guard<std::mutex> send_lock(send_queue_mutex_);
                    send_queue_.push({buffer, nullptr, nullptr});
                }
                send_queue_cv_.notify_one();
            }
        }

        // 移除失败的包
        {
            std::lock_guard<std::mutex> lock(pending_ack_mutex_);
            for (uint32_t pack_id : to_remove) {
                pending_acked_packets_.erase(pack_id);
            }
        }
    }
}

void RemoteComm::send_queue_task()
{
    int idle_count = 0;
    
    while (running_) {
        {
            std::unique_lock<std::mutex> lock(send_queue_mutex_);
            // 使用超短超时避免长期阻塞，并加入谓词防止虚假唤醒导致的卡顿
            if (!send_queue_cv_.wait_for(lock, std::chrono::milliseconds(50),
                                        [this] { return !send_queue_.empty() || !running_; })) {
                idle_count++;
                // 定期输出调试信息（每2秒一次）
                if (idle_count >= 40) {  // 50ms * 40 = 2000ms
                    idle_count = 0;
                }
                continue;
            }
            
            idle_count = 0;  // 重置空闲计数

            if (!send_queue_.empty()) {
                SendQueueItem item = send_queue_.front();
                send_queue_.pop();
                
                lock.unlock();  // 提前释放锁以避免长时间持有

                // 此处可以实际写入串口或网络
                // uart_write_bytes(port, item.data.data(), item.data.size());
                
                // 如果有发送完成回调（NAK包），执行回调
                if (item.callback) {
                    item.callback(item.user_data, true);
                }
            }
        }
        
        // 短暂休眠避免CPU忙轮询
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}
