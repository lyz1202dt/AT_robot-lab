#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <serial/serial.h>

class IMUDriver {
public:
    IMUDriver(
        const std::string& port="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0003-if00-port0",
        unsigned long baudrate=921600,
        std::chrono::duration<double> timeout_time = std::chrono::milliseconds(100));
    ~IMUDriver();

    IMUDriver(const IMUDriver&) = delete;
    IMUDriver& operator=(const IMUDriver&) = delete;
    IMUDriver(IMUDriver&&) = delete;
    IMUDriver& operator=(IMUDriver&&) = delete;

    bool is_open() const;
    bool has_received_data() const;
    
    bool get_imu_state(
        Eigen::Vector3d& angular_velocity_out,
        Eigen::Vector3d& acceleration_out,
        Eigen::Quaterniond& rotation_out);

private:
    static uint8_t calc_crc8(const uint8_t* data, std::size_t len);
    static uint16_t calc_crc16(const uint8_t* data, std::size_t len);

    bool read_exact(uint8_t* dst, std::size_t size);
    void update_last_receive_time();

    void data_recv();
    int pack_parsing();

    static constexpr uint8_t PACKET_START = 0xFC;
    static constexpr uint8_t PACKET_END = 0xFD;
    static constexpr uint8_t MSG_ACCELERATION = 0x61;
    static constexpr uint8_t MSG_ANGULAR_VEL = 0x66;
    static constexpr uint8_t MSG_QUAT_ORIEN = 0x64;
    static constexpr std::size_t MAX_PACKET_SIZE = 128;

    std::array<uint8_t, MAX_PACKET_SIZE> buffer_{};
    std::size_t buffer_len_{0};

    const std::chrono::duration<double> timeout_time_;
    std::unique_ptr<serial::Serial> serial_;
    std::unique_ptr<std::thread> serial_transmit_thread_;

    mutable std::mutex state_mutex_;
    bool stop_requested_{false};
    bool first_receive_{true};
    std::chrono::time_point<std::chrono::steady_clock> last_update_time_{};

    Eigen::Vector3d angular_velocity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond rotation_{Eigen::Quaterniond::Identity()};

    uint8_t last_seq_num_{0};
    bool first_packet_{true};

    static const uint8_t CRC8_Table[256];
    static const uint16_t CRC16_Table[256];
};
