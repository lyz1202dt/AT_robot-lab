#include "imu_driver/imu_driver.hpp"

#include <cmath>
#include <cstring>
#include <exception>
#include <iostream>

namespace {

constexpr std::size_t kHeaderBytesAfterStart = 4;  // id + len + seq + crc8
constexpr std::size_t kCrc16Bytes = 2;
constexpr std::size_t kPacketOverhead = 1 + kHeaderBytesAfterStart + kCrc16Bytes + 1;

bool is_valid_packet_id(uint8_t packet_id) {
    return packet_id == 0x61 || packet_id == 0x66 || packet_id == 0x64;
}

}  // namespace

const uint8_t IMUDriver::CRC8_Table[256] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

const uint16_t IMUDriver::CRC16_Table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

IMUDriver::IMUDriver(
    const std::string& port,
    unsigned long baudrate,
    std::chrono::duration<double> timeout_time)
    : timeout_time_(timeout_time) {
    try {
        serial_ = std::make_unique<serial::Serial>(
            port,
            baudrate,
            serial::Timeout::simpleTimeout(
                static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::milliseconds>(timeout_time_).count())));
    } catch (const std::exception& e) {
        std::cerr << "[IMUDriver] Failed to open serial port " << port
                  << ": " << e.what() << std::endl;
        return;
    }

    if (!serial_ || !serial_->isOpen()) {
        std::cerr << "[IMUDriver] Serial port is not open: " << port << std::endl;
        return;
    }

    serial_transmit_thread_ = std::make_unique<std::thread>([this]() { data_recv(); });
}

IMUDriver::~IMUDriver() {
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        stop_requested_ = true;
    }

    if (serial_ && serial_->isOpen()) {
        try {
            serial_->close();
        } catch (const std::exception& e) {
            std::cerr << "[IMUDriver] Failed to close serial port: " << e.what() << std::endl;
        }
    }

    if (serial_transmit_thread_ && serial_transmit_thread_->joinable()) {
        serial_transmit_thread_->join();
    }
}

bool IMUDriver::is_open() const {
    return serial_ && serial_->isOpen();
}

bool IMUDriver::has_received_data() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return !first_receive_;
}

bool IMUDriver::get_imu_state(
    Eigen::Vector3d& angular_velocity_out,
    Eigen::Vector3d& acceleration_out,
    Eigen::Quaterniond& rotation_out) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (first_receive_) {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now - last_update_time_ > timeout_time_) {
        return false;
    }

    angular_velocity_out = angular_velocity_;
    acceleration_out = acceleration_;
    rotation_out = rotation_;

    //std::cout<<"IMU:"<<rotation_out.x()<<","<<rotation_out.y()<<","<<rotation_out.z()<<","<<rotation_out.w()<<std::endl;
    return true;
}

uint8_t IMUDriver::calc_crc8(const uint8_t* data, std::size_t len) {
    uint8_t crc8 = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc8 = CRC8_Table[crc8 ^ data[i]];
    }
    return crc8;
}

uint16_t IMUDriver::calc_crc16(const uint8_t* data, std::size_t len) {
    uint16_t crc16 = 0;
    for (std::size_t i = 0; i < len; ++i) {
        crc16 = CRC16_Table[((crc16 >> 8) ^ data[i]) & 0xFF] ^ (crc16 << 8);
    }
    return crc16;
}

bool IMUDriver::read_exact(uint8_t* dst, std::size_t size) {
    std::size_t total_read = 0;
    while (total_read < size) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (stop_requested_) {
                return false;
            }
        }

        if (!serial_ || !serial_->isOpen()) {
            return false;
        }

        const std::size_t bytes_read = serial_->read(dst + total_read, size - total_read);
        if (bytes_read == 0) {
            return false;
        }
        total_read += bytes_read;
    }
    return true;
}

void IMUDriver::update_last_receive_time() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_update_time_ = std::chrono::steady_clock::now();
    first_receive_ = false;
}

void IMUDriver::data_recv() {
    while (true) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (stop_requested_) {
                break;
            }
        }

        if (!serial_ || !serial_->isOpen()) {
            break;
        }

        try {
            uint8_t byte = 0;
            bool found_start = false;
            while (!found_start) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (stop_requested_) {
                        return;
                    }
                }

                if (!serial_ || !serial_->isOpen()) {
                    return;
                }

                const std::size_t bytes_read = serial_->read(&byte, 1);
                if (bytes_read == 0) {
                    continue;
                }
                if (byte == PACKET_START) {
                    buffer_[0] = byte;
                    buffer_len_ = 1;
                    found_start = true;
                }
            }

            if (!read_exact(buffer_.data() + buffer_len_, kHeaderBytesAfterStart)) {
                buffer_len_ = 0;
                continue;
            }
            buffer_len_ += kHeaderBytesAfterStart;

            const uint8_t packet_id = buffer_[1];
            const uint8_t data_len = buffer_[2];
            const uint8_t seq_num = buffer_[3];
            const uint8_t crc8_received = buffer_[4];
            const uint8_t crc8_calc = calc_crc8(buffer_.data(), 4);

            if (!is_valid_packet_id(packet_id) || crc8_calc != crc8_received) {
                buffer_len_ = 0;
                continue;
            }

            if (kPacketOverhead + data_len > buffer_.size()) {
                buffer_len_ = 0;
                continue;
            }

            if (!first_packet_) {
                const uint8_t expected_seq_num = static_cast<uint8_t>(last_seq_num_ + 1);
                if (seq_num != expected_seq_num) {
                    // std::cerr << "[IMUDriver] Packet sequence jump: expected "
                    //           << static_cast<int>(expected_seq_num) << ", got "
                    //           << static_cast<int>(seq_num) << std::endl;
                }
            } else {
                first_packet_ = false;
            }
            last_seq_num_ = seq_num;

            const std::size_t remaining_bytes = kCrc16Bytes + data_len + 1;
            if (!read_exact(buffer_.data() + buffer_len_, remaining_bytes)) {
                buffer_len_ = 0;
                continue;
            }
            buffer_len_ += remaining_bytes;

            const uint16_t crc16_received =
                static_cast<uint16_t>(buffer_[5] << 8) | static_cast<uint16_t>(buffer_[6]);
            const uint8_t* payload = buffer_.data() + 7;
            const uint16_t crc16_calc = calc_crc16(payload, data_len);
            const std::size_t end_pos = 7 + data_len;

            if (crc16_calc != crc16_received || buffer_[end_pos] != PACKET_END) {
                buffer_len_ = 0;
                continue;
            }

            if (pack_parsing() == 0) {
                update_last_receive_time();
            }

            buffer_len_ = 0;
        } catch (const std::exception& e) {
            std::cerr << "[IMUDriver] Serial read error: " << e.what() << std::endl;
            buffer_len_ = 0;
        }
    }
}

int IMUDriver::pack_parsing() {
    if (buffer_len_ < kPacketOverhead) {
        return -1;
    }

    const uint8_t packet_id = buffer_[1];
    const uint8_t data_len = buffer_[2];
    const uint8_t* payload = buffer_.data() + 7;

    if (packet_id == MSG_ACCELERATION) {
        if (data_len < 3 * sizeof(float)) {
            return -1;
        }

        float acc_x = 0.0F;
        float acc_y = 0.0F;
        float acc_z = 0.0F;
        std::memcpy(&acc_x, payload, sizeof(float));
        std::memcpy(&acc_y, payload + 4, sizeof(float));
        std::memcpy(&acc_z, payload + 8, sizeof(float));

        std::lock_guard<std::mutex> lock(state_mutex_);
        acceleration_ = Eigen::Vector3d(-acc_x, acc_y, -acc_z);
        return 0;
    }

    if (packet_id == MSG_ANGULAR_VEL) {
        if (data_len < 3 * sizeof(float)) {
            return -1;
        }

        float gyro_x = 0.0F;
        float gyro_y = 0.0F;
        float gyro_z = 0.0F;
        std::memcpy(&gyro_x, payload, sizeof(float));
        std::memcpy(&gyro_y, payload + 4, sizeof(float));
        std::memcpy(&gyro_z, payload + 8, sizeof(float));

        std::lock_guard<std::mutex> lock(state_mutex_);
        angular_velocity_ = Eigen::Vector3d(-gyro_x, gyro_y, -gyro_z);
        return 0;
    }

    if (packet_id == MSG_QUAT_ORIEN) {
        if (data_len < 4 * sizeof(float)) {
            return -1;
        }

        float q0 = 0.0F;
        float q1 = 0.0F;
        float q2 = 0.0F;
        float q3 = 0.0F;
        std::memcpy(&q0, payload, sizeof(float));
        std::memcpy(&q1, payload + 4, sizeof(float));
        std::memcpy(&q2, payload + 8, sizeof(float));
        std::memcpy(&q3, payload + 12, sizeof(float));

        const Eigen::Quaterniond raw_rotation(q0, q1, q2, q3);
        const Eigen::Quaterniond fix_rotation(
            Eigen::AngleAxisd(3.14159265358979323846, Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond corrected_rotation = fix_rotation * raw_rotation;
        corrected_rotation.normalize();

        std::lock_guard<std::mutex> lock(state_mutex_);
        rotation_ = corrected_rotation;
        return 0;
    }

    return -1;
}
