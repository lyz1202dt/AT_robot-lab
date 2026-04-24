#include "leg_driver/leg_driver.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>


// constexpr uint16_t kLegDriverVendorId = 0x0483;
// constexpr uint16_t kLegDriverProductId = 0x5740;


LegDriver::LegDriver(uint16_t vid,uint16_t pid) {
    cdc_trans = std::make_unique<CDCTrans>();
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) {
        if (data == nullptr || size < static_cast<int>(sizeof(int))) {
            return;
        }

        std::lock_guard<std::mutex> lock(state_mutex_);

        int pack_type = -1;
        std::memcpy(&pack_type, data, sizeof(pack_type));

        const bool is_pack0 = pack_type == 0 && size == static_cast<int>(sizeof(DogStatePack0_t));
        const bool is_pack1 = pack_type == 1 && size == static_cast<int>(sizeof(DogStatePack1_t));
        if (!is_pack0 && !is_pack1) {
            return;
        }

        if (is_pack0) {
            std::memcpy(&state_pack.pack0, data, sizeof(DogStatePack0_t));
        } else {
            std::memcpy(&state_pack.pack1, data, sizeof(DogStatePack1_t));
        }

        const LegState_t* legs_state = is_pack0 ? state_pack.pack0.leg : state_pack.pack1.leg;
        const bool need_reset_filter = first_update;

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 3; ++j) {
                position[i][j] = legs_state[i].joint[j].rad;
                filtered_omega[i][j] = legs_state[i].joint[j].omega;

                if (need_reset_filter) {
                    torque_filters[i][j].reset(legs_state[i].joint[j].torque);
                    filtered_torque[i][j] = legs_state[i].joint[j].torque;
                } else {
                    filtered_torque[i][j] = torque_filters[i][j].update(legs_state[i].joint[j].torque);
                }
            }

            filtered_wheel_omega[i] = legs_state[i].wheel.omega;
            if (need_reset_filter) {
                wheel_torque_filters[i].reset(legs_state[i].wheel.torque);
                filtered_wheel_torque[i] = legs_state[i].wheel.torque;
            } else {
                filtered_wheel_torque[i] = wheel_torque_filters[i].update(legs_state[i].wheel.torque);
            }
        }

        first_update = false;
    });

    if (!cdc_trans->open(vid, pid)) {
        exit_thread = true;
        std::cerr << "[LegDriver] Failed to open USB CDC device "
                  << std::hex << vid << ":" << pid
                  << std::dec << std::endl;
        return;
    }

    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        while (!exit_thread) {
            cdc_trans->process_once();
        }
    });
}

bool get_imu_state(std::array<float,4> &q,std::array<float,4> &angular_vel,std::array<float,4> &acc)
{
    //TOD:未完成
    return true;
}

LegDriver::~LegDriver() {
    exit_thread = true;

    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }

    if (cdc_trans) {
        cdc_trans->close();
    }
}

bool LegDriver::set_leg_target(const std::array<LegTarget_t, 4>& legs_target) {
    if (!cdc_trans) {
        return false;
    }

    DogTargetPack_t target_pack;
    target_pack.pack_type = 0;

    if (enable_control_) {  //正常模式
        std::copy(legs_target.begin(), legs_target.end(), std::begin(target_pack.leg));
    }
    else{   //安全阻尼模式
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++) {
                target_pack.leg[i].joint[j].rad = 0.0f;
                target_pack.leg[i].joint[j].omega = 0.0f;
                target_pack.leg[i].joint[j].torque = 0.0f;
                target_pack.leg[i].joint[j].kp = 0.0f;
                target_pack.leg[i].joint[j].kd = joint_default_kd;
            }
            target_pack.leg[i].wheel.omega = 0.0f;
            target_pack.leg[i].wheel.torque = 0.0f;
        }
        
    }
    return cdc_trans->send_struct(target_pack);
}

bool LegDriver::get_leg_state(std::array<LegState_t, 4>& legs_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (first_update)
        return false;

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            legs_state[i].joint[j].rad = position[i][j];
            legs_state[i].joint[j].omega = filtered_omega[i][j];
            legs_state[i].joint[j].torque = filtered_torque[i][j];
        }
        legs_state[i].wheel.omega = filtered_wheel_omega[i];
        legs_state[i].wheel.torque = filtered_wheel_torque[i];
    }

    return true;
}

bool LegDriver::enable_control(bool cmd) {
    enable_control_ = cmd;
    first_update=true;
    return enable_control_;
}
