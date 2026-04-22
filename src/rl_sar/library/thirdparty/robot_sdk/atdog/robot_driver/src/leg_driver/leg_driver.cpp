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
        std::lock_guard<std::mutex> lock(state_mutex_);
        std::memcpy(&state_pack, data, size);
        if (state_pack.type == 0 && size == static_cast<int>(sizeof(DogStatePack0_t))) {
            
            return;
        }
        if (state_pack.type == 1 && size == static_cast<int>(sizeof(DogStatePack1_t))) {
            
            return;
        }
        first_update=false;
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
            for(j=0;j<3;j++)
                target_pack.leg[i].joint[j]=joint_default_kd;
            target_pack.leg[i].wheel=wheel_default_kd;
        }
        
    }
    return cdc_trans->send_struct(target_pack);
}

bool LegDriver::get_leg_state(std::array<LegState_t, 4>& legs_state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (first_update)
        return false;

    if(!enable_control_)
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
