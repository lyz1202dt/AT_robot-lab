#ifndef __LEG_DRIVER_HPP__
#define __LEG_DRIVER_HPP__


#include "cdc_trans.hpp"
#include "data_pack.h"
#include "kalman_filter.hpp"
#include <thread>
#include <memory>
#include <fstream>
#include <chrono>
#include <mutex>


class LegDriver
{


public:
    LegDriver(uint16_t vid=0x0483,uint16_t pid=0x5740);
    ~LegDriver();
    bool set_leg_target(const std::array<LegTarget_t,4> &legs_target);
    bool get_leg_state(std::array<LegState_t,4> &legs_state);
    bool enable_control(bool cmd);

    bool get_imu_state(std::array<float,4> &q,std::array<float,4> &angular_vel,std::array<float,4> &acc);
private:
    bool exit_thread{false};
    bool first_update{true};
    bool enable_control_{false};
    bool motor_has_error{false};


    std::unique_ptr<CDCTrans> cdc_trans;    //CDC传输类
    std::unique_ptr<std::thread> usb_event_handle_thread;   //CDC事件处理线程

    union {
        int type;
        DogStatePack3_t pack3;
    }state_pack;
    
    
    // 卡尔曼滤波器：为每个电机的力矩提供滤波
    KalmanFilter torque_filters[4][3];
    KalmanFilter wheel_torque_filters[4];

    //电机数据缓存
    float filtered_torque[4][3];
    float filtered_omega[4][3];
    float position[4][3];

    float filtered_wheel_torque[4];
    float filtered_wheel_omega[4];

    mutable std::mutex state_mutex_;

    float joint_default_kd{8.0};
    float wheel_default_kd{8.0};
};

#endif
