#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_msgs/msg/arm.hpp>
#include <robot_msgs/msg/arm4.hpp>
#include <robot_msgs/msg/vis.hpp>
#include <robot_msgs/msg/int.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>



using namespace std::chrono_literals;



ArmNode::ArmNode()
    : Node("arm_driver_node") {   

         exit_thread = false;
    
   
    hand_distance_pub = this->create_publisher<robot_msgs::msg::Int>("red_distance", 10);

    arm_sub = this->create_subscription<robot_msgs::msg::Arm>(
        "myjoints_target", 10, std::bind(&ArmNode::armSubscribCb, this, std::placeholders::_1));

    air_sub = this->create_subscription<std_msgs::msg::Int32>(
        "air_pump_target", 10, std::bind(&ArmNode::airSubscribCb, this, std::placeholders::_1));

    

    cdc_trans = std::make_unique<CDCTrans>();          

                             // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        // RCLCPP_INFO(this->get_logger(), "接收到了数据包,长度%d", size);
        if (size == sizeof(state_pack_t)) // 验证包长度，可以被视作四条腿的状态数据包
        {
            const state_pack_t* pack = reinterpret_cast<const state_pack_t*>(data);
            if (pack->pack_type == 1)         // 确认包类型正确
                publishredState(pack);        // 一旦接收，立即发布状态
                

            else
                RCLCPP_ERROR(this->get_logger(), "接收到错误的数据包类型%d", pack->pack_type);
        }
    });
      

    if (!cdc_trans->open(0x0484, 0x5740))     // 开启USB_CDC传输接口(0x0484  机械臂下位机，0x0483是狗腿下位机)
        exit_thread = true;

    // 创建线程处理CDC消息（在 open 之后、publisher 创建之后）
    usb_event_handle_thread = std::make_unique<std::thread>([this]() {
        do {
            cdc_trans->process_once();
        } while (!exit_thread);
    });

    arm_target.pack_type=0x01;
    fresh_timer=this->create_wall_timer(
        std::chrono::duration<double>(0.02),   // 定时器间隔
        [this](){
            if(!first_update)
                cdc_trans->send_struct(arm_target);
        });

    base_time=this->get_clock()->now();
}




ArmNode::~ArmNode() {
    // 请求线程退出并等待其结束，保证安全关闭
    exit_thread = true;
    if (usb_event_handle_thread && usb_event_handle_thread->joinable()) {
        usb_event_handle_thread->join();
    }
    if (cdc_trans) {
        cdc_trans->close();
    }
}


void ArmNode::publishredState(const state_pack_t *arm_state){

     if (arm_state->red_distance == 0)
    {
        return;
    }

    robot_msgs::msg::Int msg;
    msg.data = arm_state->red_distance;
    hand_distance_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "\033[35m发布了红外距离 %d\033[0m", arm_state->red_distance);
       
}



void ArmNode::armSubscribCb(const robot_msgs::msg::Arm& msg) {
   
    arm_target.servo1.up=msg.motor[3].rad;
    arm_target.servo1.low=msg.motor[2].rad;
    arm_target.servo1.down=msg.motor[1].rad;
    arm_target.rob01.except_pos=msg.motor[0].rad;
    first_update = false;
}

void ArmNode::airSubscribCb(const std_msgs::msg::Int32& msg) {
   arm_target.air_pump = msg.data;
}
