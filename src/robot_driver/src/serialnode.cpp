#include "serialnode.hpp"
#include "cdc_trans.hpp"
#include "data_pack.h"
#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/logging.hpp>
#include <robot_msgs/msg/arm.hpp>
#include <robot_msgs/msg/arm4.hpp>
#include <robot_msgs/msg/vis.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>



using namespace std::chrono_literals;



ArmNode::ArmNode()
    : Node("arm_driver_node") {   

         exit_thread = false;
    
   
    plane_dst_pub = this->create_publisher<std_msgs::msg::Float32>("plane_dst", 10);

    arm_sub = this->create_subscription<robot_msgs::msg::Arm>(
        "myjoints_target", 10, std::bind(&ArmNode::armSubscribCb, this, std::placeholders::_1));

    air_sub = this->create_subscription<std_msgs::msg::Int32>(
        "air_pump_target", 10, std::bind(&ArmNode::airSubscribCb, this, std::placeholders::_1));

    

    cdc_trans = std::make_unique<CDCTrans>();          

                             // 创建CDC传输对象
    cdc_trans->regeiser_recv_cb([this](const uint8_t* data, int size) { // 注册接收回调
        if (data == nullptr || size != sizeof(plane_dst_state_pack_t)) 
        {
            RCLCPP_INFO(this->get_logger(), "数据包错误,size = %d",size);
            return;
        }
        const auto* pack = reinterpret_cast<const plane_dst_state_pack_t*>(data);
        RCLCPP_INFO(this->get_logger(), "接收到了数据包,包头%x,数据%f", size, pack->plane_dst);
        if (pack->pack_type == 0x10) {       // 确认包类型正确
            publish_plane_dst(pack);      // 同步发布机械臂 USB 上报的平板激光测距
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




void ArmNode::publish_plane_dst(const plane_dst_state_pack_t *arm_state){
    if (plane_dst_pub == nullptr || arm_state == nullptr)
    {
        return;
    }

    plane_dst_samples[plane_dst_sample_index] = arm_state->plane_dst;
    plane_dst_sample_index = (plane_dst_sample_index + 1) % plane_dst_samples.size();
    if (plane_dst_sample_count < plane_dst_samples.size()) {
        ++plane_dst_sample_count;
    }
    if (plane_dst_sample_count < plane_dst_samples.size()) {
        return;
    }

    float sum = plane_dst_samples[0];
    float min_value = plane_dst_samples[0];
    float max_value = plane_dst_samples[0];
    for (size_t i = 1; i < plane_dst_samples.size(); ++i) {
        const float value = plane_dst_samples[i];
        sum += value;
        min_value = std::min(min_value, value);
        max_value = std::max(max_value, value);
    }

    std_msgs::msg::Float32 msg;
    msg.data = (sum - min_value - max_value) / static_cast<float>(plane_dst_samples.size() - 2);
    plane_dst_pub->publish(msg);
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
