#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <cstdint>
#include <rclcpp/timer.hpp>

class Robot;

class ArriveToTargetAction : public BT::ActionNode {
public:
    // slot 区分本节点负责去放手上箱 box1 还是平板箱 box0。
    explicit ArriveToTargetAction(BoxSlot slot);

protected:
    BT::Status execute(BT& tree) override;

private:
    void ensure_stop_timer(Robot* context);
    void arm_stop_timer(Robot* context);

    BoxSlot slot_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    std::uint64_t stop_timer_generation_{0};
};
