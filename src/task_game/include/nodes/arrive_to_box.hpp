#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <cstdint>
#include <rclcpp/timer.hpp>

class Robot;

class ArriveToBoxAction : public BT::ActionNode {
public:
    // slot 区分本节点负责去抓第一块（平板箱）还是第二块（手上箱）。
    explicit ArriveToBoxAction(BoxSlot slot);

protected:
    BT::Status execute(BT& tree) override;

private:
    void ensure_stop_timer(Robot* context);
    void arm_stop_timer(Robot* context);

    BoxSlot slot_;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    std::uint64_t stop_timer_generation_{0};
};
