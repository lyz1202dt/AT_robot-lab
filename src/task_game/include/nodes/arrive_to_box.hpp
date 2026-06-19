
#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <cstdint>
#include <rclcpp/timer.hpp>

class Robot;

class ArriveToBoxAction : public BT::ActionNode {
public:
    ArriveToBoxAction();

protected:
    BT::Status execute(BT& tree) override;

private:
    void ensure_stop_timer(Robot* context);
    void arm_stop_timer(Robot* context);

    rclcpp::TimerBase::SharedPtr stop_timer_;
    std::uint64_t stop_timer_generation_{0};
};
