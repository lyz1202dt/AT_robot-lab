
#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <rclcpp/timer.hpp>

class Robot;

class ArriveToBoxAction : public BT::ActionNode {
public:
    ArriveToBoxAction();

protected:
    BT::Status execute(BT& tree) override;

private:
    rclcpp::TimerBase::SharedPtr stop_timer_;
};
