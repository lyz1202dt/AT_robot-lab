
#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <rclcpp/timer.hpp>

class Robot;

class ArriveToTargetAction : public BT::ActionNode {
public:
    ArriveToTargetAction();

protected:
    BT::Status execute(BT& tree) override;

private:
    rclcpp::TimerBase::SharedPtr stop_timer_;
};
