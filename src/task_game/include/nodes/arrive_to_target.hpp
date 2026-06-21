
#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

#include <atomic>
#include <cstdint>
#include <rclcpp/timer.hpp>

class Robot;

class ArriveToTargetAction : public BT::ActionNode {
public:
    ArriveToTargetAction();

protected:
    BT::Status execute(BT& tree) override;

private:
    void ensure_stop_timer(Robot* context);
    void arm_stop_timer(Robot* context);
    void reset_run_state();

    rclcpp::TimerBase::SharedPtr stop_timer_;
    std::uint64_t stop_timer_generation_{0};
    bool target_loaded_{false};
    bool waiting_resume_{false};
    std::atomic_bool finished_{false};
    std::atomic_bool success_{false};
};
