
#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

class Robot;

class ArriveToBoxAction : public BT::ActionNode {
public:
    ArriveToBoxAction();

protected:
    BT::Status execute(BT& tree) override;
};
