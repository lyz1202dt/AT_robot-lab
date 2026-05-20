/**
@note 根据之前扫描的结果开始执行节点，检查存储的地图状态，首次执行时规划轨迹，之后进入该动作就进行导航点的再规划
*/


#pragma once

#include "nodes/msg.hpp"
#include "core/behavior_tree.hpp"

class Robot;

class HandlePlaneAction : public BT::ActionNode {
public:
    HandlePlaneAction();

protected:
    BT::Status execute(BT& tree) override;
};
