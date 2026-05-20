/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，生成决策
*/


#pragma once

#include "core/behavior_tree.hpp"

class Robot;

class CatchBoxAction : public BT::ActionNode {
public:
    CatchBoxAction();

protected:
    BT::Status execute(BT& tree) override;
};
