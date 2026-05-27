/**
@note 调用海康相机识别并计算算术题，得到结果，调用USB相机扫描环境箱子位置，将箱子位置和高分区域入栈，切入
*/


#pragma once

#include "core/behavior_tree.hpp"

class Robot;

class PlaceBoxAction : public BT::ActionNode {
public:
    PlaceBoxAction();

protected:
    BT::Status execute(BT& tree) override;
    std::array<float, 2> dst_box_pos_;
    bool place_at_second_floor_ = false;
};
