#pragma once

#include <vector>
#include <array>

struct MoveBoxPlan{
    std::vector<std::array<float,3>> catch_trajectory;    //轨迹点序列
    std::vector<std::array<float,3>> place_trajectory;    //轨迹点序列
    std::array<float,2> src_box_pos;
    std::array<float,2> dst_box_pos;
    bool place_at_second_floor;  //是不是要放置在第二层
};
