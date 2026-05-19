#pragma once

#include "actions/base_action.hpp"

class Robot;

class MoveToAction :public BaseAction<Robot>{
public:
    MoveToAction();
    std::string process(Robot* context, const std::string &last_status) override;
};
