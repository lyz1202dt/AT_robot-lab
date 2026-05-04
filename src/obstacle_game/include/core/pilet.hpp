#pragma once

#include <string>
#include <Eigen/Dense>

class Pilet{
public:
    explicit Pilet(const std::string ymal_path);
    ~Pilet();

    //填写机器人当前状态
    void set_state(const Eigen::Vector2d &pos,const double &yaw);

    //更新机器人目标
    bool update_target(uint32_t &policy_id,Eigen::Vector3d exp_vel);

private:

};