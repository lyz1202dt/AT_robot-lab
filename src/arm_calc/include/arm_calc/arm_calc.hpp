#pragma once

#include "arm_calc/common_types.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace arm_calc {

class ArmCalc {
public:
    explicit ArmCalc(const KDL::Chain& chain);
    ~ArmCalc() = default;

    JointVector joint_pos(const CartesianPose& pose, int* result);
    JointVector joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos);

    JointVector joint_pos_as(const CartesianPose& pose);

    CartesianPose end_pose(const JointVector& joint_pos);

private:
    static CartesianPose from_kdl_frame(const KDL::Frame& frame);
    static KDL::JntArray to_kdl_joints(const JointVector& joints);
    static JointVector from_kdl_joints(const KDL::JntArray& joints);

    KDL::Chain chain_;
    KDL::ChainFkSolverPos_recursive fk_solver_;
    KDL::ChainIkSolverPos_LMA ik_solver_;

    KDL::JntArray last_joint_solution_;
};

}  // namespace arm_calc
