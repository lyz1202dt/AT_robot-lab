#include "arm_calc/arm_calc.hpp"

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace arm_calc {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1e-9;

struct PlanarIkGeometry {
    Eigen::Vector2d base_xy{Eigen::Vector2d::Zero()};
    Eigen::Vector2d shoulder{Eigen::Vector2d::Zero()};
    Eigen::Vector2d upper_arm{Eigen::Vector2d::Zero()};
    Eigen::Vector2d forearm{Eigen::Vector2d::Zero()};
    Eigen::Vector2d tool{Eigen::Vector2d::Zero()};
    double zero_tip_pitch{0.0};
    bool valid{false};
};

Eigen::Vector<double, 6> IkWeights() {
    return Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 1.0, 0.0);
}

bool IsMovingJoint(const KDL::Joint& joint) {
    return joint.getType() != KDL::Joint::Fixed;
}

Eigen::Vector3d ToEigen(const KDL::Vector& vector) {
    return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

Eigen::Vector2d ToPlanar(const Eigen::Vector3d& vector) {
    return Eigen::Vector2d(vector.x(), vector.z());
}

Eigen::Vector2d RotatePlanar(const Eigen::Vector2d& vector, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Eigen::Vector2d(c * vector.x() + s * vector.y(), -s * vector.x() + c * vector.y());
}

double NormalizeAngleNear(double angle, double reference) {
    while (angle - reference > kPi) {
        angle -= kTwoPi;
    }
    while (angle - reference < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

double NormalizeAngleAtMost(double angle, double upper_limit) {
    while (angle > upper_limit) {
        angle -= kTwoPi;
    }
    while (angle <= upper_limit - kTwoPi) {
        angle += kTwoPi;
    }
    return angle;
}

double Clamp(double value, double lower, double upper) {
    return std::max(lower, std::min(value, upper));
}

double FramePitch(const KDL::Frame& frame) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    frame.M.GetRPY(roll, pitch, yaw);
    return pitch;
}

PlanarIkGeometry ExtractPlanarIkGeometry(const KDL::Chain& chain) {
    PlanarIkGeometry geometry;
    if (chain.getNrOfJoints() != kJointDoF) {
        return geometry;
    }

    std::array<Eigen::Vector3d, kJointDoF> joint_origins{};
    KDL::Frame frame = KDL::Frame::Identity();
    std::size_t joint_index = 0;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& segment = chain.getSegment(i);
        frame = frame * segment.pose(0.0);

        if (IsMovingJoint(segment.getJoint())) {
            if (joint_index >= kJointDoF) {
                return geometry;
            }
            joint_origins[joint_index] = ToEigen(frame.p);
            ++joint_index;
        }
    }

    if (joint_index != kJointDoF) {
        return geometry;
    }

    const Eigen::Vector3d tip = ToEigen(frame.p);
    geometry.base_xy = joint_origins[0].head<2>();
    geometry.shoulder = ToPlanar(joint_origins[1]);
    geometry.upper_arm = ToPlanar(joint_origins[2] - joint_origins[1]);
    geometry.forearm = ToPlanar(joint_origins[3] - joint_origins[2]);
    geometry.tool = ToPlanar(tip - joint_origins[3]);
    geometry.zero_tip_pitch = FramePitch(frame);
    geometry.valid = geometry.upper_arm.norm() > kEpsilon && geometry.forearm.norm() > kEpsilon;
    return geometry;
}

} // namespace

ArmCalc::ArmCalc(const KDL::Chain& chain)
    : chain_(chain)
    , fk_solver_(chain_)
    , ik_solver_(chain_, IkWeights(), 1e-6, 200, 1e-10)
    , last_joint_solution_(chain_.getNrOfJoints()) {
    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
        last_joint_solution_(i) = 0.0;
    }
}

JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result) {
    return joint_pos(pose, result, from_kdl_joints(last_joint_solution_));
}

JointVector ArmCalc::joint_pos(const CartesianPose& pose, int* result, const JointVector& seed_joint_pos) {
    KDL::JntArray seed = to_kdl_joints(seed_joint_pos);
    KDL::Frame frame;

    const Eigen::Vector3d& target_pos = pose.position;
    frame.p = KDL::Vector(target_pos[0], target_pos[1], target_pos[2]);

    const Eigen::Quaterniond input_quat = NormalizeQuaternion(pose.orientation);
    KDL::Rotation input_rot =
        KDL::Rotation::Quaternion(input_quat.x(), input_quat.y(), input_quat.z(), input_quat.w());
    [[maybe_unused]] double roll = 0.0;
    double pitch                 = 0.0;
    [[maybe_unused]] double yaw  = 0.0;
    input_rot.GetRPY(roll, pitch, yaw);

    const double prev_pitch = last_joint_solution_(3);
    if (std::abs(pitch - prev_pitch) > kPi / 2.0) {
        if (pitch > prev_pitch) {
            pitch -= 2.0 * kPi;
        } else {
            pitch += 2.0 * kPi;
        }
    }

    double ax  = -target_pos[1];
    double ay  = target_pos[0];
    const double len = std::sqrt(ax * ax + ay * ay);

    if (len < 1e-8) {
        frame.M = KDL::Rotation::RPY(0.0, pitch, 0.0);
    } else {
        ax /= len;
        ay /= len;

        const double half = pitch * 0.5;
        double qw         = std::cos(half);
        double qx         = ax * std::sin(half);
        double qy         = ay * std::sin(half);
        double qz         = 0.0;

        const double qnorm = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        if (qnorm > 1e-12) {
            qw /= qnorm;
            qx /= qnorm;
            qy /= qnorm;
            qz /= qnorm;
        }

        frame.M = KDL::Rotation::Quaternion(qx, qy, qz, qw);
    }

    int local_result = -1;
    int& ik_result   = result == nullptr ? local_result : *result;
    ik_result        = ik_solver_.CartToJnt(seed, frame, seed);

    if (ik_result >= 0) {
        last_joint_solution_ = seed;
    }

    return from_kdl_joints(seed);
}

CartesianPose ArmCalc::end_pose(const JointVector& joint_pos) {
    KDL::Frame frame;
    fk_solver_.JntToCart(to_kdl_joints(joint_pos), frame);
    return from_kdl_frame(frame);
}

JointVector ArmCalc::joint_pos_as(const CartesianPose& pose) {
    const JointVector last_solution = from_kdl_joints(last_joint_solution_);
    const PlanarIkGeometry geometry = ExtractPlanarIkGeometry(chain_);
    if (!geometry.valid) {
        return last_solution;
    }

    const Eigen::Vector3d& target_pos = pose.position;
    const double dx = target_pos.x() - geometry.base_xy.x();
    const double dy = target_pos.y() - geometry.base_xy.y();
    const double radial_distance = std::hypot(dx, dy);

    double q1 = last_solution[0];
    if (radial_distance > kEpsilon) {
        q1 = -std::atan2(dy, dx);
        q1 = NormalizeAngleNear(q1, last_solution[0]);
    }

    const double c1 = std::cos(q1);
    const double s1 = std::sin(q1);
    const double local_x = geometry.base_xy.x() + c1 * dx - s1 * dy;

    const Eigen::Quaterniond input_quat = NormalizeQuaternion(pose.orientation);
    KDL::Rotation input_rot =
        KDL::Rotation::Quaternion(input_quat.x(), input_quat.y(), input_quat.z(), input_quat.w());
    [[maybe_unused]] double roll = 0.0;
    double target_pitch = 0.0;
    [[maybe_unused]] double yaw = 0.0;
    input_rot.GetRPY(roll, target_pitch, yaw);

    const double last_tip_pitch = last_solution[1] - last_solution[2] + last_solution[3];
    const double tip_pitch = NormalizeAngleNear(target_pitch - geometry.zero_tip_pitch, last_tip_pitch);
    const Eigen::Vector2d target_planar(local_x, target_pos.z());
    const Eigen::Vector2d wrist_target =
        target_planar - geometry.shoulder - RotatePlanar(geometry.tool, tip_pitch);

    const double upper_length = geometry.upper_arm.norm();
    const double forearm_length = geometry.forearm.norm();
    const double wrist_distance = wrist_target.norm();
    const double cos_elbow =
        (wrist_distance * wrist_distance - upper_length * upper_length - forearm_length * forearm_length) /
        (2.0 * upper_length * forearm_length);

    if (cos_elbow < -1.0 - 1e-6 || cos_elbow > 1.0 + 1e-6 || wrist_distance < kEpsilon) {
        return last_solution;
    }

    const double upper_zero_angle = std::atan2(geometry.upper_arm.y(), geometry.upper_arm.x());
    const double forearm_zero_angle = std::atan2(geometry.forearm.y(), geometry.forearm.x());
    const double zero_elbow_delta = forearm_zero_angle - upper_zero_angle;
    const double clamped_cos_elbow = Clamp(cos_elbow, -1.0, 1.0);
    const double elbow_delta = std::acos(clamped_cos_elbow);
    const double wrist_angle = std::atan2(wrist_target.y(), wrist_target.x());

    JointVector best_solution = last_solution;
    double best_distance = std::numeric_limits<double>::max();

    for (const double signed_elbow_delta : {elbow_delta, -elbow_delta}) {
        const double upper_angle =
            wrist_angle -
            std::atan2(forearm_length * std::sin(signed_elbow_delta),
                       upper_length + forearm_length * std::cos(signed_elbow_delta));

        JointVector candidate;
        candidate[0] = q1;
        candidate[1] = upper_zero_angle - upper_angle;
        candidate[2] = signed_elbow_delta - zero_elbow_delta;
        candidate[3] = tip_pitch - candidate[1] + candidate[2];

        for (std::size_t i = 0; i < kJointDoF; ++i) {
            candidate[static_cast<int>(i)] =
                NormalizeAngleNear(candidate[static_cast<int>(i)], last_solution[static_cast<int>(i)]);
        }
        candidate[2] = NormalizeAngleAtMost(candidate[2], kPi);

        if (!std::isfinite(candidate[1]) || candidate[1] <= 0.0 || !std::isfinite(candidate[2]) || candidate[2] > kPi + 1e-9) {
            continue;
        }

        const double distance = (candidate - last_solution).squaredNorm();
        if (distance < best_distance) {
            best_distance = distance;
            best_solution = candidate;
        }
    }

    last_joint_solution_ = to_kdl_joints(best_solution);

    return best_solution;
}

CartesianPose ArmCalc::from_kdl_frame(const KDL::Frame& frame) {
    CartesianPose pose;
    pose.position = Eigen::Vector3d(frame.p.x(), frame.p.y(), frame.p.z());

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation = NormalizeQuaternion(Eigen::Quaterniond(w, x, y, z));
    return pose;
}

KDL::JntArray ArmCalc::to_kdl_joints(const JointVector& joints) {
    KDL::JntArray output(static_cast<unsigned int>(kJointDoF));
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output(static_cast<unsigned int>(i)) = joints[static_cast<int>(i)];
    }
    return output;
}

JointVector ArmCalc::from_kdl_joints(const KDL::JntArray& joints) {
    JointVector output = JointVector::Zero();
    for (std::size_t i = 0; i < kJointDoF; ++i) {
        output[static_cast<int>(i)] = joints(static_cast<unsigned int>(i));
    }
    return output;
}

} // namespace arm_calc
