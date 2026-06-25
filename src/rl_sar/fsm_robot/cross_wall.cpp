#include "cross_wall.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath> 


using Vector3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;

Robot_t::Robot_t(const std::string& urdf_file_path)
    : lf_base_offset(0.25, 0.21, -0.28)
    , rf_base_offset(0.25, -0.21, -0.28)
    , lb_base_offset(-0.25, 0.21, -0.28)
    , rb_base_offset(-0.25, -0.21, -0.28)
    , kp{80.0, 80.0, 80.0}
    , kd{3.0, 3.0, 3.0} {
    // 从文件直接读取 URDF
    std::ifstream urdf_file(urdf_file_path);
    if (!urdf_file.is_open()) {
        throw std::runtime_error("无法打开 URDF 文件: " + urdf_file_path);
    }

    std::stringstream buffer;
    buffer << urdf_file.rdbuf();
    urdf_xml = buffer.str();
    urdf_file.close();

    if (urdf_xml.empty()) {
        throw std::runtime_error("URDF 文件为空或读取失败: " + urdf_file_path);
    }

    // // 将四个轮子关节从continuous类型改为fixed类型，以便KDL能够求解
    // std::vector<std::string> wheel_joints = {"lf_joint4", "rf_joint4", "lb_joint4", "rb_joint4"};
    // for (const auto& joint_name : wheel_joints) {
    //     std::string search_pattern  = "<joint\n        name=\"" + joint_name + "\"\n        type=\"continuous\">";
    //     std::string replace_pattern = "<joint\n        name=\"" + joint_name + "\"\n        type=\"fixed\">";
    //     size_t pos                  = urdf_xml.find(search_pattern);
    //     if (pos != std::string::npos) {
    //         urdf_xml.replace(pos, search_pattern.length(), replace_pattern);
    //     }
    // }

    kdl_parser::treeFromString(urdf_xml, tree); // 解析四条腿的KDL树结构
    tree.getChain("base", "FL_foot", lf_leg_chain);
    tree.getChain("base", "FR_foot", rf_leg_chain);
    tree.getChain("base", "RL_foot", lb_leg_chain);
    tree.getChain("base", "RR_foot", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc             = std::make_shared<LegCalc>(lf_leg_chain, kp, kd, wheel_kd);
    rf_leg_calc             = std::make_shared<LegCalc>(rf_leg_chain, kp, kd, wheel_kd);
    lb_leg_calc             = std::make_shared<LegCalc>(lb_leg_chain, kp, kd, wheel_kd);
    rb_leg_calc             = std::make_shared<LegCalc>(rb_leg_chain, kp, kd, wheel_kd);
    lf_leg_calc->pos_offset = lf_base_offset;
    rf_leg_calc->pos_offset = rf_base_offset;
    lb_leg_calc->pos_offset = lb_base_offset;
    rb_leg_calc->pos_offset = rb_base_offset;

    const auto front_x = 0.5 * (lf_base_offset.x() + rf_base_offset.x());
    const auto rear_x = 0.5 * (lb_base_offset.x() + rb_base_offset.x());
    const auto left_y = 0.5 * (lf_base_offset.y() + lb_base_offset.y());
    const auto right_y = 0.5 * (rf_base_offset.y() + rb_base_offset.y());
    const auto x_span = std::abs(front_x - rear_x);
    const auto y_span = std::abs(left_y - right_y);
    if (x_span > 1e-6 && y_span > 1e-6) {
        const auto tree_segments = tree.getSegments();
        auto body_it = tree_segments.find("body_link");
        double body_mass = 0.0;
        double body_com_x = 0.0;
        double body_com_y = 0.0;
        if (body_it != tree_segments.end()) {
            const auto& body_inertia = body_it->second.segment.getInertia();
            body_mass = body_inertia.getMass();
            body_com_x = body_inertia.getCOG().x();
            body_com_y = body_inertia.getCOG().y();
        }

        const double front_ratio = std::clamp((body_com_x - rear_x) / x_span, 0.0, 1.0);
        const double rear_ratio = 1.0 - front_ratio;
        const double left_ratio = std::clamp((body_com_y - right_y) / y_span, 0.0, 1.0);
        const double right_ratio = 1.0 - left_ratio;
        const double total_load = body_mass * 9.81;

        robot_lf_grivate = total_load * front_ratio * left_ratio;
        robot_rf_grivate = total_load * front_ratio * right_ratio;
        robot_lb_grivate = total_load * rear_ratio * left_ratio;
        robot_rb_grivate = total_load * rear_ratio * right_ratio;

        robot_lf_grivate = 30;
        robot_rf_grivate = 30;
        robot_lb_grivate = 38;
        robot_rb_grivate = 38;
    }
}

LegCalc::~LegCalc() {
    // 默认析构函数实现
}
LegCalc::LegCalc(
     KDL::Chain& chain, const std::vector<double>& kp_list , const std::vector<double>& kd_list,
    double wheel_kd_param)
    : chain(chain)
    , fk_solver(chain)
    , jacobain_solver(chain)
    , jdot_solver(chain)
    , vel_solver(chain)
    , ik_pos_solver(chain, Eigen::Vector<double, 6>(1.0, 1.0, 1.0, 0.0, 0.0, 0.0), 1e-6, 150, 1e-10)
    , dynamin_solver(chain, KDL::Vector(0, 0, -9.81)) {
    _temp_joint3_array.resize(3); // 提前resize需要用到的KDL::JntArray防止运行时频繁申请/释放内存
    _temp2_joint3_array.resize(3);
    last_exp_joint_pos.resize(3);
    temp_jacobain.resize(3);
    _temp_joint3_vel_array.resize(3);

    C.resize(3);
    G.resize(3);
    M.resize(3);

    last_exp_joint_pos(0) = 0.0;
    last_exp_joint_pos(1) = 0.0;
    last_exp_joint_pos(2) = 0.0;

    // set_joint_pd(0,3.0,0.17);   //设置默认参数
    // set_joint_pd(1,2.8,0.14);
    // set_joint_pd(2,2.8,0.11);

    // // set_joint_pd(0,50.0,3.0);   //设置默认参数
    // // set_joint_pd(1,50.0,3.0);
    // // set_joint_pd(2,50.0,3.0);

    // set_joint_pd(3,0.0,0.5);

    if (kp_list.size() != 3 || kd_list.size() != 3) {
        throw std::runtime_error("PD param size must be 3");
    }


    for (int i = 0; i < 3; i++) {
        this->kp[i] = kp_list[i];
        this->kd[i] = kd_list[i];
    }

    this->wheel_kd = wheel_kd_param;
}

void LegCalc::set_init_joint_pos(const Eigen::Vector3d init_joint_pos) {
    last_exp_joint_pos(0) = init_joint_pos[0];
    last_exp_joint_pos(1) = init_joint_pos[1];
    last_exp_joint_pos(2) = init_joint_pos[2];
}

Eigen::Matrix<double, 3, 3> LegCalc::get_3x3_jacobian_(const KDL::Jacobian& full_jacobian) // 只关心前三行的映射关系
{
    Eigen::Matrix<double, 3, 3> jacobian_3x3;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            jacobian_3x3(i, j) = full_jacobian(i, j);
        }
    }
    return jacobian_3x3;
}


Eigen::Vector3d LegCalc::joint_pos(const Eigen::Vector3d& foot_pos, int* result) {
    KDL::Frame frame;
    Eigen::Vector3d temp = foot_pos + pos_offset;
    frame.p.x(temp[0]);
    frame.p.y(temp[1]);
    frame.p.z(temp[2]);
    frame.M = KDL::Rotation::Identity();

    *result = ik_pos_solver.CartToJnt(last_exp_joint_pos, frame, _temp_joint3_array);
    if (*result == 0)                                                                      // 缓存本次计算结果,方便下一次迭代
        last_exp_joint_pos = _temp_joint3_array;
    return {_temp_joint3_array(0), _temp_joint3_array(1), _temp_joint3_array(2)};
}

/**
    @brief 计算关节角加速度
    @param joint_rad 关节角度向量
    @param joint_vel 关节角速度
    @param foot_acc  期望的足端加速度
    @return 关节角加速度向量
 */
Eigen::Vector3d LegCalc::joint_acc(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_vel, Eigen::Vector3d foot_acc) {
    _temp_joint3_array.data          = joint_rad;
    _temp2_joint3_array.data         = joint_vel;
    _temp_joint3_vel_array.q.data    = joint_rad;
    _temp_joint3_vel_array.qdot.data = joint_vel;

    // 计算雅可比矩阵J
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    jdot_solver.JntToJacDot(_temp_joint3_vel_array, _temp_jdot_qd);

    Eigen::Matrix3d Jac = get_3x3_jacobian_(temp_jacobain);
    Vector3D jdot_dq_eigen;
    for (int i = 0; i < 3; i++) {
        jdot_dq_eigen[i] = _temp_jdot_qd(i);
    }
    return Jac.completeOrthogonalDecomposition().solve(foot_acc - jdot_dq_eigen);
}

Eigen::Vector3d LegCalc::joint_pos_setarray(const Eigen::Vector3d init_joint_pos_) {
    _temp_joint3_array(0) = init_joint_pos_[0];
    _temp_joint3_array(1) = init_joint_pos_[1];
    _temp_joint3_array(2) = init_joint_pos_[2];
    return init_joint_pos_;
}


Eigen::Vector3d
    LegCalc::foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_torque, const Eigen::Vector3d& forward_torque) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    auto jacobian = get_3x3_jacobian_(temp_jacobain);

    return jacobian.transpose().inverse() * (joint_torque - forward_torque);
}


/**
    @brief 计算足端位置
    @param joint_rad 关节角度向量
    @return 当前足端位置
 */
Eigen::Vector3d LegCalc::foot_pos(const Eigen::Vector3d& joint_rad) {
    KDL::Frame frame;
    _temp_joint3_array(0) = joint_rad[0]; // 避免运行时动态分配内存，提高效率
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];

    int fk_result = fk_solver.JntToCart(_temp_joint3_array, frame);

                                          // 添加调试：检查 FK 计算结果
#ifdef DEBUG_FK
    std::cout << "[FK DEBUG] Joint angles: [" << joint_rad[0] << ", " << joint_rad[1] << ", " << joint_rad[2] << "]" << std::endl;
    std::cout << "[FK DEBUG] FK result code: " << fk_result << std::endl;
    std::cout << "[FK DEBUG] Frame position: [" << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "]" << std::endl;
    std::cout << "[FK DEBUG] pos_offset: [" << pos_offset[0] << ", " << pos_offset[1] << ", " << pos_offset[2] << "]" << std::endl;
#endif

    Eigen::Vector3d temp;
    temp[0] = frame.p.x();
    temp[1] = frame.p.y();
    temp[2] = frame.p.z();

    return temp - pos_offset; // temp是在机器人坐标系下的足端位置，要转换成支撑相中型点的坐标输出
}

Eigen::Vector3d LegCalc::joint_vel(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_vel) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
   // 检查雅可比矩阵是否接近奇异
    double det = jacobian.determinant();
    if (std::abs(det) < 1e-6) {
        // 雅可比矩阵奇异，返回零速度
        return Eigen::Vector3d::Zero();
    }
    
    // 使用完整旋转分解求解，比直接 inverse 更稳定
    return jacobian.fullPivLu().solve(foot_vel);
}

/**
    @brief 足端期望力->计算关节力矩
    @param joint_rad 关节角度
    @param joint_force 关节末端期望力
    @return 关节空间下的力矩
 */
Eigen::Vector3d LegCalc::joint_torque_foot_force(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& foot_force) {
    _temp_joint3_array(0) = joint_rad[0];
    _temp_joint3_array(1) = joint_rad[1];
    _temp_joint3_array(2) = joint_rad[2];
    jacobain_solver.JntToJac(_temp_joint3_array, temp_jacobain);
    Eigen::Matrix<double, 3, 3> jacobian = get_3x3_jacobian_(temp_jacobain);
    Eigen::Vector3d torque(foot_force(0), foot_force(1), foot_force(2));
    return jacobian.transpose() * torque;
}

Eigen::Vector3d
    LegCalc::joint_torque_dynamic(const Eigen::Vector3d& joint_rad, const Eigen::Vector3d& joint_omega, const Eigen::Vector3d& foot_acc) {
    _temp_joint3_array(0)  = joint_rad[0];
    _temp_joint3_array(1)  = joint_rad[1];
    _temp_joint3_array(2)  = joint_rad[2];
    _temp2_joint3_array(0) = joint_omega[0];
    _temp2_joint3_array(1) = joint_omega[1];
    _temp2_joint3_array(2) = joint_omega[2];
    dynamin_solver.JntToGravity(_temp_joint3_array, G);
    dynamin_solver.JntToCoriolis(_temp_joint3_array, _temp2_joint3_array, C);
    dynamin_solver.JntToMass(_temp_joint3_array, M);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 3, 3> M_;
    Eigen::Matrix<double, 3, 1> C_, G_;

    for (int i = 0; i < 3; ++i) {
        C_(i) = C(i);
        G_(i) = G(i);
        for (int j = 0; j < 3; ++j) {
            M_(i, j) = M(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return M_ * joint_acc(joint_rad, joint_omega, foot_acc) + C_ + G_;
}

LegTarget LegCalc::signal_leg_calc(
    const Eigen::Vector3d& exp_cart_pos, const Eigen::Vector3d& exp_cart_vel, const Eigen::Vector3d& exp_cart_acc,
    const Eigen::Vector3d& exp_cart_force, Eigen::Vector3d* torque, const double wheel_vel, const double wheel_force) {

    int result;
    auto joint_rad    = joint_pos(exp_cart_pos, &result); // 一般这个位置不可能会迭代失败，所以不再对result进行处理
    auto joint_omega  = joint_vel(joint_rad, exp_cart_vel);
    auto joint_torque = joint_torque_foot_force(joint_rad, exp_cart_force);
    joint_torque += joint_torque_dynamic(joint_rad, joint_omega, exp_cart_acc);
    
        // 检查并修正 NaN/Inf 值
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(joint_omega[i]) || std::isinf(joint_omega[i])) {
            joint_omega[i] = 0.0;
        }
        if (std::isnan(joint_torque[i]) || std::isinf(joint_torque[i])) {
            joint_torque[i] = 0.0;
        }
    }
    LegTarget leg;

    // 关节0
    leg.joints[0].rad    = joint_rad[0];
    leg.joints[0].omega  = joint_omega[0];
    leg.joints[0].torque = joint_torque[0];
    leg.joints[0].kp     = kp[0];
    leg.joints[0].kd     = kd[0];

    // 关节1
    leg.joints[1].rad    = joint_rad[1];
    leg.joints[1].omega  = joint_omega[1];
    leg.joints[1].torque = joint_torque[1];
    leg.joints[1].kp     = kp[1];
    leg.joints[1].kd     = kd[1];

    // 关节2
    leg.joints[2].rad    = joint_rad[2];
    leg.joints[2].omega  = joint_omega[2];
    leg.joints[2].torque = joint_torque[2];
    leg.joints[2].kp     = kp[2];
    leg.joints[2].kd     = kd[2];

    // 轮子
    leg.wheel.omega  = wheel_vel / wheel_radius;
    leg.wheel.torque = wheel_force * wheel_radius;
    leg.wheel.kd     = wheel_kd;

    *torque = joint_torque;

    return leg;
}


/**
    @brief 设置关节kp和kd
    @param index 关节索引
    @param kp 关节kp
    @param kd 关节kd
    @return none
    @note index=0/1/2时，对应前三个关节。index=3时，对应轮子，此时只有kd参数有效，因为轮子没有kp
 */
void LegCalc::set_joint_pd(int index, double kp, double kd) {
    if (index < 3) {
        this->kp[index] = kp;
        this->kd[index] = kd;
    } else
        wheel_kd = kd;
}

/**
    @brief 得到关节kp和kd
    @param index 关节索引
    @param kp 关节kp
    @param kd 关节kd
    @return none
    @note index=0/1/2时，对应前三个关节。index=3时，对应轮子，此时只有kd参数有效，因为轮子没有kp
 */
void LegCalc::get_joint_pd(int index, double& kp, double& kd) {
    if (index < 3) {
        kp = this->kp[index];
        kd = this->kd[index];
    } else
        kd = wheel_kd;
}
