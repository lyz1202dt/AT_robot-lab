#include "cross_wall.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#define DEBUG_FK 0

using Vector3D = Eigen::Vector3d;
using Vector2D = Eigen::Vector2d;

Robot_t::Robot_t(const std::string& urdf_file_path)
    : lf_base_offset(0.25, 0.18, -0.28)
    , rf_base_offset(0.25, -0.18, -0.28)
    , lb_base_offset(-0.25, 0.18, -0.28)
    , rb_base_offset(-0.25, -0.18, -0.28)
    , robot_lb_grivate(40.0)
    , robot_lf_grivate(32.0)
    , robot_rb_grivate(40.0)
    , robot_rf_grivate(32.0)
    , kp{3.0, 2.8, 2.8}
    , kd{0.17, 0.14, 0.11} {
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
    tree.getChain("body_link", "lf_link4", lf_leg_chain);
    tree.getChain("body_link", "rf_link4", rf_leg_chain);
    tree.getChain("body_link", "lb_link4", lb_leg_chain);
    tree.getChain("body_link", "rb_link4", rb_leg_chain);

    // 初始化狗腿解算器，定义足端中性点位置
    lf_leg_calc             = std::make_shared<LegCalc>(lf_leg_chain, kp, kd, wheel_kd);
    rf_leg_calc             = std::make_shared<LegCalc>(rf_leg_chain, kp, kd, wheel_kd);
    lb_leg_calc             = std::make_shared<LegCalc>(lb_leg_chain, kp, kd, wheel_kd);
    rb_leg_calc             = std::make_shared<LegCalc>(rb_leg_chain, kp, kd, wheel_kd);
    lf_leg_calc->pos_offset = lf_base_offset;
    rf_leg_calc->pos_offset = rf_base_offset;
    lb_leg_calc->pos_offset = lb_base_offset;
    rb_leg_calc->pos_offset = rb_base_offset;
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
    return jacobian.inverse() * foot_vel;
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



bool use_limit_lf = false;
bool use_limit_rf = false;
bool use_limit_lb = false;
bool use_limit_rb = false;


std::tuple<Vector3D, double> Cross_WallState::get_robot_mass_info(
    const Vector3D& lf_joint_pos, const Vector3D& rf_joint_pos, const Vector3D& lb_joint_pos, const Vector3D& rb_joint_pos) {
    // 使用 KDL 计算全身质心（在 body_link 坐标系下）
    // 注意：这里用各 link 的 RigidBodyInertia 来做质量加权平均。
    //      需要 URDF 中每个 link 都有 inertial 标签，否则质量会为 0。

    auto accumulate_chain_com = [](const KDL::Chain& chain, const Vector3D& q_eigen, double& mass_sum, KDL::Vector& com_sum) {
        KDL::JntArray q(chain.getNrOfJoints());
        for (unsigned int i = 0; i < chain.getNrOfJoints() && i < 3; ++i) {
            q(i) = q_eigen[i];
        }

        KDL::Frame T = KDL::Frame::Identity();

        unsigned int joint_idx = 0;
        for (unsigned int seg_idx = 0; seg_idx < chain.getNrOfSegments(); ++seg_idx) {
            const auto& seg = chain.getSegment(seg_idx);

            // 计算该段末端在基座下的位姿
            if (seg.getJoint().getType() != KDL::Joint::None) {
                T = T * seg.pose(q(joint_idx));
                joint_idx++;
            } else {
                T = T * seg.pose(0.0);
            }

            // 该 segment 的刚体惯量（在 segment 坐标系下）
            const KDL::RigidBodyInertia& rbi = seg.getInertia();
            const double m                   = rbi.getMass();
            if (m <= 0.0) {
                continue;
            }

            // COM 在 segment 坐标系下的位置
            const KDL::Vector c_seg = rbi.getCOG();
            // 转到基座坐标系
            const KDL::Vector c_base = T * c_seg;

            mass_sum += m;
            com_sum = com_sum + c_base * m;
        }
    };

    double mass_sum = 0.0;
    KDL::Vector com_sum(0.0, 0.0, 0.0);

    // 4 条腿分别从 body_link 到足端 link4；它们共享 body_link，但各自 inertia 不重复（body_link 本体要单独加）
    accumulate_chain_com(robot->lf_leg_chain, lf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rf_leg_chain, rf_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->lb_leg_chain, lb_joint_pos, mass_sum, com_sum);
    accumulate_chain_com(robot->rb_leg_chain, rb_joint_pos, mass_sum, com_sum);

    // 加上 body_link 自身的质量与质心（树中 body_link 是 root，不在各腿 chain 的 segment 中）
    {
        const auto it = robot->tree.getSegment("body_link");
        if (it != robot->tree.getSegments().end()) {
            const auto body_seg = it->second.segment;
            const auto& rbi     = body_seg.getInertia();
            const double m      = rbi.getMass();
            if (m > 0.0) {
                mass_sum += m;
                com_sum = com_sum + rbi.getCOG() * m; // body_link 在 body_link 坐标系下
            }
        }
    }

    if (mass_sum <= 1e-9) {
        return std::make_tuple(Vector3D(0.0, 0.0, 0.0), 0.0);
    }

    const KDL::Vector com = com_sum / mass_sum;
    return std::make_tuple(Vector3D(com.x(), com.y(), com.z()), mass_sum);
}


    Cross_WallState::Cross_WallState(const std::string& urdf_file_path)
{

        robot = std::make_shared<Robot_t>(urdf_file_path);
    
        Vector3D com_3d;
        std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
        mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());
        
}

void Cross_WallState::enter() {
    robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
    robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
    robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
    robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
    // cross_wall_stage = -1;
    cross_wall_stage_time = std::chrono::steady_clock::now();
    stopping = false;
    stop_t = 0.0;
}


RobotTarget Cross_WallState::update() {

    Vector3D com_3d;
        std::tie(com_3d, mass) = get_robot_mass_info(robot->lf_joint_pos, robot->rf_joint_pos, robot->lb_joint_pos, robot->rb_joint_pos);
        mass_center_pos = Eigen::Vector2d(com_3d.x(), com_3d.y());

    RobotTarget joints_target;

    lf_wheel_force        = 0.0f;
    static int last_stage = -1;

    // 辅助函数:计算时间差(秒)
    auto get_elapsed_time = [this]() -> double {
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - cross_wall_stage_time);
        return duration.count() / 1000.0;
    };

    if (cross_wall_stage == -1) {
        last_stage  = -1;
        double time = get_elapsed_time();
        if (time > 0.5)
            cross_wall_stage = 0;
    }

        static int debug_print_cnt = 0;
    debug_print_cnt++;
    if (debug_print_cnt >= 250) {
        debug_print_cnt = 0;
        std::cerr << "\033[31mchange_flag = " << change_flag << ", cross_wall_stage = " << cross_wall_stage << "\033[0m" << std::endl;
    }

        auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
        auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
        auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
        lf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lf_grivate);
        rf_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rf_grivate);
        lb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_lb_grivate);
        rb_foot_exp_force=Vector3D(0.0,0.0,-robot->robot_rb_grivate);

        if(cross_wall_stage == 0  && change_flag == true){

            use_limit_lf = false;
            use_limit_lb = false;
            use_limit_rb = false;
            use_limit_rf = false;
                 
            wall_lf_foot_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
            wall_rf_foot_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
            wall_lb_foot_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
            wall_rb_foot_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);

            lf_step.update_support_trajectory(wall_lf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            rf_step.update_support_trajectory(wall_rf_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            lb_step.update_support_trajectory(wall_lb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);
            rb_step.update_support_trajectory(wall_rb_foot_pos, Vector3D(0.0,0.08,0.0), 1.0);

            //change_flag=false;
            cross_wall_stage = 1;       
        }
        else if (cross_wall_stage == 1 && change_flag == true){        
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();
                last_stage = cross_wall_stage;
            }

            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;

            bool success=false;
            double time=get_elapsed_time();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);   //在简化的二维平面模型中，提供支撑力的位置
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos=(lb_foot_exp_pos+robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            // 使用Eigen求解每条腿的支撑力
            // 构建方程组: A * f = b
            // 约束条件:
            // 1. 垂直力平衡: f_lf + f_rf + f_lb + f_rb = mg
            // 2. 绕质心的力矩平衡(x方向): (lf_pos.x - mass_center_pos.x)*f_lf + ... = 0
            // 3. 绕质心的力矩平衡(y方向): (lf_pos.y - mass_center_pos.y)*f_lf + ... = 0
            Eigen::Matrix<double, 3, 4> A;
            Eigen::Vector3d b;
            
            // 第一行: 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // lb
            A(0, 3) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 第二行: 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = lb_pos.x() - mass_center_pos.x();
            A(1, 3) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 第三行: 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = lb_pos.y() - mass_center_pos.y();
            A(2, 3) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 使用最小二乘法求解超定方程组
            Eigen::Vector4d forces = A.transpose() * (A * A.transpose()).inverse() * b;
            
            // 将求解的垂直力存入各腿的期望力向量（z分量）
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(3));
            
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(lf_cart_pos,lf_cart_pos,1.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,1.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,1.0);
                lb_step.update_flight_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0), Vector3D(0.12,0.08,0.0), Vector2D(0.0,0.0), 1.0,0.06);
                //change_flag=false;
                cross_wall_stage=2;     
            }
        }//1:其它保持不变,迈左后腿
        else if (cross_wall_stage == 2 && change_flag == true){         // 执行设置的腿长，调整质心位置，使其落在支撑三角形内

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            bool success=false;
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            double time=get_elapsed_time();
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            // 3足支撑力计算 (lf, rf, rb支撑，lb摆动)
            auto lf_pos=(lf_foot_exp_pos+robot->lf_leg_calc->pos_offset).head(2);
            auto rf_pos=(rf_foot_exp_pos+robot->rf_leg_calc->pos_offset).head(2);
            auto rb_pos=(rb_foot_exp_pos+robot->rb_leg_calc->pos_offset).head(2);
            
            Eigen::Matrix3d A;
            Eigen::Vector3d b;
            
            // 力平衡约束
            A(0, 0) = 1.0;  // lf
            A(0, 1) = 1.0;  // rf
            A(0, 2) = 1.0;  // rb
            b(0) = mass * 9.8;
            
            // 绕质心的x方向力矩平衡
            A(1, 0) = lf_pos.x() - mass_center_pos.x();
            A(1, 1) = rf_pos.x() - mass_center_pos.x();
            A(1, 2) = rb_pos.x() - mass_center_pos.x();
            b(1) = 0.0;
            
            // 绕质心的y方向力矩平衡
            A(2, 0) = lf_pos.y() - mass_center_pos.y();
            A(2, 1) = rf_pos.y() - mass_center_pos.y();
            A(2, 2) = rb_pos.y() - mass_center_pos.y();
            b(2) = 0.0;
            
            // 求解3x3方程组
            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);
            
            lf_foot_exp_force = Vector3D(0.0, 0.0, -forces(0));
            rf_foot_exp_force = Vector3D(0.0, 0.0, -forces(1));
            lb_foot_exp_force = Vector3D::Zero();  // 摆动腿无支撑力
            rb_foot_exp_force = Vector3D(0.0, 0.0, -forces(2));

            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(1.44,0.822,-0.165),2.0);
                
                //change_flag=false;
                cross_wall_stage=3;    
            }
        }
        else if (cross_wall_stage == 3 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success); 
            rf_foot_exp_pos=wall_rf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {
               
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(1.44,-0.92,0.48),2.0);
                
                //change_flag=false;
                cross_wall_stage=4;
            }
        }
        else if (cross_wall_stage == 4 && change_flag == true) {

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 摆动腿

            if(!success)
            {   

                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0 ,0.0,0.0),2.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.12,0.0,0.0),2.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0 ,0.0,0.0),2.0);

                //change_flag=false;
                cross_wall_stage=5;
            }
        }
        else if(cross_wall_stage==5 && change_flag == true){

            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            
            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿
            
            if(!success)
            {

                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D(-0.12,-1.00,0.96),1.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,1.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,1.0);
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,1.0);
                //change_flag=false;
                cross_wall_stage=6;
            }
        }
        //右前腿规划：6-9
        else if(cross_wall_stage == 6 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();  
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);

            // 三足支撑：rf, lb, rb
            auto rf_pos = (rf_foot_exp_pos + robot->rf_leg_calc->pos_offset).head(2);
            auto lb_pos = (lb_foot_exp_pos + robot->lb_leg_calc->pos_offset).head(2);
            auto rb_pos = (rb_foot_exp_pos + robot->rb_leg_calc->pos_offset).head(2);

            Eigen::Matrix3d A;
            Eigen::Vector3d b;

            A << 1, 1, 1,
                rf_pos.x() - mass_center_pos.x(), lb_pos.x() - mass_center_pos.x(), rb_pos.x() - mass_center_pos.x(),
                rf_pos.y() - mass_center_pos.y(), lb_pos.y() - mass_center_pos.y(), rb_pos.y() - mass_center_pos.y();
            b << mass*9.8, 0, 0;

            Eigen::Vector3d forces = A.colPivHouseholderQr().solve(b);

            rf_foot_exp_force = Vector3D(0,0,-forces(0));
            lb_foot_exp_force = Vector3D(0,0,-forces(1));
            rb_foot_exp_force = Vector3D(0,0,-forces(2));
            lf_foot_exp_force = Vector3D::Zero(); // 墙上腿

            if(!success)
            {
                int result;
                robot->rf_leg_calc->set_init_joint_pos(Vector3D(-0.13,-0.42,-0.536));
                rf_joint_exp_pos_ = robot->rf_leg_calc->joint_pos(rf_foot_exp_pos,&result);
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(robot->rf_joint_pos,Vector3D(-1.35,-0.827,-0.531),1.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,-0.03),1.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,-0.07),1.0);
                //change_flag=false;
                cross_wall_stage=7;
            }
        }
        else if(cross_wall_stage == 7 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=get_elapsed_time();

            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            
            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(robot->rf_joint_pos,Vector3D(-1.5,1.08,-0.531),2.0);

                //change_flag=false;
                cross_wall_stage=8;
            }
       }
        else if(cross_wall_stage == 8 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            double time=get_elapsed_time();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
            lb_foot_exp_pos=wall_lb_foot_pos;
            rb_foot_exp_pos=wall_rb_foot_pos;

            if(!success)
            {
                wall_lf_foot_pos=lf_joint_exp_pos_;
                wall_rf_foot_pos=rf_joint_exp_pos_;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                rf_step.update_support_trajectory(robot->rf_joint_pos,Vector3D(-0.60,0.60,-0.531),3.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(-0.08,0.0,-0.03),3.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(-0.08,0.0,-0.03),3.0);
                
                //change_flag=false;
                cross_wall_stage=9;
            }
        }
        else if(cross_wall_stage == 9 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success = false;
            double time=get_elapsed_time();
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            lf_joint_exp_pos_=wall_lf_foot_pos;
           
            if(!success)
            {
                wall_rf_foot_pos = rf_joint_exp_pos_;
                wall_lf_foot_pos = lf_joint_exp_pos_;
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;

                lf_step.update_support_trajectory(robot->lf_joint_pos,Vector3D( 0.0267, 2.1,-0.2),2.0); 
                rf_step.update_support_trajectory(robot->rf_joint_pos,Vector3D(-0.0267,-2.1, 0.2),2.0); 
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(-0.08,0.0,-0.08),2.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(-0.08,0.0,-0.08),2.0);

                //change_flag=false;               
                cross_wall_stage=10;     
            }
        }
        else if(cross_wall_stage == 10 && change_flag == true){
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = true;
            use_limit_rb = false;
            use_limit_rf = true;
            bool success=false;
            
            double time=get_elapsed_time();
            
            std::tie(rf_joint_exp_pos_,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_joint_exp_pos_,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);

            if(!success)
            {    
                wall_lb_foot_pos = lb_foot_exp_pos;
                wall_rb_foot_pos = rb_foot_exp_pos;
                wall_lf_foot_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                wall_rf_foot_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);

                lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                //change_flag=false;
                cross_wall_stage = 11;
                
            }
        }
        else if(cross_wall_stage == 11 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                //change_flag=false;
                cross_wall_stage=12;
                RL_walk_flag = true;
            }
        }
        else if(cross_wall_stage == 12 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                last_stage = cross_wall_stage;
            }
            // if(RL_walk_flag == false)
            // {
            //     robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
            //     robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
            //     robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
            //     robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
            //     cross_wall_stage = 13;
            // }
        }
        else if (cross_wall_stage == 13 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                //change_flag=false;
                cross_wall_stage=14;
            }
        }
        else if (cross_wall_stage == 14 && change_flag == true)
        { 
            if (cross_wall_stage != last_stage)
            {
                lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.0,0.0,0.0),2.0); 
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(lf_cart_pos,Vector3D(0.15,0.0,0.15),2.0); 
                rf_step.update_support_trajectory(rf_cart_pos,Vector3D(0.15,0.0,0.15),2.0); 
                rb_step.update_support_trajectory(rb_cart_pos,rb_cart_pos,2.0);
                lb_step.update_support_trajectory(lb_cart_pos,lb_cart_pos,2.0);

                //change_flag=false;
                cross_wall_stage=15;
            }
        }
        else if(cross_wall_stage == 15 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();
            
            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);

            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                lf_step.update_support_trajectory(lf_cart_pos,lf_cart_pos,1.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,1.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,-0.08,0.4),1.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0, 0.08,0.4),1.0);

                //change_flag=false;
                cross_wall_stage=16;
            }
        }
        else if(cross_wall_stage == 16 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;
                
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.25,-0.08,0.4),1.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.25, 0.08,0.4),1.0);

                //change_flag=false;
                cross_wall_stage=17;
            }
        }
        else if(cross_wall_stage == 17 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

               
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,-0.08),4.0);
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,-0.08),4.0);
                

                //change_flag=false;
                cross_wall_stage=18;
            }
        }
        else if(cross_wall_stage == 18 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success=false;
            double time=get_elapsed_time();
            
           
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
           
            if(!success)
            {   
                wall_lf_foot_pos=lf_foot_exp_pos;
                wall_rf_foot_pos=rf_foot_exp_pos;
                wall_lb_foot_pos=lb_foot_exp_pos;
                wall_rb_foot_pos=rb_foot_exp_pos;

                lf_step.update_support_trajectory(lf_cart_pos,lf_cart_pos,2.0);
                rf_step.update_support_trajectory(rf_cart_pos,rf_cart_pos,2.0); 
                rb_step.update_support_trajectory(rb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
                lb_step.update_support_trajectory(lb_cart_pos,Vector3D(0.0,0.0,0.0),2.0);
              
                //change_flag=false;
                cross_wall_stage=19;
            }
        }
        else if(cross_wall_stage==19 && change_flag == true)
        {
            if (cross_wall_stage != last_stage)
            {
                cross_wall_stage_time = std::chrono::steady_clock::now();;
                last_stage = cross_wall_stage;
            }
            use_limit_lb = false;
            use_limit_lf = false;
            use_limit_rb = false;
            use_limit_rf = false;
            bool success = false;

            double time=get_elapsed_time();

            std::tie(rf_foot_exp_pos,rf_foot_exp_vel,rf_foot_exp_acc)=rf_step.get_target(time, success);
            std::tie(lf_foot_exp_pos,lf_foot_exp_vel,lf_foot_exp_acc)=lf_step.get_target(time, success);
            std::tie(rb_foot_exp_pos,rb_foot_exp_vel,rb_foot_exp_acc)=rb_step.get_target(time, success);
            std::tie(lb_foot_exp_pos,lb_foot_exp_vel,lb_foot_exp_acc)=lb_step.get_target(time, success);
            
            if(!success)
            {
                change_flag=false;
                // 打印四条腿的关节角度
                // RCLCPP_INFO(robot->node_->get_logger(),
                //     "\033[36m=== Joint Angles ===\n"
                //     "LF: (%.3f, %.3f, %.3f)\n"
                //     "RF: (%.3f, %.3f, %.3f)\n"
                //     "LB: (%.3f, %.3f, %.3f)\n"
                //     "RB: (%.3f, %.3f, %.3f)\033[0m",
                //     robot->lf_joint_pos[0], robot->lf_joint_pos[1], robot->lf_joint_pos[2],
                //     robot->rf_joint_pos[0], robot->rf_joint_pos[1], robot->rf_joint_pos[2],
                //     robot->lb_joint_pos[0], robot->lb_joint_pos[1], robot->lb_joint_pos[2],
                //     robot->rb_joint_pos[0], robot->rb_joint_pos[1], robot->rb_joint_pos[2]);
                
                auto lf_cart_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
                auto rf_cart_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
                auto lb_cart_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
                auto rb_cart_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
                robot->lf_z_vmc->reset(lf_cart_pos.z(), 0.0);
                robot->rf_z_vmc->reset(rf_cart_pos.z(), 0.0);
                robot->lb_z_vmc->reset(lb_cart_pos.z(), 0.0);
                robot->rb_z_vmc->reset(rb_cart_pos.z(), 0.0);
                Cross_wall_over = true;
                cross_wall_stage = 20;
            }
        }



    /*******************************lf**********************************/
    if (use_limit_lf) {

        for (int i = 0; i < 3; i++) {
            joints_target.legs[1].joints[i].kp  = robot->lf_leg_calc->kp[i];
            joints_target.legs[1].joints[i].kd  = robot->lf_leg_calc->kd[i];
            joints_target.legs[1].joints[i].rad = lf_joint_exp_pos_[i];
        }
        robot->lf_leg_calc->joint_pos_setarray(robot->lf_joint_pos);
        lf_foot_exp_pos = robot->lf_leg_calc->foot_pos(robot->lf_joint_pos);
        // static int cnt_ = 0;
        // cnt_++;
        // if(cnt_>=1)
        // {
        //     cnt_ = 0;
        //     std::cerr << "\033[31mlf_joint_pos = (" << robot->lf_joint_pos[0] << ", " 
        //              << robot->lf_joint_pos[1] << " " << robot->lf_joint_pos[2] 
        //              << "), lf_joint_exp_pos_ = (" << lf_joint_exp_pos_[0] << ", " 
        //              << lf_joint_exp_pos_[1] << ", " << lf_joint_exp_pos_[2] << ")\033[0m" << std::endl;
        // }
    } else {
        joints_target.legs[1] = robot->lf_leg_calc->signal_leg_calc(
            lf_foot_exp_pos, lf_foot_exp_vel, lf_foot_exp_acc, lf_foot_exp_force, &robot->lf_forward_torque, lf_wheel_vel, lf_wheel_force);
    }

    /*******************************rf**********************************/
    if (use_limit_rf) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[0].joints[i].rad = rf_joint_exp_pos_[i];
            joints_target.legs[0].joints[i].kp  = robot->rf_leg_calc->kp[i];
            joints_target.legs[0].joints[i].kd  = robot->rf_leg_calc->kd[i];
        }
        robot->rf_leg_calc->joint_pos_setarray(robot->rf_joint_pos);
        rf_foot_exp_pos = robot->rf_leg_calc->foot_pos(robot->rf_joint_pos);
    } else {
        joints_target.legs[0] = robot->rf_leg_calc->signal_leg_calc(
            rf_foot_exp_pos, rf_foot_exp_vel, rf_foot_exp_acc, rf_foot_exp_force, &robot->rf_forward_torque, rf_wheel_vel, rf_wheel_force);
    }
    
    /*******************************lb**********************************/
    if (use_limit_lb) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[3].joints[i].kp  = robot->lb_leg_calc->kp[i];
            joints_target.legs[3].joints[i].kd  = robot->lb_leg_calc->kd[i];
            joints_target.legs[3].joints[i].rad = lb_joint_exp_pos_[i];
        }
        robot->lb_leg_calc->joint_pos_setarray(robot->lb_joint_pos);
        lb_foot_exp_pos = robot->lb_leg_calc->foot_pos(robot->lb_joint_pos);
    } else {
        joints_target.legs[3] = robot->lb_leg_calc->signal_leg_calc(
            lb_foot_exp_pos, lb_foot_exp_vel, lb_foot_exp_acc, lb_foot_exp_force, &robot->lb_forward_torque, lb_wheel_vel, lb_wheel_force);
    }
    
    /*******************************rb**********************************/
    if (use_limit_rb) {
        for (int i = 0; i < 3; i++) {
            joints_target.legs[2].joints[i].kp  = robot->rb_leg_calc->kp[i];
            joints_target.legs[2].joints[i].kd  = robot->rb_leg_calc->kd[i];
            joints_target.legs[2].joints[i].rad = rb_joint_exp_pos_[i];
        }
        robot->rb_leg_calc->joint_pos_setarray(robot->rb_joint_pos);
        rb_foot_exp_pos = robot->rb_leg_calc->foot_pos(robot->rb_joint_pos);
    } else {
        joints_target.legs[2] = robot->rb_leg_calc->signal_leg_calc(
            rb_foot_exp_pos, rb_foot_exp_vel, rb_foot_exp_acc, rb_foot_exp_force, &robot->rb_forward_torque, rb_wheel_vel, rb_wheel_force);
    }

    // TODO: 替换为纯 C++ 的控制指令输出方式
    // robot->legs_target_pub->publish(joints_target);
    // 目前直接返回 joints_target,由调用者处理

    return joints_target;
}
