#include <chrono>
#include <core/robot.hpp>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/time.hpp>

using namespace std::chrono_literals;


Robot::Robot(const std::shared_ptr<rclcpp::Node> node)
    : node_(node) {
    node_->declare_parameter<std::string>("scene_path","");
    node_->declare_parameter<std::string>("yaml_file_path","");

    auto yaml_path = node_->get_parameter("yaml_file_path").as_string();
    if (yaml_path.empty()) {
        yaml_path = node_->get_parameter("scene_path").as_string();
    }
    if (yaml_path.empty()) {
        autopilot_available = false;
        RCLCPP_WARN(node_->get_logger(), "无导航点yaml文件路径，自动驾驶不可用");
    }

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


    pilot = std::make_shared<Pilot>(node_, yaml_path);
    record=std::make_shared<Record>(node_);

    // 机器人运动控制指令发布
    cmd_pub_ = node_->create_publisher<robot_msgs::msg::Cmd>("robot_move_cmd", 10);

    // 机器人遥控器指令订阅
    remote_sub_ = node_->create_subscription<robot_msgs::msg::Remote>("remote", 10, [this](const robot_msgs::msg::Remote& msg) {
        // TODO:处理并发布遥控器数据
        if (check_key_trigger(msg.key,1)) // 手动控制
        {
            if (current_control_mode == 1) {
                cmd.mode = 1;    // 如果刚才是自动控制，那么切入手动控制时进入位控站立模式(可能是有紧急情况)
            }
            pilot->reset();
            pilot->stop();
            current_control_mode = 0;
            RCLCPP_INFO(node_->get_logger(), "请求切入手动控制");
        }
        if (check_key_trigger(msg.key,2)) {
            current_control_mode = 1;
            RCLCPP_INFO(node_->get_logger(), "请求切入自动控制");
        }

        // 只有手动控制模式下可以使用遥控器修改机器人当前使用的策略
        if (current_control_mode == 0) {
            if (check_key_trigger(msg.key,4)) {         //模式控制
                cmd.mode = 1; // 位控站立
            } else if (check_key_trigger(msg.key,5)) {
                cmd.mode = 2; // 普通行走策略
            } else if (check_key_trigger(msg.key,6)) {
                cmd.mode = 3; // 过沙地策略
            } else if (check_key_trigger(msg.key,3)) {
                cmd.mode = 4; // 上台阶策略
            }

            //摇杆赋值
            cmd.vx=(float)std::clamp((double)msg.lx,-1.2,1.2);
            cmd.vy=(float)std::clamp((double)msg.ly,-1.2,1.2);
            cmd.vz=(float)std::clamp((double)msg.rx,-1.0,1.0);


        } else if (current_control_mode == 1) {
            if (check_key_trigger(msg.key,4)) {
                pilot->reset();
                pilot->stop();
            } else if (check_key_trigger(msg.key,5)) {
                pilot->start();        // 开始执行自动控制
            } else if (check_key_trigger(msg.key,6)) {
                pilot->stop();         //  自动控制执行暂停
            }
        }


        // if(check_key_trigger(msg.key,5))    //开启文件记录
        // {
        //     const auto now = std::chrono::system_clock::now();
        //     const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        //     std::tm local_tm{};
        //     localtime_r(&now_time, &local_tm);

        //     std::ostringstream oss;
        //     oss << node_->get_parameter("yaml_file_path").as_string()
        //         << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".yaml";
        //     record->set_output_yaml(oss.str());
        // }
        // else if(check_key_trigger(msg.key,6))   //关闭文件记录
        // {
        //     record->finishe_record();
        // }
        // else if(check_key_trigger(msg.key,7))   //记录点位
        // {
        //     Record::PathPoint point;
        //     point.target_pos[0]=
        //     record->record_pos(point);
        // }

        record_key(msg.key);
    });

    param_server_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        RCLCPP_INFO(node_->get_logger(), "更新参数");
        for (const auto& param : params) {
            // TODO:处理参数更新
        }
        return result;
    });

    control_timer = node_->create_wall_timer(50ms, [this]() {
        if (current_control_mode == 1) {
            geometry_msgs::msg::TransformStamped transfer;
            try {
                transfer = tf_buffer_->lookupTransform("robot", "world", tf2::TimePointZero, tf2::durationFromSec(0.05));
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(node_->get_logger(), "获取目标 TF 失败，自动驾驶仪停止运行: %s", ex.what());
                current_control_mode = 0;
                return;
            }

            tf2::Quaternion q;
            q.setW(transfer.transform.rotation.w);
            q.setX(transfer.transform.rotation.x);
            q.setY(transfer.transform.rotation.y);
            q.setZ(transfer.transform.rotation.z);
            double cur_roll, cur_pitch, cur_yaw;
            tf2::Matrix3x3(q).getRPY(cur_roll, cur_pitch, cur_yaw);

            pilot->set_state(Eigen::Vector2d(transfer.transform.translation.x, transfer.transform.translation.y), cur_yaw);

            cmd = pilot->get_command(std::chrono::high_resolution_clock::now());
        }
        cmd_pub_->publish(cmd);
    });
}

bool Robot::check_key_trigger(uint32_t current_key,int index)
{
    bool current_is_true=((current_key>>index)&0x0001);
    bool last_is_false=!((last_key>>index)&0x0001);
    return current_is_true&&last_is_false;
}

void Robot::record_key(uint32_t current_key)
{
    last_key=current_key;
}
