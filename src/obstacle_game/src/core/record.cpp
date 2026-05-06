#include <core/record.hpp>

#include <filesystem>
#include <utility>

Record::Record(rclcpp::Node::SharedPtr node)
    : node_(std::move(node))
{
}

bool Record::set_output_yaml(const std::string file_path)
{
    close_current_file();
    recorded_count_ = 0;
    output_file_path_ = file_path;

    if (output_file_path_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "输出yaml路径为空");
        return false;
    }

    try {
        const std::filesystem::path path(output_file_path_);
        if (path.has_parent_path()) {
            std::filesystem::create_directories(path.parent_path());
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "创建yaml目录失败: %s", e.what());
        output_file_path_.clear();
        return false;
    }

    yaml_out_.open(output_file_path_, std::ios::out | std::ios::trunc);
    if (!yaml_out_.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "无法打开yaml文件: %s", output_file_path_.c_str());
        output_file_path_.clear();
        return false;
    }

    write_yaml_header();
    yaml_out_.flush();
    return true;
}

bool Record::record_pos(const PathPoint &target_info)
{
    if (!yaml_out_.is_open()) {
        RCLCPP_WARN(node_->get_logger(), "尚未设置输出yaml文件，record_pos被忽略");
        return false;
    }

    write_path_point(target_info);
    yaml_out_.flush();

    if (!yaml_out_.good()) {
        RCLCPP_ERROR(node_->get_logger(), "写入yaml文件失败: %s", output_file_path_.c_str());
        return false;
    }

    ++recorded_count_;
    return true;
}

int Record::finishe_record()
{
    const int count = recorded_count_;
    close_current_file();
    recorded_count_ = 0;
    output_file_path_.clear();
    return count;
}

void Record::close_current_file()
{
    if (yaml_out_.is_open()) {
        yaml_out_.flush();
        yaml_out_.close();
    }
}

void Record::write_yaml_header()
{
    yaml_out_ << "#policy_id表示机器人执行这段路径时使用的策略，0表示上电策略，不用；1表示位控站立，一般也不用；\n";
    yaml_out_ << "#2表示walk策略，3表示sand策略，4表示stair策略\n";
    yaml_out_ << "\n";
    yaml_out_ << "paths:\n";
}

void Record::write_path_point(const PathPoint &target_info)
{
    yaml_out_ << "  - policy_id: " << target_info.policy_id << "\n";
    yaml_out_ << "    target_pos:\n";
    yaml_out_ << "      x: " << target_info.target_pos.x() << "\n";
    yaml_out_ << "      y: " << target_info.target_pos.y() << "\n";
    yaml_out_ << "    target_vel: " << target_info.target_vel << "\n";
    yaml_out_ << "    max_velocity: " << target_info.max_velocity << "\n";
    yaml_out_ << "    max_accelation: " << target_info.max_accelation << "\n";
    yaml_out_ << "    kp:\n";
    yaml_out_ << "      x: " << target_info.kp.x() << "\n";
    yaml_out_ << "      y: " << target_info.kp.y() << "\n";
    yaml_out_ << "      yaw: " << target_info.kp.z() << "\n";
    yaml_out_ << "    allow_start_dir_error: " << target_info.allow_start_dir_error << "\n";
    yaml_out_ << "    err_allow: " << target_info.err_allow << "\n";
    yaml_out_ << "    adjust_min_vel: " << target_info.adjust_min_vel << "\n";
}
