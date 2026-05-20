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

bool Record::record_pos(const Eigen::Vector3d &target_info)
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
    yaml_out_ << "paths:\n";
}

void Record::write_path_point(const Eigen::Vector3d &target_info)
{
    yaml_out_ << "  - update_policy: false \n";
    yaml_out_ << "    params:" "\n";
    yaml_out_ << "    target_pos:\n";
    yaml_out_ << "      x: " <<  target_info[0]  << "\n";
    yaml_out_ << "      y: " << target_info[1] << "\n";
}
