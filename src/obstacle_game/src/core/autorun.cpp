#include "core/autorun.hpp"

#include <utility>

#include "executer/pilot.hpp"

namespace {

double read_required_scalar(const YAML::Node &node, const char *key)
{
    return node[key].as<double>();
}

}  // namespace

AutoRun::AutoRun(rclcpp::Node::SharedPtr node, const std::string yaml_path)
    : node_(std::move(node))
{
    auto default_pilot = std::make_shared<Pilot>(node_);
    executer_map_.insert_or_assign(default_pilot->get_executer_name(), default_pilot);

    path_loaded_ = load_path(yaml_path);
}

void AutoRun::register_executer(std::shared_ptr<BaseExecuter> executer)
{
    if (!executer) {
        RCLCPP_WARN(node_->get_logger(), "注册执行器失败: executer为空");
        return;
    }

    if (has_robot_state_) {
        executer->set_state(current_pos_, static_cast<float>(current_yaw_));
    }

    executer_map_.insert_or_assign(executer->get_executer_name(), std::move(executer));
}

void AutoRun::set_robot_state(const Eigen::Vector2d &pos, const double &yaw)
{
    current_pos_ = pos;
    current_yaw_ = yaw;
    has_robot_state_ = true;

    for (auto &[name, executer] : executer_map_) {
        (void)name;
        if (executer) {
            executer->set_state(pos, static_cast<float>(yaw));
        }
    }
}

robot_msgs::msg::Cmd AutoRun::get_command(std::chrono::time_point<std::chrono::high_resolution_clock> time)
{
    if (!is_running_ || !active_executer_) {
        return make_zero_command();
    }

    return active_executer_->get_command(time);
}

bool AutoRun::start()
{
    if (!path_loaded_) {
        RCLCPP_ERROR(node_->get_logger(), "自动驾驶启动失败: 路径文件未成功加载");
        return false;
    }

    if (path_points_.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "自动驾驶启动失败: 路径点为空");
        return false;
    }

    if (current_index_ >= path_points_.size()) {
        RCLCPP_INFO(node_->get_logger(), "路径已经执行完毕，自动从起点重新开始");
        current_index_ = 0;
        active_executer_.reset();
    }

    if (is_running_) {
        RCLCPP_INFO(node_->get_logger(), "自动驾驶已经处于运行状态");
        return true;
    }

    is_running_ = true;
    return start_current_target();
}

bool AutoRun::stop()
{
    is_running_ = false;

    if (active_executer_) {
        return active_executer_->stop();
    }

    return true;
}

bool AutoRun::reset()
{
    const bool stop_ok = stop();
    current_index_ = 0;
    active_executer_.reset();
    return stop_ok;
}

bool AutoRun::load_path(const std::string &yaml_path)
{
    try {
        const YAML::Node root = YAML::LoadFile(yaml_path);
        const YAML::Node paths = root["paths"];
        if (!paths || !paths.IsSequence()) {
            RCLCPP_ERROR(node_->get_logger(), "路径文件格式错误: 缺少paths数组: %s", yaml_path.c_str());
            return false;
        }

        path_points_.clear();
        path_points_.reserve(paths.size());
        for (std::size_t index = 0; index < paths.size(); ++index) {
            if (!load_path_point(paths[index], index)) {
                path_points_.clear();
                return false;
            }
        }

        RCLCPP_INFO(node_->get_logger(), "成功加载自动驾驶路径，共%zu个点: %s", path_points_.size(), yaml_path.c_str());
        return true;
    } catch (const YAML::Exception &ex) {
        RCLCPP_ERROR(node_->get_logger(), "解析路径文件失败 %s: %s", yaml_path.c_str(), ex.what());
        return false;
    }
}

bool AutoRun::load_path_point(const YAML::Node &path_node, std::size_t index)
{
    if (!path_node.IsMap()) {
        RCLCPP_ERROR(node_->get_logger(), "路径点%zu格式错误: 必须是对象", index);
        return false;
    }

    const YAML::Node executer_node = path_node["executer"];
    const YAML::Node params_node = path_node["params"];
    const YAML::Node target_pos_node = path_node["target_pos"];
    if (!executer_node || !target_pos_node || !target_pos_node.IsMap()) {
        RCLCPP_ERROR(node_->get_logger(), "路径点%zu缺少必要字段executer/target_pos", index);
        return false;
    }

    if (!target_pos_node["x"] || !target_pos_node["y"] || !target_pos_node["yaw"]) {
        RCLCPP_ERROR(node_->get_logger(), "路径点%zu缺少target_pos.x/y/yaw", index);
        return false;
    }

    PathPoint point;
    point.executer_name = executer_node.as<std::string>();
    point.params = params_node ? params_node.as<std::string>() : "";
    point.target_pose.x() = read_required_scalar(target_pos_node, "x");
    point.target_pose.y() = read_required_scalar(target_pos_node, "y");
    point.target_pose.z() = read_required_scalar(target_pos_node, "yaw");

    path_points_.push_back(std::move(point));
    return true;
}

bool AutoRun::start_current_target()
{
    if (!is_running_) {
        return true;
    }

    if (current_index_ >= path_points_.size()) {
        is_running_ = false;
        active_executer_.reset();
        RCLCPP_INFO(node_->get_logger(), "自动驾驶路径执行完成");
        return true;
    }

    if (!prepare_current_target()) {
        is_running_ = false;
        return false;
    }

    const std::size_t target_index = current_index_;
    return active_executer_->start([this, target_index](int success) {
        if (target_index != current_index_) {
            return;
        }
        advance_to_next_target(success);
    });
}

bool AutoRun::prepare_current_target()
{
    if (current_index_ >= path_points_.size()) {
        RCLCPP_ERROR(node_->get_logger(), "准备路径点失败: 索引越界");
        return false;
    }

    const PathPoint &point = path_points_[current_index_];
    auto executer = find_executer(point.executer_name);
    if (!executer) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "路径点%zu请求的执行器[%s]尚未注册",
            current_index_,
            point.executer_name.c_str());
        return false;
    }

    if (active_executer_ && active_executer_ != executer) {
        active_executer_->stop();
    }

    active_executer_ = executer;
    if (has_robot_state_) {
        active_executer_->set_state(current_pos_, static_cast<float>(current_yaw_));
    }

    std::string params = point.params;
    if (!active_executer_->set_target(point.target_pose, params)) {
        RCLCPP_ERROR(node_->get_logger(), "路径点%zu设置目标失败", current_index_);
        return false;
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "开始执行路径点%zu/%zu, executer=%s, target=(%.3f, %.3f, %.3f), scene_id=%d",
        current_index_ + 1,
        path_points_.size(),
        point.executer_name.c_str(),
        point.target_pose.x(),
        point.target_pose.y(),
        point.target_pose.z(),
        current_scene_id_);

    return true;
}

void AutoRun::advance_to_next_target(int success)
{
    if (success <= 0) {
        RCLCPP_WARN(node_->get_logger(), "路径点%zu执行失败，自动驾驶停止", current_index_ + 1);
        is_running_ = false;
        if (active_executer_) {
            active_executer_->stop();
        }
        return;
    }

    ++current_index_;
    if (current_index_ >= path_points_.size()) {
        is_running_ = false;
        active_executer_.reset();
        RCLCPP_INFO(node_->get_logger(), "全部路径点执行完成");
        return;
    }

    if (!is_running_) {
        return;
    }

    if (!start_current_target()) {
        RCLCPP_ERROR(node_->get_logger(), "切换到下一个路径点失败");
    }
}

robot_msgs::msg::Cmd AutoRun::make_zero_command() const
{
    return robot_msgs::msg::Cmd();
}

std::shared_ptr<BaseExecuter> AutoRun::find_executer(const std::string &executer_name) const
{
    const auto iter = executer_map_.find(executer_name);
    if (iter == executer_map_.end()) {
        return nullptr;
    }
    return iter->second;
}
