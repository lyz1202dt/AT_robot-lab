#include "nodes/generate_plan.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr int kArmboxid = 7;

const auto box_id_kSemaphoreTimeout = std::chrono::hours(24 * 365 * 100);
constexpr const char* kGeneratePlanConfigParam = "generate_plan_config";

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
        std::this_thread::sleep_for(50ms);
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

bool wait_for_start(Robot* context) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && (!context->start_game)) {
        std::this_thread::sleep_for(50ms);
    }

    return rclcpp::ok() && context->auto_pilot_enabled.load();
}

bool wait_with_interrupt(Robot* context, const std::chrono::milliseconds duration) {
    auto remaining = duration;
    while (remaining.count() > 0) {
        if (!rclcpp::ok() || !context->auto_pilot_enabled.load()) {
            return false;
        }
        const auto step = std::min(remaining, 50ms);
        std::this_thread::sleep_for(step);
        remaining -= step;
    }

    return true;
}

bool wait_semaphore_with_timeout(sem_t* sem, const std::chrono::seconds timeout) {
    timespec deadline{};
    if (clock_gettime(CLOCK_REALTIME, &deadline) != 0) {
        return false;
    }

    deadline.tv_sec += timeout.count();

    while (sem_timedwait(sem, &deadline) != 0) {
        if (errno == EINTR) {
            continue;
        }
        return false;
    }

    return true;
}

void drain_semaphore(sem_t* sem) {
    while (sem_trywait(sem) == 0) {
    }

    if (errno != EAGAIN) {
        errno = 0;
    }
}

// 三行四列的位置表：第 0 行为放置区，第 1/2 行为两条抓取线。
using PositionGrid = std::array<std::array<std::tuple<float, float, float>, 4>, 3>;
using BoxPositionGrid = std::array<std::array<std::array<float, 2>, 4>, 3>;

struct RoutePoints {
    std::array<float, 3> a1{};
    std::array<float, 3> a2{};
};

struct PlanConfig {
    PositionGrid positions{};
    BoxPositionGrid arm_box_positions{};
    RoutePoints route{};
    TargetPoint start_to_a1{};
    TargetPoint a1_to_box0{};
    TargetPoint box0_to_box1{};
    TargetPoint box1_to_a2{};
    TargetPoint a2_to_dst1{};
    TargetPoint box1_to_dst1{};
    TargetPoint dst1_to_dst0{};
    TargetPoint dst0_to_dst2{};
    TargetPoint dst2_to_box0{};
};

struct BoxInfo {
    BoxIdGrid box_ids{};
    PositionGrid positions{};
};

struct SelectedBox {
    int line{-1};
    int col{-1};
};

std::tuple<float, float, float> read_point3_tuple(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(name + " 必须是长度为 3 的数组");
    }
    return {node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

std::array<float, 3> read_point3(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 3) {
        throw std::runtime_error(name + " 必须是长度为 3 的数组");
    }
    return {node[0].as<float>(), node[1].as<float>(), node[2].as<float>()};
}

std::array<std::tuple<float, float, float>, 4> read_point3_row(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 4) {
        throw std::runtime_error(name + " 必须包含 4 个点");
    }

    std::array<std::tuple<float, float, float>, 4> row{};
    for (size_t i = 0; i < row.size(); ++i) {
        row[i] = read_point3_tuple(node[i], name + "[" + std::to_string(i) + "]");
    }
    return row;
}

std::array<float, 2> read_point2(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 2) {
        throw std::runtime_error(name + " 必须是长度为 2 的数组");
    }
    return {node[0].as<float>(), node[1].as<float>()};
}

std::array<std::array<float, 2>, 4> read_point2_row(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsSequence() || node.size() != 4) {
        throw std::runtime_error(name + " 必须包含 4 个点");
    }

    std::array<std::array<float, 2>, 4> row{};
    for (size_t i = 0; i < row.size(); ++i) {
        row[i] = read_point2(node[i], name + "[" + std::to_string(i) + "]");
    }
    return row;
}

TargetPoint read_target_point(const YAML::Node& node, const std::string& name) {
    if (!node || !node.IsMap()) {
        throw std::runtime_error(name + " 必须是对象");
    }

    const auto kp_node = node["kp"];
    if (!kp_node || !kp_node.IsSequence() || kp_node.size() != 3) {
        throw std::runtime_error(name + ".kp 必须是长度为 3 的数组");
    }

    TargetPoint target_point;
    target_point.constraint_target_yaw = node["constraint_target_yaw"].as<bool>();
    target_point.target_vel = node["target_vel"].as<float>();
    target_point.max_velocity = node["max_velocity"].as<float>();
    target_point.max_accelation = node["max_accelation"].as<float>();
    target_point.max_omega = node["max_omega"].as<float>();
    target_point.kp = Eigen::Vector3d(kp_node[0].as<double>(), kp_node[1].as<double>(), kp_node[2].as<double>());
    target_point.allow_start_dir_error = node["allow_start_dir_error"].as<float>();
    target_point.allow_final_dir_error = node["allow_final_dir_error"].as<float>();
    target_point.allow_final_pos_allow = node["allow_final_pos_allow"].as<float>();
    target_point.adjust_min_vel = node["adjust_min_vel"].as<float>();
    target_point.adjust_min_omega = node["adjust_min_omega"].as<float>();
    target_point.allow_y_vel = node["allow_y_vel"].as<bool>();
    if (const auto connection_radius = node["trajectory_connection_radius"]) {
        target_point.trajectory_connection_radius = connection_radius.as<float>();
    }
    return target_point;
}

PlanConfig load_plan_config(const std::string& yaml_path) {
    const YAML::Node root = YAML::LoadFile(yaml_path);

    const auto arm_box_positions = root["arm_box_positions"];
    const auto box_positions = root["box_positions"];
    const auto routes = root["routes"];
    const auto target_points = root["target_points"];
    if (!arm_box_positions || !box_positions || !routes || !target_points) {
        throw std::runtime_error("generate_plan.yaml 缺少 arm_box_positions/box_positions/routes/target_points");
    }

    PlanConfig config;
    config.arm_box_positions[0] = read_point2_row(arm_box_positions["arm_place"], "arm_box_positions.arm_place");
    config.arm_box_positions[1] = read_point2_row(arm_box_positions["arm_pick_line_0"], "arm_box_positions.arm_pick_line_0");
    config.arm_box_positions[2] = read_point2_row(arm_box_positions["arm_pick_line_1"], "arm_box_positions.arm_pick_line_1");
    config.positions[0] = read_point3_row(box_positions["place"], "box_positions.place");
    config.positions[1] = read_point3_row(box_positions["pick_line_0"], "box_positions.pick_line_0");
    config.positions[2] = read_point3_row(box_positions["pick_line_1"], "box_positions.pick_line_1");

    config.route.a1 = read_point3(routes["a1"], "routes.a1");
    config.route.a2 = read_point3(routes["a2"], "routes.a2");

    config.start_to_a1 = read_target_point(target_points["start_to_a1"], "target_points.start_to_a1");
    config.a1_to_box0 = read_target_point(target_points["a1_to_box0"], "target_points.a1_to_box0");
    config.box0_to_box1 = read_target_point(target_points["box0_to_box1"], "target_points.box0_to_box1");
    config.box1_to_a2 = read_target_point(target_points["box1_to_a2"], "target_points.box1_to_a2");
    config.a2_to_dst1 = read_target_point(target_points["a2_to_dst1"], "target_points.a2_to_dst1");
    config.box1_to_dst1 = read_target_point(target_points["box1_to_dst1"], "target_points.box1_to_dst1");
    config.dst1_to_dst0 = read_target_point(target_points["dst1_to_dst0"], "target_points.dst1_to_dst0");
    config.dst0_to_dst2 = read_target_point(target_points["dst0_to_dst2"], "target_points.dst0_to_dst2");
    config.dst2_to_box0 = read_target_point(target_points["dst2_to_box0"], "target_points.dst2_to_box0");
    return config;
}

std::array<float, 3> pose_from_grid(const PositionGrid& positions, int line, int col) {
    return {
        std::get<0>(positions[line][col]),
        std::get<1>(positions[line][col]),
        std::get<2>(positions[line][col])};
}

int choose_first_col(const PlanConfig& plan_config, const std::array<float, 3>& a1) {
    int first_col = 0;
    float min_y_error = std::numeric_limits<float>::max();
    for (int col = 0; col < 4; ++col) {
        const auto near_line_pose = pose_from_grid(plan_config.positions, 2, col);
        const float y_error = std::abs(a1[1] - near_line_pose[1]);
        if (y_error < min_y_error) {
            first_col = col;
            min_y_error = y_error;
        }
    }
    return first_col;
}

std::vector<SelectedBox> make_fixed_pick_order(int first_col) {
    std::vector<SelectedBox> pick_order;
    pick_order.reserve(8);

    // 第一轮先清出一列通道：先抓近排放平板，再抓远排留在机械臂上。
    pick_order.push_back({1, first_col});
    pick_order.push_back({0, first_col});

    for (int offset = 1; offset < 4; ++offset) {
        const int lower_col = first_col + offset;
        if (lower_col < 4) {
            pick_order.push_back({0, lower_col});
            pick_order.push_back({1, lower_col});
        }

        const int upper_col = first_col - offset;
        if (upper_col >= 0) {
            pick_order.push_back({0, upper_col});
            pick_order.push_back({1, upper_col});
        }
    }

    return pick_order;
}

void fill_box_task(BoxMoveTask& task,
                   const PlanConfig& plan_config,
                   const BoxInfo& box_info,
                   const SelectedBox& selected) {
    const int position_line = selected.line + 1;
    const int box_id = box_info.box_ids[selected.line][selected.col];
    task.pick_box_pos = plan_config.arm_box_positions[position_line][selected.col];
    task.place_box_pos = plan_config.arm_box_positions[0][box_id];
    task.box_id = box_id;
    task.line = selected.line;
    task.col = selected.col;
}

}  // namespace

GeneratePlaneAction::GeneratePlaneAction()
    : BT::ActionNode("generate_plan_action") {
    sem_init(&box_id_grid_sem_, 0, 0);
}

GeneratePlaneAction::~GeneratePlaneAction() {
    sem_destroy(&box_id_grid_sem_);
}

void GeneratePlaneAction::reset_generated() {
    generated = false;
    first_run_ = true;
    {
        std::lock_guard<std::mutex> lock(box_id_grid_mutex_);
        lock_box_id_grid_ = false;
    }
    drain_semaphore(&box_id_grid_sem_);
}

// 只订阅箱子矩阵，规划输入统一来自 box_id_grid。
void GeneratePlaneAction::init_subscriptions(const rclcpp::Node::SharedPtr& node) {
    if (subscriptions_ready_) {
        return;
    }

    node_ = node;
    node_->declare_parameter<std::string>(kGeneratePlanConfigParam, "");
    box_grid_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "box_id_grid", 10,
        [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
            if (msg->data.size() < 8) {
                RCLCPP_ERROR(node_->get_logger(), "box_id_grid 至少需要 8 个元素，当前只有 %zu 个", msg->data.size());
                return;
            }

            std::lock_guard<std::mutex> lock(box_id_grid_mutex_);
            if (lock_box_id_grid_) {
                return;
            }

            box_id_grid_[0][0] = msg->data[0];
            box_id_grid_[0][1] = msg->data[1];
            box_id_grid_[0][2] = msg->data[2];
            box_id_grid_[0][3] = msg->data[3];
            box_id_grid_[1][0] = msg->data[4];
            box_id_grid_[1][1] = msg->data[5];
            box_id_grid_[1][2] = msg->data[6];
            box_id_grid_[1][3] = msg->data[7];
            sem_post(&box_id_grid_sem_);
        });

    subscriptions_ready_ = true;
}

void GeneratePlaneAction::init_publishers(const rclcpp::Node::SharedPtr& node) {
    if (publishers_ready_) {
        return;
    }

    arm_cmd_pub_ = node->create_publisher<std_msgs::msg::Int32>("arm_cmd", 10);
    publishers_ready_ = true;
}



BT::Status GeneratePlaneAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (generated) {
        return BT::SUCCESS;
    }

    init_subscriptions(context->node_);
    RCLCPP_INFO(context->node_->get_logger(), box_grid_sub_ ? "创建接收方话题 box_id_grid 成功" : "创建接收方话题 box_id_grid 失败");
    init_publishers(context->node_);
    RCLCPP_INFO(context->node_->get_logger(), arm_cmd_pub_ ? "创建发送方 arm_cmd 成功" : "创建发送方 arm_cmd 失败");

    if (!wait_for_stage(context, Robot::kTreeGeneratePlan)) {
        return BT::FAILED;
    }

    if (first_run_) {
        if (context->auto_pilot_enabled.load()) {
            context->cmd.mode = 1;
        }
        if (!wait_with_interrupt(context, 3s)) {
            return BT::FAILED;
        }

        std_msgs::msg::Int32 msg;
        msg.data = kArmboxid;
        arm_cmd_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "发送arm_cmd 消息 %d", msg.data);

        if (!wait_semaphore_with_timeout(&box_id_grid_sem_, box_id_kSemaphoreTimeout)) {
            RCLCPP_ERROR(context->node_->get_logger(), "等待 box_id_grid 超时");
            return BT::FAILED;
        }

        if (!wait_for_start(context)) {
            return BT::FAILED;
        }

        {
            std::lock_guard<std::mutex> lock(box_id_grid_mutex_);
            lock_box_id_grid_ = true;
        }
        drain_semaphore(&box_id_grid_sem_);
        RCLCPP_INFO(context->node_->get_logger(), "行为树开始运行");
        first_run_ = false;
    }

    const auto config_path = context->node_->get_parameter(kGeneratePlanConfigParam).as_string();
    if (config_path.empty()) {
        RCLCPP_ERROR(context->node_->get_logger(), "GeneratePlaneAction: generate_plan_config 为空");
        return BT::FAILED;
    }

    PlanConfig plan_config;
    try {
        plan_config = load_plan_config(config_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(context->node_->get_logger(), "GeneratePlaneAction: 加载配置失败: %s", e.what());
        return BT::FAILED;
    }

    BoxInfo box_info;
    box_info.positions = plan_config.positions;
    {
        std::lock_guard<std::mutex> lock(box_id_grid_mutex_);
        box_info.box_ids = box_id_grid_;
    }

    for (const auto& line : box_info.box_ids) {
        for (const int box_id : line) {
            if (box_id < 0 || box_id >= 4) {
                RCLCPP_ERROR(context->node_->get_logger(), "箱子 ID=%d 越界，必须在 [0,3]", box_id);
                return BT::FAILED;
            }
        }
    }

    const std::array<float, 3> a1 = plan_config.route.a1;
    const std::array<float, 3> a2 = plan_config.route.a2;
    const int first_col = choose_first_col(plan_config, a1);
    const auto pick_order = make_fixed_pick_order(first_col);
    if (pick_order.size() != 8) {
        RCLCPP_ERROR(context->node_->get_logger(), "固定抓取顺序数量错误，当前为 %zu", pick_order.size());
        return BT::FAILED;
    }

    std::array<int, 4> placed_count{};
    std::array<float, 3> last_dst2 = a1;
    std::vector<MoveBoxPlan> move_plan;
    move_plan.reserve(4);
    bool first_plan = true;

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 根据 a1.y=%.3f 选择第 %d 列先清通道", a1[1], first_col + 1);

    for (size_t order_index = 0; order_index < pick_order.size(); order_index += 2) {
        const auto box0_selected = pick_order[order_index];
        const auto box1_selected = pick_order[order_index + 1];
        const auto box0_src = pose_from_grid(plan_config.positions, box0_selected.line + 1, box0_selected.col);
        const auto box1_src = pose_from_grid(plan_config.positions, box1_selected.line + 1, box1_selected.col);

        MoveBoxPlan plan;
        fill_box_task(plan.box0, plan_config, box_info, box0_selected);
        fill_box_task(plan.box1, plan_config, box_info, box1_selected);

        const auto box0_dst = pose_from_grid(plan_config.positions, 0, plan.box0.box_id);
        const auto box1_dst = pose_from_grid(plan_config.positions, 0, plan.box1.box_id);

        const bool is_first_plan = first_plan;
        if (is_first_plan) {
            plan.box0.to_box.trajectory.push_back(a1);
            plan.box0.to_box.target_points.push_back(plan_config.start_to_a1);
            first_plan = false;
        } else {
            plan.box0.to_box.trajectory.push_back(last_dst2);
            plan.box0.to_box.target_points.push_back(plan_config.dst0_to_dst2);
        }
        plan.box0.to_box.trajectory.push_back(box0_src);
        plan.box0.to_box.target_points.push_back(is_first_plan ? plan_config.a1_to_box0 : plan_config.dst2_to_box0);

        plan.box1.to_box.trajectory.push_back(box1_src);
        plan.box1.to_box.target_points.push_back(plan_config.box0_to_box1);

        if (is_first_plan) {
            plan.box1.to_dst.trajectory.push_back(a2);
            plan.box1.to_dst.target_points.push_back(plan_config.box1_to_a2);
        }
        plan.box1.to_dst.trajectory.push_back(box1_dst);
        plan.box1.to_dst.target_points.push_back(is_first_plan ? plan_config.a2_to_dst1 : plan_config.box1_to_dst1);
        plan.box0.to_dst.trajectory.push_back(box0_dst);
        plan.box0.to_dst.target_points.push_back(plan_config.dst1_to_dst0);

        // 放置顺序是 box1 再 box0，因此二层计数也按这个顺序更新。
        plan.box1.place_at_second_floor = placed_count[plan.box1.box_id] > 0;
        ++placed_count[plan.box1.box_id];
        plan.box0.place_at_second_floor = placed_count[plan.box0.box_id] > 0;
        ++placed_count[plan.box0.box_id];

        plan.dst2_pos = {box0_dst[0] - 0.1f, box0_dst[1], 3.14};
        plan.dst0_to_dst2 = plan_config.dst0_to_dst2;
        last_dst2 = plan.dst2_pos;

        RCLCPP_INFO(
            context->node_->get_logger(),
            "生成双箱计划: box0(line=%d,col=%d,id=%d,second=%s), box1(line=%d,col=%d,id=%d,second=%s)",
            plan.box0.line,
            plan.box0.col,
            plan.box0.box_id,
            plan.box0.place_at_second_floor ? "true" : "false",
            plan.box1.line,
            plan.box1.col,
            plan.box1.box_id,
            plan.box1.place_at_second_floor ? "true" : "false");

        move_plan.push_back(plan);
    }

    tree.write_msg("move_plan", move_plan);
    tree.write_msg<int>("plan_index", 0);

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 已生成 %zu 轮双箱移动计划", move_plan.size());

    generated = true;
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }
    return BT::SUCCESS;
}
