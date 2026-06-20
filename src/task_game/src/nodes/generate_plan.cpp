#include "nodes/generate_plan.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <thread>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

//constexpr auto kSemaphoreTimeout = 10s;
 const auto kSemaphoreTimeout = std::chrono::hours(24 * 365 * 100);
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
    RoutePoints route_a{};
    RoutePoints route_c{};
    RoutePoints route_e{};
    TargetPoint start_to_a1{};
    TargetPoint a1_to_a2{};
    TargetPoint a2_to_a3{};
    TargetPoint a3_to_src{};
    TargetPoint src_to_dst{};
    TargetPoint dst_to_dst2{};
    TargetPoint dst2_to_src{};
};

// 箱子布局、位置和 VIP ID 的聚合信息。
struct BoxInfo {
    BoxIdGrid box_ids{};
    PositionGrid positions{};
    int vip_box_id{-1};

    // 判断 VIP 箱子是否出现在第一条抓取线。
    bool is_vip_in_first_line() const {
        for (int col = 0; col < 4; ++col) {
            if (box_ids[0][col] == vip_box_id) {
                return true;
            }
        }
        return false;
    }

    // 查找 vip_box_id 在 box_ids 中的所有索引 (line, col)
    // 支持同一个 ID 出现多次（如两个 VIP 箱子有相同 ID）
    std::vector<std::pair<int, int>> find_vip_indices() const {
        std::vector<std::pair<int, int>> results;
        for (int line = 0; line < static_cast<int>(box_ids.size()); ++line) {
            for (int col = 0; col < static_cast<int>(box_ids[line].size()); ++col) {
                if (box_ids[line][col] == vip_box_id) {
                    results.emplace_back(line, col);
                }
            }
        }
        return results;
    }
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

RoutePoints select_route_points(const PlanConfig& config, char plan_case) {
    if (plan_case == 'A') {
        return config.route_a;
    }
    if (plan_case == 'C') {
        return config.route_c;
    }
    return config.route_e;
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

    config.route_a.a1 = read_point3(routes["A"]["a1"], "routes.A.a1");
    config.route_a.a2 = read_point3(routes["A"]["a2"], "routes.A.a2");
    config.route_c.a1 = read_point3(routes["C"]["a1"], "routes.C.a1");
    config.route_c.a2 = read_point3(routes["C"]["a2"], "routes.C.a2");
    config.route_e.a1 = read_point3(routes["E"]["a1"], "routes.E.a1");
    config.route_e.a2 = read_point3(routes["E"]["a2"], "routes.E.a2");

    config.start_to_a1 = read_target_point(target_points["start_to_a1"], "target_points.start_to_a1");
    config.a1_to_a2 = read_target_point(target_points["a1_to_a2"], "target_points.a1_to_a2");
    config.a2_to_a3 = read_target_point(target_points["a2_to_a3"], "target_points.a2_to_a3");
    config.a3_to_src = read_target_point(target_points["a3_to_src"], "target_points.a3_to_src");
    config.src_to_dst = read_target_point(target_points["src_to_dst"], "target_points.src_to_dst");
    config.dst_to_dst2 = read_target_point(target_points["dst_to_dst2"], "target_points.dst_to_dst2");
    config.dst2_to_src = read_target_point(target_points["dst2_to_src"], "target_points.dst2_to_src");
    return config;
}

// 计算两点之间的欧几里得距离（仅考虑 x, y）
float calc_distance(const std::array<float, 3>& point, const std::pair<float, float>& target) {
    float dx = point[0] - target.first;
    float dy = point[1] - target.second;
    return std::sqrt(dx * dx + dy * dy);
}
// 计算点 a 到 positions 第 line 条抓取线各点的最小距离和对应的列索引
struct NearestInfo {
    int col{-1};                    // 最近点的列索引
    float dist{std::numeric_limits<float>::max()};  // 最小距离
};

NearestInfo find_nearest_box(const std::array<float, 3>& point,
                             const BoxInfo& box_info,
                             int box_line,
                             const std::array<std::array<bool, 4>, 2>& picked,
                             const std::function<bool(int)>& accept_box_id) {
    NearestInfo result;
    const int position_line = box_line + 1;
    for (int col = 0; col < 4; ++col) {
        if (picked[box_line][col] || !accept_box_id(box_info.box_ids[box_line][col])) {
            continue;
        }

        const float dist = calc_distance(point, {
            std::get<0>(box_info.positions[position_line][col]),
            std::get<1>(box_info.positions[position_line][col])});
        if (dist < result.dist) {
            result.dist = dist;
            result.col = col;
        }
    }
    return result;
}

// 按 VIP 所在边列选择进入路线：左右边都有 VIP 走 C，仅左边走 E，其余走 A。
char detect_case(const BoxIdGrid& boxes, int vip_id) {
    bool in_col_0 = false;
    bool in_col_3 = false;

    for (int line = 0; line < 2; ++line) {
        if (boxes[line][0] == vip_id) {
            in_col_0 = true;
        }
        if (boxes[line][3] == vip_id) {
            in_col_3 = true;
        }
    }

    if (in_col_0 && in_col_3) {
        return 'C';
    }
    if (in_col_0) {
        return 'E';
    }
    return 'A';
}

}  // namespace

GeneratePlaneAction::GeneratePlaneAction()
    : BT::ActionNode("generate_plan_action") {
    sem_init(&vip_box_id_sem_, 0, 0);
    sem_init(&box_id_grid_sem_, 0, 0);
}

GeneratePlaneAction::~GeneratePlaneAction() {
    sem_destroy(&vip_box_id_sem_);
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

// 初始化 ROS 订阅，只创建一次订阅者，等待后续消息填充规划输入。
void GeneratePlaneAction::init_subscriptions(const rclcpp::Node::SharedPtr& node) {
    if (subscriptions_ready_) {
        return;
    }

    node_ = node;
    node_->declare_parameter<std::string>(kGeneratePlanConfigParam, "");
    vip_box_id_sub_ = node_->create_subscription<robot_msgs::msg::Int>(
        "vip_box_id", 10,
        [this](const robot_msgs::msg::Int& msg) {
            vip_box_id_ = msg.data;
            sem_post(&vip_box_id_sem_);
        });

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

// 行为树动作入口：等待箱子布局和 VIP ID 后生成完整搬箱顺序。
BT::Status GeneratePlaneAction::execute(BT& tree) {
    auto* context = tree.get_context<Robot>();
    if (!context) {
        return BT::FAILED;
    }

    if (generated) {
        return BT::SUCCESS;
    }

    init_subscriptions(context->node_);
    init_publishers(context->node_);

    if (!wait_for_stage(context, Robot::kTreeGeneratePlan)) {
        return BT::FAILED;
    }

    // if (!wait_with_interrupt(context, 3s)) {
    //     return BT::FAILED;
    // }
    if (first_run_) {
        if (context->auto_pilot_enabled.load()) {
            context->cmd.mode = 1;
        }
        if (!wait_with_interrupt(context, 3s)) {
            return BT::FAILED;
        }

        std_msgs::msg::Int32 msg;
        msg.data = 5;

        arm_cmd_pub_->publish(msg);

        if (!wait_semaphore_with_timeout(&box_id_grid_sem_, kSemaphoreTimeout)) {
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
        first_run_ = false;
    }



    //此处有个话题用来触发工业相机识别VIP_ID功能，暂定，无需改动

    if (!wait_semaphore_with_timeout(&vip_box_id_sem_, kSemaphoreTimeout)) {
        RCLCPP_ERROR(context->node_->get_logger(), "等待 vip_box_id 超时");
        return BT::FAILED;
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

    //位置数据
    box_info.positions = plan_config.positions;

    //箱子ID数据
    {
        std::lock_guard<std::mutex> lock(box_id_grid_mutex_);
        box_info.box_ids = box_id_grid_;
    }
    //VIP箱子ID
    box_info.vip_box_id = vip_box_id_;

    // 查找 vip 箱子在 box_ids 中的所有索引 (line, col)
    auto vip_indices = box_info.find_vip_indices();
    for (size_t i = 0; i < vip_indices.size(); ++i) {
        RCLCPP_INFO(context->node_->get_logger(), "VIP 箱子 %zu: line=%d, col=%d",
                    i, vip_indices[i].first, vip_indices[i].second);
    }

    //判断VIP箱子是否在第一条抓取线
    const bool vip_in_first_line = box_info.is_vip_in_first_line();
    //判断VIP箱子列的信息
    const char plan_case = detect_case(box_info.box_ids, box_info.vip_box_id);
    const auto route_points = select_route_points(plan_config, plan_case);

    std::array<float, 3> a1 = route_points.a1;
    std::array<float, 3> a2 = route_points.a2;

    // 根据进入路线设置机器人先到达的两个过渡点。
    if (plan_case == 'A') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 A");
    } else if (plan_case == 'C') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 C");
    } else if (plan_case == 'E') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 E");
    }

    //此时走到第一排箱子区域
    //A：65 C：67 E：69

    // 记录两排抓取区中已经加入计划的箱子，避免重复规划。
    std::array<std::array<bool, 4>, 2> picked{};
    // 记录每种箱子 ID 已放置次数，第二次放置同 ID 时需要放到第二层。
    std::array<int, 4> placed_count{};
    // 上一次规划结束时的期望位置，用于选择最近的下一个箱子。
    std::array<float, 3> last_expected_pos = a2;
    // 放置点后的退让点，x 方向相对放置点后退 0.5m，避免下一次转向时碰到放置区箱子。
    std::array<float, 3> last_dst2 = a2;

    //计移动箱子计划数容器
    std::vector<MoveBoxPlan> move_plan;
    //移动8个箱子，所以应该有8个计划
    move_plan.reserve(8);
    //狗到中间标志位
    bool dog_to_middle = false;

    // 封装单次抓取/放置计划的生成逻辑。
    auto append_plan = [&](int box_line, int col) {
        const int position_line = box_line + 1;
        const int box_id = box_info.box_ids[box_line][col];
        const std::array<float, 2> src_box_pos = plan_config.arm_box_positions[position_line][col];
        const std::array<float, 2> dst_box_pos = plan_config.arm_box_positions[0][box_id];
        const std::array<float, 3> src = {
            std::get<0>(box_info.positions[position_line][col]),
            std::get<1>(box_info.positions[position_line][col]),
            std::get<2>(box_info.positions[position_line][col])};
        const std::array<float, 3> dst = {
            std::get<0>(box_info.positions[0][box_id]),
            std::get<1>(box_info.positions[0][box_id]),
            std::get<2>(box_info.positions[0][box_id])};
        const std::array<float, 3> dst2 = {
            dst[0] - 0.1f,
            dst[1],
            dst[2]};

        const std::array<float, 3> a3 = {
            a2[0],
            src[1],
            src[2]};

        MoveBoxPlan plan;
        if (!dog_to_middle) {
            plan.catch_trajectory.push_back(a1);
            plan.target_point.push_back(plan_config.start_to_a1);
            plan.catch_trajectory.push_back(a2);
            plan.target_point.push_back(plan_config.a1_to_a2);
            plan.catch_trajectory.push_back(a3);
            plan.target_point.push_back(plan_config.a2_to_a3);
            plan.catch_trajectory.push_back(src);
            plan.target_point.push_back(plan_config.a3_to_src);
            dog_to_middle = true;
        } else {
            plan.catch_trajectory.push_back(last_dst2);
            plan.target_point.push_back(plan_config.dst_to_dst2);
            plan.catch_trajectory.push_back(src);
            plan.target_point.push_back(plan_config.dst2_to_src);
        }

        plan.place_trajectory.push_back(dst);
        plan.target_point.push_back(plan_config.src_to_dst);
        plan.src_box_pos = src_box_pos;
        plan.dst_box_pos = dst_box_pos;
        plan.place_at_second_floor = placed_count[box_id] > 0;
        move_plan.push_back(plan);

        picked[box_line][col] = true;
        ++placed_count[box_id];
        last_expected_pos = dst2;
        last_dst2 = dst2;

        RCLCPP_INFO(
            context->node_->get_logger(),
            "生成抓取计划: line=%d, col=%d, box_id=%d, second_floor=%s",
            box_line,
            col,
            box_id,
            plan.place_at_second_floor ? "true" : "false");
    };

    auto pick_nearest_in_line = [&](int box_line, const std::function<bool(int)>& accept_box_id) {
        const auto nearest = find_nearest_box(last_expected_pos, box_info, box_line, picked, accept_box_id);
        if (nearest.col < 0) {
            return false;
        }

        append_plan(box_line, nearest.col);
        return true;
    };

    while (pick_nearest_in_line(0, [&](int box_id) { return box_id == box_info.vip_box_id; })) {
    }
    while (pick_nearest_in_line(0, [](int) { return true; })) {
    }
    while (pick_nearest_in_line(1, [&](int box_id) { return box_id == box_info.vip_box_id; })) {
    }
    while (pick_nearest_in_line(1, [](int) { return true; })) {
    }

    tree.write_msg("move_plan", move_plan);
    tree.write_msg<int>("plan_index", 0);

    RCLCPP_INFO(
        context->node_->get_logger(),
        "GeneratePlaneAction: case=%c, vip_box_id=%d, vip_in_first_line=%s, 已生成 %zu 条移动计划",
        plan_case,
        box_info.vip_box_id,
        vip_in_first_line ? "true" : "false",
        move_plan.size());

    generated=true;
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }
    return BT::SUCCESS;
}
