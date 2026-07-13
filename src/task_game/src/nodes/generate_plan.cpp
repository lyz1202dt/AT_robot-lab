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
    std::array<float, 3> a0{};
    std::array<float, 3> a1{};
    std::array<float, 3> a2{};
};

struct PlanConfig {
    PositionGrid positions{};
    BoxPositionGrid arm_box_positions{};
    RoutePoints route{};
    TargetPoint start_to_a0{};
    TargetPoint a0_to_a1{};
    TargetPoint a1_to_a2{};
    TargetPoint a2_to_a3{};
    TargetPoint a3_to_src{};
    TargetPoint src_to_dst{};
    TargetPoint dst0_to_dst2{};
    TargetPoint dst2_to_src{};
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

    config.route.a0 = read_point3(routes["a0"], "routes.a0");
    config.route.a1 = read_point3(routes["a1"], "routes.a1");
    config.route.a2 = read_point3(routes["a2"], "routes.a2");

    config.start_to_a0 = read_target_point(target_points["start_to_a0"], "target_points.start_to_a0");
    config.a0_to_a1 = read_target_point(target_points["a0_to_a1"], "target_points.a0_to_a1");
    config.a1_to_a2 = read_target_point(target_points["a1_to_a2"], "target_points.a1_to_a2");
    config.a2_to_a3 = read_target_point(target_points["a2_to_a3"], "target_points.a2_to_a3");
    config.a3_to_src = read_target_point(target_points["a3_to_src"], "target_points.a3_to_src");
    config.src_to_dst = read_target_point(target_points["src_to_dst"], "target_points.src_to_dst");
    config.dst0_to_dst2 = read_target_point(target_points["dst0_to_dst2"], "target_points.dst0_to_dst2");
    config.dst2_to_src = read_target_point(target_points["dst2_to_src"], "target_points.dst2_to_src");
    return config;
}

std::array<float, 3> pose_from_grid(const PositionGrid& positions, int line, int col) {
    return {
        std::get<0>(positions[line][col]),
        std::get<1>(positions[line][col]),
        std::get<2>(positions[line][col])};
}

float squared_distance_xy(const std::array<float, 3>& lhs, const std::array<float, 3>& rhs) {
    const float dx = lhs[0] - rhs[0];
    const float dy = lhs[1] - rhs[1];
    return dx * dx + dy * dy;
}

constexpr int kDoneBoxId = 255;
constexpr int kMaxBoxIdCount = 2;

std::vector<SelectedBox> make_serial_pick_sequence(const PlanConfig& plan_config,
                                                   const BoxInfo& box_info,
                                                   const std::array<float, 3>& initial_expected_pos) {
    std::vector<SelectedBox> sequence;
    sequence.reserve(8);

    std::array<std::array<bool, 4>, 2> picked{};
    std::array<float, 3> last_expected_pos = initial_expected_pos;

    auto pick_nearest_in_line = [&](int line) {
        while (true) {
            int best_col = -1;
            float best_distance = std::numeric_limits<float>::max();
            for (int col = 0; col < 4; ++col) {
                if (picked[line][col] || box_info.box_ids[line][col] == kDoneBoxId) {
                    continue;
                }

                const auto candidate_pos = pose_from_grid(plan_config.positions, line + 1, col);
                const float distance = squared_distance_xy(last_expected_pos, candidate_pos);
                if (distance < best_distance) {
                    best_col = col;
                    best_distance = distance;
                }
            }

            if (best_col < 0) {
                break;
            }

            picked[line][best_col] = true;
            sequence.push_back({line, best_col});

            const int box_id = box_info.box_ids[line][best_col];
            if (box_id >= 0 && box_id < 4) {
                const auto dst = pose_from_grid(plan_config.positions, 0, box_id);
                last_expected_pos = {dst[0] - 0.05f, dst[1], dst[2]};
            } else {
                last_expected_pos = pose_from_grid(plan_config.positions, line + 1, best_col);
            }
        }
    };

    // 严格按逐箱最近邻串行顺序：先清第一抓取线，再清第二抓取线。
    pick_nearest_in_line(0);
    pick_nearest_in_line(1);
    return sequence;
}

bool normalize_box_id_grid(const std_msgs::msg::Int32MultiArray& msg, BoxIdGrid& box_id_grid, const rclcpp::Logger& logger) {
    std::array<int, 4> id_counts{};

    for (size_t i = 0; i < 8; ++i) {
        const int value = msg.data[i];
        if (value == kDoneBoxId) {
            continue;
        }

        if (value < 0 || value >= 4) {
            RCLCPP_ERROR(logger, "box_id_grid[%zu]=%d 非法，必须是 [0,3] 或 255；255 表示该位置已完成", i, value);
            return false;
        }

        ++id_counts[value];
        if (id_counts[value] > kMaxBoxIdCount) {
            RCLCPP_ERROR(logger, "box_id_grid 中 ID=%d 的剩余数量超过 %d", value, kMaxBoxIdCount);
            return false;
        }
    }

    box_id_grid[0][0] = msg.data[0];
    box_id_grid[0][1] = msg.data[1];
    box_id_grid[0][2] = msg.data[2];
    box_id_grid[0][3] = msg.data[3];
    box_id_grid[1][0] = msg.data[4];
    box_id_grid[1][1] = msg.data[5];
    box_id_grid[1][2] = msg.data[6];
    box_id_grid[1][3] = msg.data[7];
    return true;
}

void fill_box_task(MoveBoxPlan& task,
                   const PlanConfig& plan_config,
                   const BoxInfo& box_info,
                   const SelectedBox& selected) {
    const int position_line = selected.line + 1;
    const int box_id = box_info.box_ids[selected.line][selected.col];
    if (box_id < 0 || box_id >= 4) {
        throw std::runtime_error("选中的箱子 ID 非法，不能生成搬运任务");
    }
    task.pick_box_pos = plan_config.arm_box_positions[position_line][selected.col];
    task.place_box_pos = plan_config.arm_box_positions[0][box_id];
    task.box_id = box_id;
    task.line = selected.line;
    task.col = selected.col;
}

std::array<int, 4> make_initial_placed_count(const BoxIdGrid& box_ids) {
    std::array<int, 4> remaining_count{};
    for (const auto& line : box_ids) {
        for (const int box_id : line) {
            if (box_id >= 0 && box_id < 4) {
                ++remaining_count[box_id];
            }
        }
    }

    std::array<int, 4> placed_count{};
    for (int id = 0; id < 4; ++id) {
        placed_count[id] = std::clamp(kMaxBoxIdCount - remaining_count[id], 0, kMaxBoxIdCount);
    }
    return placed_count;
}

TrajectoryPlan make_to_box_plan(const PlanConfig& plan_config,
                                const std::array<float, 3>& src,
                                bool first_plan,
                                const std::array<float, 3>& a0,
                                const std::array<float, 3>& a1,
                                const std::array<float, 3>& a2) {
    TrajectoryPlan to_box;
    if (first_plan) {
        const std::array<float, 3> a3{src[0], a2[1], a2[2]};
        to_box.trajectory.push_back(a0);
        to_box.target_points.push_back(plan_config.start_to_a0);
        to_box.trajectory.push_back(a1);
        to_box.target_points.push_back(plan_config.a0_to_a1);
        to_box.trajectory.push_back(a2);
        to_box.target_points.push_back(plan_config.a1_to_a2);
        to_box.trajectory.push_back(a3);
        to_box.target_points.push_back(plan_config.a2_to_a3);
        to_box.trajectory.push_back(src);
        to_box.target_points.push_back(plan_config.a3_to_src);
    } else {
        to_box.trajectory.push_back(src);
        to_box.target_points.push_back(plan_config.dst2_to_src);
    }
    return to_box;
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

            if (!normalize_box_id_grid(*msg, box_id_grid_, node_->get_logger())) {
                return;
            }
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
        if (!wait_with_interrupt(context, 500ms)) {
            return BT::FAILED;
        }

        bool is_first_game = context->node_->get_parameter("is_first_game").as_bool();

        if(is_first_game)
        {
            // grid_pub_ = context->node_->create_publisher<std_msgs::msg::Int32MultiArray>("box_id_grid", 10);
            // auto grid_msg = std_msgs::msg::Int32MultiArray();
            // grid_msg.data = {
            //     255, 255, 255, 255,
            //     1, 0, 2, 3
            // };
            // grid_pub_->publish(grid_msg);
            // std_msgs::msg::Int32 msg;
            // arm_cmd_pub_->publish(msg);
        }

        

        // 单吸手上流程不发送识别触发 arm_cmd；box_id_grid 仅作为默认 ID 来源，
        // 真正每个箱子的放置区 ID 在抓取时由 pnp_box_index 话题给出。
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
            if ((box_id < 0 || box_id >= 4) && box_id != kDoneBoxId) {
                RCLCPP_ERROR(context->node_->get_logger(), "箱子 ID=%d 越界，必须在 [0,3] 或 255", box_id);
                return BT::FAILED;
            }
        }
    }

    const std::array<float, 3> a0 = plan_config.route.a0;
    const std::array<float, 3> a1 = plan_config.route.a1;
    const std::array<float, 3> a2 = plan_config.route.a2;
    const auto pick_sequence = make_serial_pick_sequence(plan_config, box_info, a1);

    std::vector<MoveBoxPlan> move_plan;
    move_plan.reserve(pick_sequence.size());
    bool first_plan = true;

    RCLCPP_INFO(
        context->node_->get_logger(),
        "GeneratePlaneAction: 固定 C 通道单箱流程，a0=(%.3f,%.3f,%.3f), a1=(%.3f,%.3f,%.3f), a2=(%.3f,%.3f,%.3f)，生成 %zu 个单箱计划",
        a0[0],
        a0[1],
        a0[2],
        a1[0],
        a1[1],
        a1[2],
        a2[0],
        a2[1],
        a2[2],
        pick_sequence.size());

    for (const auto& selected : pick_sequence) {
        const bool is_first_plan = first_plan;
        const auto src = pose_from_grid(plan_config.positions, selected.line + 1, selected.col);

        MoveBoxPlan plan;
        try {
            fill_box_task(plan, plan_config, box_info, selected);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(context->node_->get_logger(), "GeneratePlaneAction: %s", e.what());
            return BT::FAILED;
        }

        const auto dst = pose_from_grid(plan_config.positions, 0, plan.box_id);
        plan.to_box = make_to_box_plan(plan_config, src, is_first_plan, a0, a1, a2);
        plan.to_dst.trajectory.push_back(dst);
        plan.to_dst.target_points.push_back(plan_config.src_to_dst);
        plan.dst2_pos = {dst[0] - 0.05f, dst[1], dst[2]};
        plan.dst0_to_dst2 = plan_config.dst0_to_dst2;
        move_plan.push_back(plan);
        first_plan = false;

        RCLCPP_INFO(
            context->node_->get_logger(),
            "生成单箱计划: line=%d,col=%d,默认id=%d,dst2=(%.3f,%.3f,%.3f)",
            plan.line,
            plan.col,
            plan.box_id,
            plan.dst2_pos[0],
            plan.dst2_pos[1],
            plan.dst2_pos[2]);
    }

    // 放置区查表存到行为树黑板，供抓取时按运行期 ID 查放置位：
    // place_table[id] = 放置点机器狗导航位 [x,y,yaw]；
    // arm_place_table[id] = 机械臂放置位 [x,y]；
    // placed_count[id] = 运行期已成功放置计数，用于二层判定。
    std::array<std::array<float, 3>, 4> place_table{};
    std::array<std::array<float, 2>, 4> arm_place_table{};
    for (int id = 0; id < 4; ++id) {
        place_table[id] = pose_from_grid(plan_config.positions, 0, id);
        arm_place_table[id] = plan_config.arm_box_positions[0][id];
    }
    tree.write_msg("place_table", place_table);
    tree.write_msg("arm_place_table", arm_place_table);
    tree.write_msg("placed_count", make_initial_placed_count(box_info.box_ids));

    tree.write_msg("move_plan", move_plan);
    tree.write_msg<int>("plan_index", 0);

    if (move_plan.empty()) {
        RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: box_id_grid 全部为 255，没有剩余箱子需要搬运");
        generated = true;
        context->enter_manual_mode();
        return BT::SUCCESS;
    }

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 已生成 %zu 轮移动计划", move_plan.size());

    generated = true;
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }
    return BT::SUCCESS;
}
