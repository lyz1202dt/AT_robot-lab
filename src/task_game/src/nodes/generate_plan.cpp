#include "nodes/generate_plan.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <algorithm>
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

constexpr int kArmCheckBoxID = 7;
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
    PositionGrid positions{};        // 后续轮次（box_positions_second）
    PositionGrid positions_first{};  // 第一轮（box_positions_first）
    BoxPositionGrid arm_box_positions{};
    RoutePoints route{};
    TargetPoint start_to_a0{};
    TargetPoint a0_to_a1{};
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

struct PickGroup {
    SelectedBox box0{};
    SelectedBox box1{};
    bool hand_only{false};
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
    const auto box_positions_second = root["box_positions_second"];
    const auto box_positions_first = root["box_positions_first"];
    const auto routes = root["routes"];
    const auto target_points = root["target_points"];
    if (!arm_box_positions || !box_positions_second || !box_positions_first || !routes || !target_points) {
        throw std::runtime_error("generate_plan.yaml 缺少 arm_box_positions/box_positions_first/box_positions_second/routes/target_points");
    }

    PlanConfig config;
    config.arm_box_positions[0] = read_point2_row(arm_box_positions["arm_place"], "arm_box_positions.arm_place");
    config.arm_box_positions[1] = read_point2_row(arm_box_positions["arm_pick_line_0"], "arm_box_positions.arm_pick_line_0");
    config.arm_box_positions[2] = read_point2_row(arm_box_positions["arm_pick_line_1"], "arm_box_positions.arm_pick_line_1");
    config.positions[0] = read_point3_row(box_positions_second["place"], "box_positions_second.place");
    config.positions[1] = read_point3_row(box_positions_second["pick_line_0"], "box_positions_second.pick_line_0");
    config.positions[2] = read_point3_row(box_positions_second["pick_line_1"], "box_positions_second.pick_line_1");
    // 第一轮只使用抓取线位姿；放置位统一取 box_positions_second，故不读取 box_positions_first.place。
    config.positions_first[1] = read_point3_row(box_positions_first["pick_line_0"], "box_positions_first.pick_line_0");
    config.positions_first[2] = read_point3_row(box_positions_first["pick_line_1"], "box_positions_first.pick_line_1");

    config.route.a0 = read_point3(routes["a0"], "routes.a0");
    config.route.a1 = read_point3(routes["a1"], "routes.a1");
    config.route.a2 = read_point3(routes["a2"], "routes.a2");

    config.start_to_a0 = read_target_point(target_points["start_to_a0"], "target_points.start_to_a0");
    config.a0_to_a1 = read_target_point(target_points["a0_to_a1"], "target_points.a0_to_a1");
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

bool same_box(const SelectedBox& lhs, const SelectedBox& rhs) {
    return lhs.line == rhs.line && lhs.col == rhs.col;
}

float box_y(const PlanConfig& plan_config, const SelectedBox& selected) {
    return pose_from_grid(plan_config.positions, selected.line + 1, selected.col)[1];
}

constexpr int kDoneBoxId = 255;
constexpr int kMaxBoxIdCount = 2;

std::vector<SelectedBox> make_available_boxes(const BoxInfo& box_info) {
    std::vector<SelectedBox> boxes;
    boxes.reserve(8);
    for (int col = 0; col < 4; ++col) {
        for (int line = 0; line < 2; ++line) {
            if (box_info.box_ids[line][col] != kDoneBoxId) {
                boxes.push_back({line, col});
            }
        }
    }
    return boxes;
}

// box_id_grid 含 255 即为重试：至少有一个位置已完成。
bool is_retry_grid(const BoxIdGrid& box_ids) {
    for (const auto& line : box_ids) {
        for (const int box_id : line) {
            if (box_id == kDoneBoxId) {
                return true;
            }
        }
    }
    return false;
}

void remove_selected_box(std::vector<SelectedBox>& boxes, const SelectedBox& selected) {
    boxes.erase(
        std::remove_if(boxes.begin(), boxes.end(), [&](const SelectedBox& box) {
            return same_box(box, selected);
        }),
        boxes.end());
}

SelectedBox choose_nearest_box_by_y(const PlanConfig& plan_config, const std::vector<SelectedBox>& boxes, float target_y) {
    if (boxes.empty()) {
        throw std::runtime_error("没有可选箱子");
    }

    auto best = boxes.front();
    float best_error = std::numeric_limits<float>::max();
    for (const auto& box : boxes) {
        const float y_error = std::abs(box_y(plan_config, box) - target_y);
        if (y_error < best_error) {
            best = box;
            best_error = y_error;
        }
    }
    return best;
}

SelectedBox choose_nearest_box_on_line_by_y(const PlanConfig& plan_config,
                                            const std::vector<SelectedBox>& boxes,
                                            int line,
                                            float target_y) {
    std::vector<SelectedBox> line_boxes;
    line_boxes.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (box.line == line) {
            line_boxes.push_back(box);
        }
    }

    return choose_nearest_box_by_y(plan_config, line_boxes, target_y);
}

SelectedBox choose_pair_box(const PlanConfig& plan_config, const std::vector<SelectedBox>& boxes, const SelectedBox& box0) {
    const SelectedBox same_col_pair{1 - box0.line, box0.col};
    for (const auto& box : boxes) {
        if (same_box(box, same_col_pair)) {
            return box;
        }
    }

    return choose_nearest_box_by_y(plan_config, boxes, box_y(plan_config, box0));
}

std::vector<PickGroup> make_dst2_nearest_pick_groups(const PlanConfig& plan_config,
                                                    int first_col,
                                                    float& current_dst2_y,
                                                    const BoxInfo& box_info) {
    std::vector<PickGroup> pick_groups;
    pick_groups.reserve(4);

    std::array<int, 4> col_counts{};
    for (const auto& box : make_available_boxes(box_info)) {
        ++col_counts[box.col];
    }

    std::vector<SelectedBox> remaining_boxes;
    std::vector<SelectedBox> tail_boxes;
    remaining_boxes.reserve(8);
    tail_boxes.reserve(4);
    for (const auto& box : make_available_boxes(box_info)) {
        if (col_counts[box.col] == 1) {
            tail_boxes.push_back(box);
        } else {
            remaining_boxes.push_back(box);
        }
    }

    bool first_pair = true;
    const SelectedBox first_box0{1, first_col};
    const SelectedBox first_box1{0, first_col};
    while (!remaining_boxes.empty()) {
        SelectedBox box0{};
        if (first_pair) {
            const auto first_it = std::find_if(remaining_boxes.begin(), remaining_boxes.end(), [&](const SelectedBox& box) {
                return same_box(box, first_box0);
            });
            if (first_it != remaining_boxes.end()) {
                box0 = *first_it;
            } else {
                const auto pair_it = std::find_if(remaining_boxes.begin(), remaining_boxes.end(), [&](const SelectedBox& box) {
                    return same_box(box, first_box1);
                });
                box0 = pair_it != remaining_boxes.end() ? *pair_it : choose_nearest_box_by_y(plan_config, remaining_boxes, current_dst2_y);
            }
        } else {
            try {
                box0 = choose_nearest_box_on_line_by_y(plan_config, remaining_boxes, 0, current_dst2_y);
            } catch (const std::runtime_error&) {
                box0 = choose_nearest_box_by_y(plan_config, remaining_boxes, current_dst2_y);
            }
        }
        first_pair = false;
        remove_selected_box(remaining_boxes, box0);

        if (remaining_boxes.empty()) {
            tail_boxes.push_back(box0);
            break;
        }

        const auto box1 = choose_pair_box(plan_config, remaining_boxes, box0);
        remove_selected_box(remaining_boxes, box1);
        pick_groups.push_back({box0, box1, false});

        const int box0_id = box_info.box_ids[box0.line][box0.col];
        current_dst2_y = pose_from_grid(plan_config.positions, 0, box0_id)[1];
    }

    std::sort(tail_boxes.begin(), tail_boxes.end(), [&](const SelectedBox& lhs, const SelectedBox& rhs) {
        return std::abs(box_y(plan_config, lhs) - current_dst2_y) < std::abs(box_y(plan_config, rhs) - current_dst2_y);
    });
    for (const auto& box : tail_boxes) {
        pick_groups.push_back({box, box, true});
        const int box_id = box_info.box_ids[box.line][box.col];
        current_dst2_y = pose_from_grid(plan_config.positions, 0, box_id)[1];
    }

    return pick_groups;
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

void fill_box_task(BoxMoveTask& task,
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

// 第一轮 box0 去箱轨迹：
// - 正常启动(非重试)：start->a0->a1->box0；
// - 重试(box_id_grid 含 255)：start->a0->box0（跳过 a1）。
// 后续轮：dst2->box0。
TrajectoryPlan make_to_box_plan(const PlanConfig& plan_config,
                                const std::array<float, 3>& src,
                                bool first_plan,
                                bool retry,
                                const std::array<float, 3>& a0,
                                const std::array<float, 3>& a1) {
    TrajectoryPlan to_box;
    if (first_plan) {
        to_box.trajectory.push_back(a0);
        to_box.target_points.push_back(plan_config.start_to_a0);
        if (!retry) {
            to_box.trajectory.push_back(a1);
            to_box.target_points.push_back(plan_config.a0_to_a1);
        }
        to_box.trajectory.push_back(src);
        to_box.target_points.push_back(plan_config.a1_to_box0);
    } else {
        to_box.trajectory.push_back(src);
        to_box.target_points.push_back(plan_config.dst2_to_box0);
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
            grid_pub_ = context->node_->create_publisher<std_msgs::msg::Int32MultiArray>("box_id_grid", 10);
            auto grid_msg = std_msgs::msg::Int32MultiArray();
            grid_msg.data = {
                255, 255, 255, 255,
                1, 0, 2, 3
            };
            grid_pub_->publish(grid_msg);
            // std_msgs::msg::Int32 msg;
            // msg.data = kArmCheckBoxID;
            // arm_cmd_pub_->publish(msg);
        }

        

        // 注意：新流程不再发送 arm_cmd=7。box_id_grid 仅作为默认 ID 来源，
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
    const int first_col = choose_first_col(plan_config, a1);
    float current_dst2_y = a1[1];
    const auto pick_groups = make_dst2_nearest_pick_groups(plan_config, first_col, current_dst2_y, box_info);
    const bool retry = is_retry_grid(box_info.box_ids);

    std::vector<MoveBoxPlan> move_plan;
    move_plan.reserve(pick_groups.size());
    bool first_plan = true;

    RCLCPP_INFO(context->node_->get_logger(), "GeneratePlaneAction: 根据 a1.y=%.3f 选择第 %d 列先清通道，%s，剩余生成 %zu 轮计划", a1[1], first_col + 1, retry ? "重试(跳过a1)" : "正常启动", pick_groups.size());

    for (const auto& group : pick_groups) {
        const auto box0_selected = group.box0;
        const auto box1_selected = group.box1;
        const bool is_first_plan = first_plan;
        // 第一轮 box0/box1 的导航抓取位使用 box_positions_first，后续轮使用 box_positions_second。
        const PositionGrid& pick_positions = is_first_plan ? plan_config.positions_first : plan_config.positions;
        const auto box0_src = pose_from_grid(pick_positions, box0_selected.line + 1, box0_selected.col);
        const auto box1_src = pose_from_grid(pick_positions, box1_selected.line + 1, box1_selected.col);

        MoveBoxPlan plan;
        try {
            fill_box_task(plan.box0, plan_config, box_info, box0_selected);
            fill_box_task(plan.box1, plan_config, box_info, box1_selected);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(context->node_->get_logger(), "GeneratePlaneAction: %s", e.what());
            return BT::FAILED;
        }

        const auto box0_dst = pose_from_grid(plan_config.positions, 0, plan.box0.box_id);
        const auto box1_dst = pose_from_grid(plan_config.positions, 0, plan.box1.box_id);
        plan.box0.to_box = make_to_box_plan(plan_config, box0_src, is_first_plan, retry, a0, a1);
        first_plan = false;

        if (group.hand_only) {
            plan.hand_only_plan = true;
            plan.box0.to_box = {};
            plan.box0.to_dst = {};
            plan.box1.to_box = make_to_box_plan(plan_config, box1_src, is_first_plan, retry, a0, a1);
            if (is_first_plan) {
                plan.box1.to_dst.trajectory.push_back(a2);
                plan.box1.to_dst.target_points.push_back(plan_config.box1_to_a2);
            }
            plan.box1.to_dst.trajectory.push_back(box1_dst);
            plan.box1.to_dst.target_points.push_back(is_first_plan ? plan_config.a2_to_dst1 : plan_config.box1_to_dst1);
            plan.dst2_pos = {box1_dst[0] - 0.05f, box1_dst[1], box1_dst[2]};
            RCLCPP_INFO(
                context->node_->get_logger(),
                "生成单吸手放计划: box(line=%d,col=%d,默认id=%d), dst2.y=%.3f",
                plan.box1.line,
                plan.box1.col,
                plan.box1.box_id,
                plan.dst2_pos[1]);
        } else {
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
            plan.dst2_pos = {box0_dst[0] - 0.05f, box0_dst[1], box0_dst[2]};
            RCLCPP_INFO(
                context->node_->get_logger(),
                "生成双箱计划: box0(line=%d,col=%d,默认id=%d), box1(line=%d,col=%d,默认id=%d), dst2.y=%.3f",
                plan.box0.line,
                plan.box0.col,
                plan.box0.box_id,
                plan.box1.line,
                plan.box1.col,
                plan.box1.box_id,
                plan.dst2_pos[1]);
        }

        plan.dst0_to_dst2 = plan_config.dst0_to_dst2;
        move_plan.push_back(plan);
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
