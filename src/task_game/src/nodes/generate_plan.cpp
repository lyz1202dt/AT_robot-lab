#include "nodes/generate_plan.hpp"
#include "nodes/msg.hpp"
#include "core/robot.hpp"
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <functional>
#include <limits>
#include <rclcpp/logging.hpp>
#include <utility>
#include <vector>

using namespace std::chrono_literals;

namespace {

constexpr auto kSemaphoreTimeout = 10s;

bool wait_for_stage(Robot* context, int32_t expected_stage) {
    while (rclcpp::ok() && context->auto_pilot_enabled.load() && context->tree_start_key.load() != expected_stage) {
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

// 三行四列的位置表：第 0 行为放置区，第 1/2 行为两排抓取区。
using PositionGrid = std::array<std::array<std::pair<float, float>, 4>, 3>;

// 箱子布局、位置和 VIP ID 的聚合信息。
struct BoxInfo {
    BoxIdGrid box_ids{};
    PositionGrid positions{};
    int vip_box_id{-1};

    // 判断 VIP 箱子是否出现在第一行。
    bool is_vip_in_first_row() const {
        for (int col = 0; col < 4; ++col) {
            if (box_ids[0][col] == vip_box_id) {
                return true;
            }
        }
        return false;
    }

    // 查找 vip_box_id 在 box_ids 中的所有索引 (row, col)
    // 支持同一个 ID 出现多次（如两个 VIP 箱子有相同 ID）
    std::vector<std::pair<int, int>> find_vip_indices() const {
        std::vector<std::pair<int, int>> results;
        for (int row = 0; row < static_cast<int>(box_ids.size()); ++row) {
            for (int col = 0; col < static_cast<int>(box_ids[row].size()); ++col) {
                if (box_ids[row][col] == vip_box_id) {
                    results.emplace_back(row, col);
                }
            }
        }
        return results;
    }
};

// 计算两点之间的欧几里得距离（仅考虑 x, y）
float calc_distance(const std::array<float, 3>& point, const std::pair<float, float>& target) {
    float dx = point[0] - target.first;
    float dy = point[1] - target.second;
    return std::sqrt(dx * dx + dy * dy);
}
// 计算点 a 到 positions 第 row 行各点的最小距离和对应的列索引
struct NearestInfo {
    int col{-1};                    // 最近点的列索引
    float dist{std::numeric_limits<float>::max()};  // 最小距离
};

NearestInfo find_nearest_box(const std::array<float, 3>& point,
                             const BoxInfo& box_info,
                             int box_row,
                             const std::array<std::array<bool, 4>, 2>& picked,
                             const std::function<bool(int)>& accept_box_id) {
    NearestInfo result;
    const int position_row = box_row + 1;
    for (int col = 0; col < 4; ++col) {
        if (picked[box_row][col] || !accept_box_id(box_info.box_ids[box_row][col])) {
            continue;
        }

        const float dist = calc_distance(point, box_info.positions[position_row][col]);
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

    for (int row = 0; row < 2; ++row) {
        if (boxes[row][0] == vip_id) {
            in_col_0 = true;
        }
        if (boxes[row][3] == vip_id) {
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
}

// 初始化 ROS 订阅，只创建一次订阅者，等待后续消息填充规划输入。
void GeneratePlaneAction::init_subscriptions(const rclcpp::Node::SharedPtr& node) {
    if (subscriptions_ready_) {
        return;
    }

    node_ = node;
    vip_box_id_sub_ = node_->create_subscription<robot_msgs::msg::Int>(
        "vip_box_id", 10,
        [this](const robot_msgs::msg::Int& msg) {
            vip_box_id_ = msg.data;
            sem_post(&vip_box_id_sem_);
        });

    box_id_grid_sub_ = node_->create_subscription<robot_msgs::msg::BoxIdGrid>(
        "box_id_grid", 10,
        [this](const robot_msgs::msg::BoxIdGrid& msg) {
            box_id_grid_[0][0] = msg.data[0];
            box_id_grid_[0][1] = msg.data[1];
            box_id_grid_[0][2] = msg.data[2];
            box_id_grid_[0][3] = msg.data[3];
            box_id_grid_[1][0] = msg.data[4];
            box_id_grid_[1][1] = msg.data[5];
            box_id_grid_[1][2] = msg.data[6];
            box_id_grid_[1][3] = msg.data[7];
            sem_post(&box_id_grid_sem_);
        });

    subscriptions_ready_ = true;
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

    if (!wait_for_stage(context, Robot::kTreeGeneratePlan)) {
        return BT::FAILED;
    }

    if (!wait_with_interrupt(context, 3s)) {
        return BT::FAILED;
    }
    if (context->auto_pilot_enabled.load()) {
        context->cmd.mode = 1;
    }
    if (!wait_with_interrupt(context, 5s)) {
        return BT::FAILED;
    }

    //此处有个话题用来发布摄像头功能，暂定，无需改动

    if (!wait_semaphore_with_timeout(&vip_box_id_sem_, kSemaphoreTimeout)) {
        RCLCPP_ERROR(context->node_->get_logger(), "等待 vip_box_id 超时");
        return BT::FAILED;
    }
    if (!wait_semaphore_with_timeout(&box_id_grid_sem_, kSemaphoreTimeout)) {
        RCLCPP_ERROR(context->node_->get_logger(), "等待 box_id_grid 超时");
        return BT::FAILED;
    }

    BoxInfo box_info;

    //位置数据
    box_info.positions = {{
        //放置区位置
        {std::make_pair(15.0f, -10.0f), std::make_pair(15.0f, -5.0f), std::make_pair(15.0f, 5.0f), std::make_pair(15.0f, 10.0f)},
        //抓取区位置，第一行
        {std::make_pair(10.0f, -10.0f), std::make_pair(10.0f, -5.0f), std::make_pair(10.0f, 5.0f), std::make_pair(10.0f, 10.0f)},
        //抓取区位置，第二行
        {std::make_pair(10.0f, -10.0f), std::make_pair(10.0f, -5.0f), std::make_pair(5.0f, 5.0f), std::make_pair(10.0f, 10.0f)}
    }};

    //箱子ID数据
    box_info.box_ids = box_id_grid_;
    //VIP箱子ID
    box_info.vip_box_id = vip_box_id_;

    // 查找 vip 箱子在 box_ids 中的所有索引 (row, col)
    auto vip_indices = box_info.find_vip_indices();
    for (size_t i = 0; i < vip_indices.size(); ++i) {
        RCLCPP_INFO(context->node_->get_logger(), "VIP 箱子 %zu: row=%d, col=%d",
                    i, vip_indices[i].first, vip_indices[i].second);
    }

    //判断VIP箱子是否在第一排
    const bool vip_in_first_row = box_info.is_vip_in_first_row();
    //判断VIP箱子列的信息
    const char plan_case = detect_case(box_info.box_ids, box_info.vip_box_id);

    std::array<float, 3> a1, a2;

    // 根据进入路线设置机器人先到达的两个过渡点。
    if (plan_case == 'A') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 A");
        a1 = {0.0f, -15.0f, 0.0f}; //待改数据
        a2 = {0.0f, -15.0f, 0.0f}; //待改数据
    } else if (plan_case == 'C') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 C");
        a1 = {0.0f, 0.0f, 0.0f}; //待改数据
        a2 = {10.0f, 0.0f, 0.0f}; //待改数据
    } else if (plan_case == 'E') {
        RCLCPP_INFO(context->node_->get_logger(), "执行道路 E");
        a1 = {0.0f, 15.0f, 0.0f}; //待改数据
        a2 = {10.0f, 15.0f, 0.0f}; //待改数据
    }

    //此时走到第一排箱子区域
    //A：65 C：67 E：69

    // 记录两排抓取区中已经加入计划的箱子，避免重复规划。
    std::array<std::array<bool, 4>, 2> picked{};
    // 记录每种箱子 ID 已放置次数，第二次放置同 ID 时需要放到第二层。
    std::array<int, 4> placed_count{};
    // 上一次规划结束时的期望位置，用于选择最近的下一个箱子。
    std::array<float, 3> last_expected_pos = a2;

    //计移动箱子计划数容器
    std::vector<MoveBoxPlan> move_plan;
    //移动8个箱子，所以应该有8个计划
    move_plan.reserve(8);
    //狗到中间标志位
    bool dog_to_middle = false;

    // 封装单次抓取/放置计划的生成逻辑。
    auto append_plan = [&](int box_row, int col) {
        const int position_row = box_row + 1;
        const int box_id = box_info.box_ids[box_row][col];
        const std::array<float, 3> src = {
            box_info.positions[position_row][col].first,
            box_info.positions[position_row][col].second,
            -M_PI_2f};
        const std::array<float, 3> dst = {
            box_info.positions[0][box_id].first,
            box_info.positions[0][box_id].second,
            0.0f};

        MoveBoxPlan plan;
        if (!dog_to_middle) {
            plan.catch_trajectory.push_back(a1);
            plan.catch_trajectory.push_back(a2);
            dog_to_middle = true;
        }

        plan.catch_trajectory.push_back(src);
        plan.place_trajectory.push_back(dst);
        plan.src_box_pos = {src[0], src[1]};
        plan.dst_box_pos = {dst[0], dst[1]};
        plan.place_at_second_floor = placed_count[box_id] > 0;
        move_plan.push_back(plan);

        picked[box_row][col] = true;
        ++placed_count[box_id];
        last_expected_pos = dst;

        RCLCPP_INFO(
            context->node_->get_logger(),
            "生成抓取计划: row=%d, col=%d, box_id=%d, second_floor=%s",
            box_row,
            col,
            box_id,
            plan.place_at_second_floor ? "true" : "false");
    };

    auto pick_nearest_in_row = [&](int box_row, const std::function<bool(int)>& accept_box_id) {
        const auto nearest = find_nearest_box(last_expected_pos, box_info, box_row, picked, accept_box_id);
        if (nearest.col < 0) {
            return false;
        }

        append_plan(box_row, nearest.col);
        return true;
    };

    while (pick_nearest_in_row(0, [&](int box_id) { return box_id == box_info.vip_box_id; })) {
    }
    while (pick_nearest_in_row(0, [](int) { return true; })) {
    }
    while (pick_nearest_in_row(1, [&](int box_id) { return box_id == box_info.vip_box_id; })) {
    }
    while (pick_nearest_in_row(1, [](int) { return true; })) {
    }

    tree.write_msg("move_plan", move_plan);
    tree.write_msg<int>("plan_index", 0);

    RCLCPP_INFO(
        context->node_->get_logger(),
        "GeneratePlaneAction: case=%c, vip_box_id=%d, vip_in_first_row=%s, 已生成 %zu 条移动计划",
        plan_case,
        box_info.vip_box_id,
        vip_in_first_row ? "true" : "false",
        move_plan.size());

    generated=true;
    if (!context->is_tree_debug_mode()) {
        context->advance_tree_stage();
    }
    return BT::SUCCESS;
}
