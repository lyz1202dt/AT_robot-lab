#pragma once


#include "actions/base_action.hpp"
#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>

// 模板化Action，Context是状态需要访问的上下文类型
template <typename Context>
class Action {
public:
    // 构造函数，接收上下文指针和初始状态名
    explicit Action(Context* context, const std::string& init_state)
        : context_(context)
        , next_state(init_state) {}

    ~Action() = default;

    void run() {
        // 如果没有状态，直接返回
        if (states.empty()) {
            return;
        }

        const auto current_it = states.find(next_state);
        if (current_it == states.end()) {
            on_state_transition_failed(last_state, next_state);
            return;
        }

        last_state = next_state;
        next_state = current_it->second->process(context_, last_state);  //执行动作

        if (states.find(next_state) == states.end()) {      //动作合法性检验
            on_state_transition_failed(last_state, next_state);
        }
    }

    bool register_action(std::unique_ptr<BaseAction<Context>> state) {
        if (!state) {
            return false;
        }

        std::string state_name = state->name;

        // 检查状态是否已存在
        if (states.find(state_name) != states.end()) {
            return false;
        }

        // 使用move语义将unique_ptr移动到map中
        states[state_name] = std::move(state);
        return true;
    }

    bool unregister_action(const std::string& name) {
        // 查找并删除状态
        auto it = states.find(name);
        if (it != states.end()) {
            states.erase(it);
            return true;
        }
        return false;
    }

protected:
    virtual void on_state_transition_failed(const std::string& current_state, const std::string& target_state) {
        RCLCPP_INFO(
            rclcpp::get_logger("Action"), "State transition failed! Current state: %s, Target state: %s", current_state.c_str(),
            target_state.c_str());
    }

public:
    Context* context_; // 上下文指针
    std::string last_state;
    std::string next_state;
    std::unordered_map<std::string, std::unique_ptr<BaseAction<Context>>> states;
};
