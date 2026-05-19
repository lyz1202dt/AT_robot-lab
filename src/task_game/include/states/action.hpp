#pragma once


#include "states/base_state.hpp"
#include <memory>
#include <rclcpp/logger.hpp>
#include <unordered_map>
#include <string>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// 模板化FSM，Context是状态需要访问的上下文类型
template<typename Context>
class FSM{
    public:
    // 构造函数，接收上下文指针和初始状态名
    explicit FSM(Context* context, const std::string &init_state)
        : context_(context), first_run(true), last_state(init_state) {}
    
    ~FSM() = default;

    void run() {
        // 如果没有状态，直接返回
        if (states.empty()) {
            return;
        }

        // 如果是第一次运行，初始化到第一个状态
        if (first_run) {
            if(states.find(last_state) != states.end()) {
                states[last_state]->enter(context_, "");
            }
            first_run = false;
        }

        // 调用当前状态的update获取下一个状态名
        std::string next_state = states[last_state]->update(context_);

        // 如果返回的状态名与当前状态不同，则切换状态
        if (next_state != last_state) {
            // 检查下一个状态是否存在
            if (states.find(next_state) != states.end()) {
                std::string prev_state = last_state;
                last_state = next_state;
                // 调用新状态的enter方法
                states[last_state]->enter(context_, prev_state);
            } else {
                // 状态跳转失败，调用处理方法
                on_state_transition_failed(last_state, next_state);
            }
        }
    }

    bool register_state(std::unique_ptr<BaseState<Context>> state) {
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

    bool unregister_state(const std::string &name) {
        // 查找并删除状态
        auto it = states.find(name);
        if (it != states.end()) {
            states.erase(it);
            return true;
        }
        return false;
    }

    protected:
    virtual void on_state_transition_failed(const std::string &current_state, const std::string &target_state) {
        RCLCPP_INFO(rclcpp::get_logger("FSM"),"State transition failed! Current state: %s, Target state: %s",current_state.c_str(),target_state.c_str());
    }

    public:
    Context* context_;      // 上下文指针
    bool first_run;
    std::string last_state;
    std::unordered_map<std::string, std::unique_ptr<BaseState<Context>>> states;
};