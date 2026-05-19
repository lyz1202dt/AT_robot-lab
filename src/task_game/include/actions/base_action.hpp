#pragma once

#include <string>
#include <queue>
#include <any>
#include <utility>

// 前向声明模板类
template<typename Context>
class Action;

// 状态基类
// Context: 状态需要访问的上下文类型（可以是包含FSM的类、其他组件等）
template<typename Context>
class BaseAction{
    public:
    explicit BaseAction(const std::string &name) : name(name) {}
    virtual ~BaseAction() = default;

    // 子类需要实现的动作接口
    virtual std::string process(Context* context, const std::string &last_status) = 0;

    bool push_msg(const std::any &msg) {
        msg_queue.push(msg);
        return true;
    }

    bool push_msg(std::any &&msg) {
        msg_queue.push(std::move(msg));
        return true;
    }

    template<typename MsgType>
    bool push_msg(MsgType &&msg) {
        msg_queue.emplace(std::forward<MsgType>(msg));
        return true;
    }

    bool pop_msg(std::any &msg) {
        if (msg_queue.empty()) {
            return false;
        }

        msg = std::move(msg_queue.front());
        msg_queue.pop();
        return true;
    }

    template<typename MsgType>
    bool pop_msg(MsgType &msg) {
        if (msg_queue.empty()) {
            return false;
        }

        try {
            msg = std::any_cast<MsgType>(std::move(msg_queue.front()));
        } catch (const std::bad_any_cast &) {
            return false;
        }

        msg_queue.pop();
        return true;
    }
    
    std::string name;       //建议状态名不要超过15个字节，超过SSO容量后会触发string的堆内存分配，影响效率
    inline static std::queue<std::any> msg_queue;  //静态消息队列，用于状态间通信，全局唯一
};
