#pragma once

#include <string>

// 前向声明模板类
template<typename Context>
class FSM;

// 状态基类
// Context: 状态需要访问的上下文类型（可以是包含FSM的类、其他组件等）
template<typename Context>
class BaseState{
    public:
    explicit BaseState(const std::string &name) : name(name) {}
    virtual ~BaseState() = default;

    // 子类需要实现的类型安全接口
    virtual bool enter(Context* context, const std::string &last_status) = 0;
    virtual std::string update(Context* context) = 0;
    
    std::string name;       //建议状态名不要超过15个字节，超过SSO容量后会触发string的堆内存分配，影响效率
};