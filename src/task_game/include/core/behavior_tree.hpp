#pragma once

#include <any>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

class BT {
public:
    // This behavior tree only exposes terminal results to callers.
    // Intermediate progress is modeled by ending the current run and
    // letting the next BT::run() restart from the root.
    enum Status {
        SUCCESS = 0,
        FAILED
    };

    class Node {
    public:
        explicit Node(std::string name);
        virtual ~Node() = default;

        const std::string& name() const;
        Status last_status() const;

        virtual Status tick(BT& tree) = 0;
        virtual void reset();

    protected:
        friend class BT;

        void attach_parent(const std::shared_ptr<Node>& parent);
        bool attach_child(const std::shared_ptr<Node>& child);
        const std::vector<std::weak_ptr<Node>>& children() const;
        std::shared_ptr<Node> parent() const;

        std::string name_;
        Status last_status_{FAILED};
        std::weak_ptr<Node> parent_;
        std::vector<std::weak_ptr<Node>> children_;
    };

    class ControlNode : public Node {
    public:
        explicit ControlNode(const std::string& name);
        ~ControlNode() override = default;
    };

    class SequenceNode : public ControlNode {
    public:
        explicit SequenceNode(const std::string& name);
        // Runs children from left to right.
        // Any FAILED child fails the whole sequence.
        // If a child requests a root restart, the current run stops here.
        Status tick(BT& tree) override;
        void reset() override;
    };

    class FallbackNode : public ControlNode {
    public:
        explicit FallbackNode(const std::string& name);
        // Runs children from left to right until one succeeds.
        // Any FAILED child causes the fallback to try the next branch.
        // This applies to all child node types, including action nodes.
        Status tick(BT& tree) override;
        void reset() override;
    };

    class LeafNode : public Node {
    public:
        explicit LeafNode(const std::string& name);
        Status tick(BT& tree) override;

    protected:
        virtual Status execute(BT& tree) = 0;
    };

    class ConditionNode : public LeafNode {
    public:
        using ConditionFunc = std::function<bool(BT&)>;

        // Condition nodes are configured with a predicate at construction time.
        // Returning true maps to SUCCESS, false maps to FAILED.
        ConditionNode(const std::string& name, ConditionFunc condition);
        ~ConditionNode() override = default;

    protected:
        Status execute(BT& tree) override;

    private:
        ConditionFunc condition_;
    };

    class ActionNode : public LeafNode {
    public:
        explicit ActionNode(const std::string& name);
        // Action nodes always request the next BT::run() to restart from root
        // after execute() returns, regardless of SUCCESS or FAILED.
        Status tick(BT& tree) override;
        ~ActionNode() override = default;
    };

    BT() = default;
    ~BT() = default;

    template <typename Context>
    void set_context(Context* context) {
        context_ = context;
    }

    template <typename Context>
    Context* get_context() const {
        return static_cast<Context*>(context_);
    }

    bool rgister(
        const std::shared_ptr<Node>& node,
        const std::string& parent_name = "",
        const std::vector<std::string>& child_names = {});

    bool set_root(const std::string& root_name);
    Status run();
    void reset();

    template <typename MsgType>
    bool write_msg(const std::string& name, MsgType&& msg) {
        msg_blackboard_[name] = std::forward<MsgType>(msg);
        return true;
    }

    template <typename MsgType>
    bool read_msg(const std::string& name, MsgType& msg) const {
        const auto it = msg_blackboard_.find(name);
        if (it == msg_blackboard_.end()) {
            return false;
        }

        try {
            msg = std::any_cast<MsgType>(it->second);
        } catch (const std::bad_any_cast&) {
            return false;
        }
        return true;
    }

    std::shared_ptr<Node> get_node(const std::string& name) const;
    static const char* to_string(Status status);
    void reset_subtree(const std::shared_ptr<Node>& node);
    // True when a child action has finished and asked the tree to stop the
    // current run so the next run can restart from the root.
    bool should_restart_from_root() const;
    void request_restart_from_root();

private:
    bool attach_parent_child(const std::shared_ptr<Node>& parent, const std::shared_ptr<Node>& child);
    void resolve_pending_links_for(const std::string& node_name);
    std::shared_ptr<Node> resolve_root() const;
    void clear_restart_from_root();

    std::unordered_map<std::string, std::shared_ptr<Node>> nodes_;
    std::unordered_map<std::string, std::string> pending_parent_links_;
    std::unordered_map<std::string, std::vector<std::string>> pending_child_links_;
    std::unordered_map<std::string, std::any> msg_blackboard_;
    std::string root_name_;
    void* context_{nullptr};
    bool restart_from_root_{false};
};
