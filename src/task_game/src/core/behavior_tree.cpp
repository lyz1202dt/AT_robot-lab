#include "core/behavior_tree.hpp"

#include <algorithm>

BT::Node::Node(std::string name)
    : name_(std::move(name)) {}

const std::string& BT::Node::name() const {
    return name_;
}

BT::Status BT::Node::last_status() const {
    return last_status_;
}

void BT::Node::reset() {
    // Nodes do not keep a running state. Resetting only clears the last result.
    last_status_ = FAILED;
}

void BT::Node::attach_parent(const std::shared_ptr<Node>& parent) {
    parent_ = parent;
}

bool BT::Node::attach_child(const std::shared_ptr<Node>& child) {
    const auto duplicated = std::any_of(children_.begin(), children_.end(), [&child](const std::weak_ptr<Node>& item) {
        const auto locked = item.lock();
        return locked && locked->name() == child->name();
    });
    if (duplicated) {
        return false;
    }

    children_.push_back(child);
    return true;
}

const std::vector<std::weak_ptr<BT::Node>>& BT::Node::children() const {
    return children_;
}

std::shared_ptr<BT::Node> BT::Node::parent() const {
    return parent_.lock();
}

BT::ControlNode::ControlNode(const std::string& name)
    : Node(name) {}

BT::SequenceNode::SequenceNode(const std::string& name)
    : ControlNode(name) {}

BT::Status BT::SequenceNode::tick(BT& tree) {
    if (children_.empty()) {
        last_status_ = SUCCESS;
        return last_status_;
    }

    // Sequence is strict: any failure stops evaluation immediately.
    for (const auto& child_weak : children_) {
        auto child = child_weak.lock();
        if (!child) {
            continue;
        }

        const Status child_status = child->tick(tree);
        if (child_status == FAILED) {
            last_status_ = FAILED;
            return last_status_;
        }

        // Actions ask the tree to restart from root on the next run.
        // Sequence propagates that request immediately and does not
        // continue to later siblings in the same run.
        if (tree.should_restart_from_root()) {
            last_status_ = child_status;
            return last_status_;
        }
    }

    last_status_ = SUCCESS;
    return last_status_;
}

void BT::SequenceNode::reset() {
    Node::reset();
}

BT::FallbackNode::FallbackNode(const std::string& name)
    : ControlNode(name) {}

BT::Status BT::FallbackNode::tick(BT& tree) {
    if (children_.empty()) {
        last_status_ = FAILED;
        return last_status_;
    }

    // Fallback is permissive: any FAILED child simply advances to the next
    // branch, regardless of whether that child is a condition, action, or
    // another control node.
    for (const auto& child_weak : children_) {
        auto child = child_weak.lock();
        if (!child) {
            continue;
        }

        const Status child_status = child->tick(tree);
        if (child_status == SUCCESS) {
            last_status_ = SUCCESS;
            return last_status_;
        }

        // A failed action child may have requested a root restart.
        // Fallback consumes that request because its contract is to keep
        // trying later branches until one succeeds or all fail.
        tree.clear_restart_from_root();
    }

    last_status_ = FAILED;
    return last_status_;
}

void BT::FallbackNode::reset() {
    Node::reset();
}

BT::LeafNode::LeafNode(const std::string& name)
    : Node(name) {}

BT::Status BT::LeafNode::tick(BT& tree) {
    // Leaf nodes delegate their result directly to execute().
    last_status_ = execute(tree);
    return last_status_;
}

BT::ConditionNode::ConditionNode(const std::string& name, ConditionFunc condition)
    : LeafNode(name),
      condition_(std::move(condition)) {}

BT::Status BT::ConditionNode::execute(BT& tree) {
    if (!condition_) {
        return FAILED;
    }

    return condition_(tree) ? SUCCESS : FAILED;
}

BT::ActionNode::ActionNode(const std::string& name)
    : LeafNode(name) {}

BT::Status BT::ActionNode::tick(BT& tree) {
    last_status_ = execute(tree);
    // Actions always end the current traversal so the next BT::run()
    // reevaluates the tree from the root.
    //tree.request_restart_from_root();
    return last_status_;
}

bool BT::rgister(
    const std::shared_ptr<Node>& node,
    const std::string& parent_name,
    const std::vector<std::string>& child_names) {
    if (!node || node->name().empty()) {
        return false;
    }

    if (nodes_.find(node->name()) != nodes_.end()) {
        return false;
    }

    nodes_.emplace(node->name(), node);

    if (!parent_name.empty()) {
        const auto parent_it = nodes_.find(parent_name);
        if (parent_it != nodes_.end()) {
            if (!attach_parent_child(parent_it->second, node)) {
                return false;
            }
        } else {
            pending_parent_links_[node->name()] = parent_name;
        }
    }

    for (const auto& child_name : child_names) {
        if (child_name.empty() || child_name == node->name()) {
            return false;
        }

        const auto child_it = nodes_.find(child_name);
        if (child_it != nodes_.end()) {
            if (!attach_parent_child(node, child_it->second)) {
                return false;
            }
        } else {
            pending_child_links_[node->name()].push_back(child_name);
        }
    }

    resolve_pending_links_for(node->name());
    return true;
}

bool BT::set_root(const std::string& root_name) {
    if (nodes_.find(root_name) == nodes_.end()) {
        return false;
    }

    root_name_ = root_name;
    return true;
}

BT::Status BT::run() {
    const auto root = resolve_root();
    if (!root) {
        return FAILED;
    }

    // Each traversal starts with a clean restart flag.
    restart_from_root_ = false;
    return root->tick(*this);
}

void BT::reset() {
    const auto root = resolve_root();
    if (!root) {
        return;
    }

    // Reset also clears any pending restart request from the previous run.
    restart_from_root_ = false;
    reset_subtree(root);
}

std::shared_ptr<BT::Node> BT::get_node(const std::string& name) const {
    const auto it = nodes_.find(name);
    if (it == nodes_.end()) {
        return nullptr;
    }
    return it->second;
}

const char* BT::to_string(Status status) {
    switch (status) {
    case SUCCESS:
        return "SUCCESS";
    case FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}

bool BT::attach_parent_child(const std::shared_ptr<Node>& parent, const std::shared_ptr<Node>& child) {
    if (!parent || !child || parent->name() == child->name()) {
        return false;
    }

    const auto old_parent = child->parent();
    if (old_parent && old_parent->name() != parent->name()) {
        return false;
    }

    child->attach_parent(parent);
    return parent->attach_child(child);
}

void BT::resolve_pending_links_for(const std::string& node_name) {
    const auto current_it = nodes_.find(node_name);
    if (current_it == nodes_.end()) {
        return;
    }

    auto parent_link_it = pending_parent_links_.find(node_name);
    if (parent_link_it != pending_parent_links_.end()) {
        const auto parent_it = nodes_.find(parent_link_it->second);
        if (parent_it != nodes_.end() && attach_parent_child(parent_it->second, current_it->second)) {
            pending_parent_links_.erase(parent_link_it);
        }
    }

    auto child_link_it = pending_child_links_.find(node_name);
    if (child_link_it != pending_child_links_.end()) {
        auto& child_names = child_link_it->second;
        child_names.erase(
            std::remove_if(child_names.begin(), child_names.end(), [this, &current_it](const std::string& child_name) {
                const auto child_it = nodes_.find(child_name);
                return child_it != nodes_.end() && attach_parent_child(current_it->second, child_it->second);
            }),
            child_names.end());

        if (child_names.empty()) {
            pending_child_links_.erase(child_link_it);
        }
    }

    for (auto it = pending_child_links_.begin(); it != pending_child_links_.end();) {
        const auto parent_it = nodes_.find(it->first);
        if (parent_it == nodes_.end()) {
            ++it;
            continue;
        }

        auto& child_names = it->second;
        child_names.erase(
            std::remove_if(child_names.begin(), child_names.end(), [this, &parent_it, &node_name](const std::string& child_name) {
                if (child_name != node_name) {
                    return false;
                }

                const auto child_it = nodes_.find(child_name);
                return child_it != nodes_.end() && attach_parent_child(parent_it->second, child_it->second);
            }),
            child_names.end());

        if (child_names.empty()) {
            it = pending_child_links_.erase(it);
        } else {
            ++it;
        }
    }
}

std::shared_ptr<BT::Node> BT::resolve_root() const {
    if (!root_name_.empty()) {
        return get_node(root_name_);
    }

    for (const auto& item : nodes_) {
        if (!item.second->parent()) {
            return item.second;
        }
    }

    return nullptr;
}

void BT::reset_subtree(const std::shared_ptr<Node>& node) {
    if (!node) {
        return;
    }

    node->reset();
    for (const auto& child : node->children()) {
        reset_subtree(child.lock());
    }
}

bool BT::should_restart_from_root() const {
    return restart_from_root_;
}

void BT::request_restart_from_root() {
    restart_from_root_ = true;
}

void BT::clear_restart_from_root() {
    restart_from_root_ = false;
}
