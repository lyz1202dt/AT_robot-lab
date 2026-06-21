#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <robot_msgs/msg/cmd.hpp>

class Pilot;

class AutoTaskPauseController {
public:
    enum class PauseReason : std::uint8_t {
        None = 0,
        Remote = 1 << 0,
        TfFault = 1 << 1,
    };

    explicit AutoTaskPauseController(std::shared_ptr<Pilot> pilot);

    void set_manual_zero_command(robot_msgs::msg::Cmd* cmd);

    void pause_for_remote();
    void resume_for_remote(bool allow_resume);
    void pause_for_tf_fault();
    void clear_tf_fault(bool allow_resume);

    bool is_paused() const;
    bool should_run_bt() const;
    bool should_request_pilot_resume() const;
    void mark_pilot_resume_requested_done();
    bool has_tf_fault() const;
    std::uint8_t pause_mask() const;
    std::string describe_pause_state() const;

private:
    void pause(PauseReason reason);
    void resume(PauseReason reason, bool allow_resume);
    static std::uint8_t to_mask(PauseReason reason);
    void apply_stand_command();

    std::shared_ptr<Pilot> pilot_;
    robot_msgs::msg::Cmd* cmd_{nullptr};
    std::uint8_t pause_mask_{0};
    bool should_request_pilot_resume_{false};
};
