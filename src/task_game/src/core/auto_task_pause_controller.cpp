#include "core/auto_task_pause_controller.hpp"

#include <sstream>
#include <utility>

#include "core/pilot.hpp"

namespace {

constexpr std::uint8_t kPauseReasonRemote = static_cast<std::uint8_t>(AutoTaskPauseController::PauseReason::Remote);
constexpr std::uint8_t kPauseReasonTfFault = static_cast<std::uint8_t>(AutoTaskPauseController::PauseReason::TfFault);

}  // namespace

AutoTaskPauseController::AutoTaskPauseController(std::shared_ptr<Pilot> pilot)
    : pilot_(std::move(pilot)) {}

void AutoTaskPauseController::set_manual_zero_command(robot_msgs::msg::Cmd* cmd) {
    cmd_ = cmd;
}

void AutoTaskPauseController::pause_for_remote() {
    pause(PauseReason::Remote);
}

void AutoTaskPauseController::resume_for_remote(bool allow_resume) {
    resume(PauseReason::Remote, allow_resume);
}

void AutoTaskPauseController::pause_for_tf_fault() {
    pause(PauseReason::TfFault);
}

void AutoTaskPauseController::clear_tf_fault(bool allow_resume) {
    resume(PauseReason::TfFault, allow_resume);
}

bool AutoTaskPauseController::is_paused() const {
    return pause_mask_ != 0;
}

bool AutoTaskPauseController::should_run_bt() const {
    return pause_mask_ == 0;
}

bool AutoTaskPauseController::should_request_pilot_resume() const {
    return should_request_pilot_resume_;
}

void AutoTaskPauseController::mark_pilot_resume_requested_done() {
    should_request_pilot_resume_ = false;
}

bool AutoTaskPauseController::has_tf_fault() const {
    return (pause_mask_ & kPauseReasonTfFault) != 0;
}

std::uint8_t AutoTaskPauseController::pause_mask() const {
    return pause_mask_;
}

std::string AutoTaskPauseController::describe_pause_state() const {
    if (pause_mask_ == 0) {
        return "running";
    }

    std::ostringstream oss;
    bool first = true;
    if ((pause_mask_ & kPauseReasonRemote) != 0) {
        oss << "remote";
        first = false;
    }
    if ((pause_mask_ & kPauseReasonTfFault) != 0) {
        if (!first) {
            oss << "+";
        }
        oss << "tf_fault";
    }
    return oss.str();
}

void AutoTaskPauseController::pause(PauseReason reason) {
    const std::uint8_t reason_mask = to_mask(reason);
    if ((pause_mask_ & reason_mask) != 0) {
        apply_stand_command();
        return;
    }

    pause_mask_ |= reason_mask;
    should_request_pilot_resume_ = false;
    if (pilot_) {
        pilot_->stop();
    }
    apply_stand_command();
}

void AutoTaskPauseController::resume(PauseReason reason, bool allow_resume) {
    const std::uint8_t reason_mask = to_mask(reason);
    if ((pause_mask_ & reason_mask) == 0) {
        return;
    }

    pause_mask_ &= static_cast<std::uint8_t>(~reason_mask);
    should_request_pilot_resume_ = false;
    if (pause_mask_ != 0) {
        apply_stand_command();
        return;
    }

    if (!allow_resume) {
        apply_stand_command();
    }
}

std::uint8_t AutoTaskPauseController::to_mask(PauseReason reason) {
    return static_cast<std::uint8_t>(reason);
}

void AutoTaskPauseController::apply_stand_command() {
    if (!cmd_) {
        return;
    }

    cmd_->mode = 1;
    cmd_->vx = 0.0f;
    cmd_->vy = 0.0f;
    cmd_->vz = 0.0f;
    cmd_->wheel_vel = 0.0f;
}
