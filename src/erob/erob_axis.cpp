#include "erob/erob_axis.h"

#include <cmath>

namespace erob {

namespace {

constexpr double kFollowInputMinAngleDeg = -130.0;
constexpr double kFollowInputMaxAngleDeg = 130.0;

}  // namespace

ErobAxis::ErobAxis(const AxisConfig& config)
    : config_(config),
      state_buffer_(AxisState{}),
    command_buffer_(AxisCommand{}) {}

bool ErobAxis::bindMotor(const MotorIdentity& motor) {
    config_.bound_motor = motor;
    return config_.bound_motor.slave_index != 0;
}

const AxisConfig& ErobAxis::config() const {
    return config_;
}

AxisState ErobAxis::getState() const {
    return state_buffer_.load();
}

std::string ErobAxis::lastError() const {
    return last_error_;
}

bool ErobAxis::enable() {
    quick_stop_latched_.store(false, std::memory_order_release);
    AxisCommand command = command_buffer_.load();
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.reset_fault_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    return publishCommand(command);
}

bool ErobAxis::disable() {
    quick_stop_latched_.store(false, std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    planner_velocity_deg_s_ = 0.0;
    interp_start_velocity_deg_s_ = 0.0;
    interp_target_velocity_deg_s_ = 0.0;
    interpolated_velocity_deg_s_ = 0.0;
    interp_step_ = 0;
    AxisCommand command = command_buffer_.load();
    command.disable_requested = true;
    command.enable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    setPositionModeState(PositionModeState::kIdle);
    setFollowModeState(FollowModeState::kIdle);
    return publishCommand(command);
}

bool ErobAxis::resetFault() {
    quick_stop_latched_.store(false, std::memory_order_release);
    AxisCommand command = command_buffer_.load();
    command.reset_fault_requested = true;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    return publishCommand(command);
}

bool ErobAxis::quickStop() {
    quick_stop_latched_.store(true, std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    planner_velocity_deg_s_ = 0.0;
    interp_start_velocity_deg_s_ = 0.0;
    interp_target_velocity_deg_s_ = 0.0;
    interpolated_velocity_deg_s_ = 0.0;
    interp_step_ = 0;

    AxisCommand command = command_buffer_.load();
    command.quick_stop_requested = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.reset_fault_requested = false;
    command.target_velocity_deg_s = 0.0;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    setPositionModeState(PositionModeState::kFault);
    setFollowModeState(FollowModeState::kFault);
    return publishCommand(command);
}

bool ErobAxis::setProfilePositionTarget(
    double angle_deg,
    double velocity_deg_s,
    ProfilePositionParams* params,
    uint64_t* request_id) {
    if (params == nullptr) {
        setLastError("profile position params output is null");
        return false;
    }
    if (velocity_deg_s <= 0.0) {
        setLastError("profile position velocity must be positive");
        return false;
    }

    quick_stop_latched_.store(false, std::memory_order_release);
    const uint64_t new_request_id = next_profile_request_id_.fetch_add(1, std::memory_order_relaxed);
    profile_request_id_.store(new_request_id, std::memory_order_release);
    const double clamped_angle = clampAngle(angle_deg);
    const double limited_velocity = Clamp(velocity_deg_s, 0.0, config_.max_velocity_deg_s);

    params->profile_velocity = static_cast<uint32_t>(std::llround(std::fabs(velocityToCount(limited_velocity))));
    params->profile_acceleration = static_cast<uint32_t>(std::llround(std::fabs(config_.max_accel_deg_s2 * config_.counts_per_degree)));
    params->profile_deceleration = static_cast<uint32_t>(std::llround(std::fabs(config_.max_decel_deg_s2 * config_.counts_per_degree)));

    AxisCommand command = command_buffer_.load();
    AxisState state = state_buffer_.load();
    const bool switching_from_follow =
        follow_active_.load(std::memory_order_acquire) ||
        command.requested_mode == MotionMode::kCyclicSyncVelocity ||
        state.motion_mode == MotionMode::kCyclicSyncVelocity ||
        state.follow_mode_state != FollowModeState::kIdle;

    follow_active_.store(false, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    planner_velocity_deg_s_ = 0.0;
    interp_start_velocity_deg_s_ = 0.0;
    interp_target_velocity_deg_s_ = 0.0;
    interpolated_velocity_deg_s_ = 0.0;
    interp_step_ = 0;

    command.requested_mode = MotionMode::kProfilePosition;
    command.target_angle_deg = clamped_angle;
    command.target_velocity_deg_s = limited_velocity;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    profile_transition_pending_ = false;
    pending_profile_target_deg_ = clamped_angle;
    pending_profile_velocity_deg_s_ = limited_velocity;
    state.target_angle_deg = clamped_angle;
    state.position_error_deg = clamped_angle - state.actual_angle_deg;
    state.target_reached = false;
    state.position_mode_state = state.enabled
        ? PositionModeState::kSendingSetpoint
        : PositionModeState::kWaitingEnable;
    state.follow_mode_state = FollowModeState::kIdle;
    state_buffer_.publish(state);
    if (request_id != nullptr) {
        *request_id = new_request_id;
    }
    return publishCommand(command);
}

bool ErobAxis::enterFollowMode(uint64_t* request_id) {
    quick_stop_latched_.store(false, std::memory_order_release);
    profile_transition_pending_ = false;
    profile_request_id_.store(
        next_profile_request_id_.fetch_add(1, std::memory_order_relaxed),
        std::memory_order_release);
    const AxisState state = getState();

    const AxisCommand current_command = command_buffer_.load();
    const bool already_requested =
        follow_active_.load(std::memory_order_acquire) &&
        current_command.requested_mode == MotionMode::kCyclicSyncVelocity &&
        state.follow_mode_state != FollowModeState::kStopping &&
        state.follow_mode_state != FollowModeState::kFault;
    if (already_requested) {
        if (request_id != nullptr) {
            *request_id = follow_request_id_.load(std::memory_order_acquire);
        }
        return true;
    }

    const uint64_t new_request_id = next_follow_request_id_.fetch_add(1, std::memory_order_relaxed);
    follow_request_id_.store(new_request_id, std::memory_order_release);
    follow_active_.store(true, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    last_follow_target_ns_.store(SteadyClockNowNs(), std::memory_order_release);
    follow_target_deg_.store(state.actual_angle_deg, std::memory_order_release);
    planner_filtered_target_deg_ = state.actual_angle_deg;
    planner_velocity_deg_s_ = 0.0;

    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kCyclicSyncVelocity;
    command.target_velocity_deg_s = 0.0;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    setPositionModeState(PositionModeState::kIdle);
    setFollowModeState(state.enabled
        ? FollowModeState::kEnteringCsvMode
        : FollowModeState::kWaitingEnable);
    if (request_id != nullptr) {
        *request_id = new_request_id;
    }
    return publishCommand(command);
}

bool ErobAxis::updateFollowTarget(double angle_deg) {
    if (!follow_active_.load(std::memory_order_acquire)) {
        setLastError("follow mode is not active");
        return false;
    }
    if (angle_deg < kFollowInputMinAngleDeg || angle_deg > kFollowInputMaxAngleDeg) {
        return true;
    }
    follow_target_deg_.store(clampAngle(angle_deg), std::memory_order_release);
    last_follow_target_ns_.store(SteadyClockNowNs(), std::memory_order_release);
    return true;
}

bool ErobAxis::stopFollowMode(uint64_t* request_id) {
    const AxisState state = getState();
    const AxisCommand current_command = command_buffer_.load();
    const bool already_stopped =
        !follow_active_.load(std::memory_order_acquire) &&
        state.follow_mode_state == FollowModeState::kIdle &&
        current_command.target_velocity_deg_s == 0.0;
    if (already_stopped) {
        if (request_id != nullptr) {
            *request_id = follow_request_id_.load(std::memory_order_acquire);
        }
        return true;
    }

    const uint64_t new_request_id = next_follow_request_id_.fetch_add(1, std::memory_order_relaxed);
    follow_request_id_.store(new_request_id, std::memory_order_release);
    profile_request_id_.store(
        next_profile_request_id_.fetch_add(1, std::memory_order_relaxed),
        std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    planner_velocity_deg_s_ = 0.0;
    interp_start_velocity_deg_s_ = interpolated_velocity_deg_s_;
    interp_target_velocity_deg_s_ = 0.0;
    interp_step_ = 0;

    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kCyclicSyncVelocity;
    command.target_velocity_deg_s = 0.0;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    setFollowModeState(FollowModeState::kStopping);
    if (request_id != nullptr) {
        *request_id = new_request_id;
    }
    return publishCommand(command);
}

void ErobAxis::planFollowStep() {
    const bool follow_active = follow_active_.load(std::memory_order_acquire);
    if (!follow_active) {
        return;
    }

    const AxisState state = state_buffer_.load();
    if (state.fault || state.cia402_state == CiA402State::kQuickStopActive) {
        setFollowModeState(FollowModeState::kFault);
        planner_velocity_deg_s_ = 0.0;
        return;
    }
    if (!state.online || !state.enabled) {
        planner_velocity_deg_s_ = 0.0;

        AxisCommand command = command_buffer_.load();
        command.requested_mode = MotionMode::kCyclicSyncVelocity;
        command.target_velocity_deg_s = 0.0;
        command.command_updated = true;
        command.enable_requested = true;
        command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
        setFollowModeState(FollowModeState::kWaitingEnable);
        publishCommand(command);
        return;
    }

    const double dt = 1.0 / std::max(1.0, config_.follow.control_rate_hz);

    double desired_velocity = 0.0;
    const double raw_target = follow_target_deg_.load(std::memory_order_acquire);
    const double clamped_target = clampAngle(raw_target);
    const double limit_hold_band_deg = std::max(2.0, config_.follow.position_limit_margin_deg * 0.4);
    const double limit_settle_deg = 1.0;
    const double limit_capture_velocity_deg_s =
        std::max(2.0, config_.follow.max_velocity_deg_s * 0.03);
    const double limit_soft_target_offset_deg = limit_hold_band_deg;
    const double limit_release_band_deg = limit_hold_band_deg + 1.0;
    const bool target_at_positive_limit = clamped_target >= (config_.max_angle_deg - 1e-6);
    const bool target_at_negative_limit = clamped_target <= (config_.min_angle_deg + 1e-6);
    const bool target_near_positive_limit = clamped_target >= (config_.max_angle_deg - limit_hold_band_deg);
    const bool target_near_negative_limit = clamped_target <= (config_.min_angle_deg + limit_hold_band_deg);
    double effective_target_deg = clamped_target;
    if (target_at_positive_limit) {
        effective_target_deg = config_.max_angle_deg - limit_soft_target_offset_deg;
    } else if (target_at_negative_limit) {
        effective_target_deg = config_.min_angle_deg + limit_soft_target_offset_deg;
    }
    planner_filtered_target_deg_ =
        config_.follow.target_filter_alpha * effective_target_deg +
        (1.0 - config_.follow.target_filter_alpha) * planner_filtered_target_deg_;


    if (follow_positive_limit_hold_ && clamped_target < (config_.max_angle_deg - limit_release_band_deg)) {
        follow_positive_limit_hold_ = false;
    }
    if (follow_negative_limit_hold_ && clamped_target > (config_.min_angle_deg + limit_release_band_deg)) {
        follow_negative_limit_hold_ = false;
    }

    if (!(follow_positive_limit_hold_ || follow_negative_limit_hold_)) {
        const double error = planner_filtered_target_deg_ - state.actual_angle_deg;
        if (std::fabs(error) > config_.follow.deadband_deg) {
            desired_velocity = config_.follow.kp * error - config_.follow.kd * state.actual_velocity_deg_s;
        }
    }

    desired_velocity = Clamp(
        desired_velocity,
        -config_.follow.max_velocity_deg_s,
        config_.follow.max_velocity_deg_s);

    if (!(follow_positive_limit_hold_ || follow_negative_limit_hold_)) {
        const double max_accel = desired_velocity >= planner_velocity_deg_s_
            ? config_.follow.max_accel_deg_s2
            : config_.follow.max_decel_deg_s2;
        const double accel_step = max_accel * dt;
        if (desired_velocity > planner_velocity_deg_s_) {
            desired_velocity = std::min(desired_velocity, planner_velocity_deg_s_ + accel_step);
        } else {
            desired_velocity = std::max(desired_velocity, planner_velocity_deg_s_ - accel_step);
        }
    }

    if (!(follow_positive_limit_hold_ || follow_negative_limit_hold_)) {
        const double pos_error = std::fabs(effective_target_deg - state.actual_angle_deg);
        const double limited_velocity_for_hold = applyVelocityLimiter(state, desired_velocity);
        const bool can_capture_positive_limit =
            target_near_positive_limit &&
            state.actual_angle_deg >= (config_.max_angle_deg - limit_hold_band_deg) &&
            (pos_error < limit_settle_deg ||
             std::fabs(limited_velocity_for_hold) <= limit_capture_velocity_deg_s);
        const bool can_capture_negative_limit =
            target_near_negative_limit &&
            state.actual_angle_deg <= (config_.min_angle_deg + limit_hold_band_deg) &&
            (pos_error < limit_settle_deg ||
             std::fabs(limited_velocity_for_hold) <= limit_capture_velocity_deg_s);

        if (can_capture_positive_limit) {
            follow_positive_limit_hold_ = true;
        } else if (can_capture_negative_limit) {
            follow_negative_limit_hold_ = true;
        }
    }

    const bool holding_limit = follow_positive_limit_hold_ || follow_negative_limit_hold_;
    if (holding_limit) {
        planner_filtered_target_deg_ = Clamp(
            state.actual_angle_deg,
            config_.min_angle_deg,
            config_.max_angle_deg);
        desired_velocity = 0.0;
    }

    planner_velocity_deg_s_ = desired_velocity;

    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kCyclicSyncVelocity;
    command.target_velocity_deg_s = desired_velocity;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    if (state.motion_mode != MotionMode::kCyclicSyncVelocity) {
        setFollowModeState(FollowModeState::kEnteringCsvMode);
    } else {
        setFollowModeState(FollowModeState::kFollowing);
    }
    publishCommand(command);
}

void ErobAxis::updateFeedback(const TxPdoCommon& txpdo, int al_status_code) {
    AxisState state = state_buffer_.load();
    const AxisCommand command = command_buffer_.load();
    state.statusword = txpdo.statusword;
    state.actual_position_count = txpdo.actual_position;
    state.actual_velocity_count_s = txpdo.actual_velocity;
    state.actual_torque = txpdo.actual_torque;
    state.motion_mode = Cia402ValueToMotionMode(txpdo.mode_of_operation_display);
    state.cia402_state = ParseCiA402State(txpdo.statusword);
    state.enabled = IsCia402Enabled(state.cia402_state);
    state.fault = state.cia402_state == CiA402State::kFault ||
        state.cia402_state == CiA402State::kFaultReactionActive;
    state.actual_angle_deg = countToAngle(txpdo.actual_position);
    state.actual_velocity_deg_s = countToVelocity(txpdo.actual_velocity);
    state.target_angle_deg = follow_active_.load(std::memory_order_acquire)
        ? follow_target_deg_.load(std::memory_order_acquire)
        : command.target_angle_deg;
    if (command.requested_mode == MotionMode::kProfilePosition) {
        const double abs_counts_per_degree = std::fabs(config_.counts_per_degree);
        if (abs_counts_per_degree > 1e-9) {
            state.position_error_deg =
                static_cast<double>(target_position_count_ - txpdo.actual_position) / config_.counts_per_degree;
        } else {
            state.position_error_deg = 0.0;
        }
        const double position_tolerance_deg = abs_counts_per_degree > 1e-9
            ? std::max(0.05, 20.0 / abs_counts_per_degree)
            : 0.05;
        const double velocity_tolerance_deg_s = std::max(0.5, config_.max_velocity_deg_s * 0.02);
        const bool within_position_tolerance =
            std::fabs(state.position_error_deg) <= position_tolerance_deg;
        const bool within_velocity_tolerance =
            std::fabs(state.actual_velocity_deg_s) <= velocity_tolerance_deg_s;
        const bool reached_bit = (txpdo.statusword & 0x0400U) != 0;
        const bool settled_at_target =
            pp_pulse_cycles_remaining_ == 0 && within_position_tolerance && within_velocity_tolerance;
        state.target_reached = (reached_bit && within_position_tolerance) || settled_at_target;
    } else {
        state.position_error_deg = state.target_angle_deg - state.actual_angle_deg;
        state.target_reached = false;
    }
    state.near_positive_limit = state.actual_angle_deg >=
        (config_.max_angle_deg - config_.follow.position_limit_margin_deg);
    state.near_negative_limit = state.actual_angle_deg <=
        (config_.min_angle_deg + config_.follow.position_limit_margin_deg);
    state.online = true;
    state.al_status_code = al_status_code;

    if (state.fault || state.cia402_state == CiA402State::kQuickStopActive) {
        state.position_mode_state = PositionModeState::kFault;
    } else if (command.requested_mode != MotionMode::kProfilePosition) {
        state.position_mode_state = PositionModeState::kIdle;
    } else if (state.position_mode_state == PositionModeState::kTimeout) {
    } else if (!state.enabled) {
        state.position_mode_state = PositionModeState::kWaitingEnable;
    } else if (pp_pulse_cycles_remaining_ > 0) {
        state.position_mode_state = PositionModeState::kSendingSetpoint;
    } else if (state.target_reached) {
        state.position_mode_state = PositionModeState::kTargetReached;
    } else {
        state.position_mode_state = PositionModeState::kMoving;
    }

    const bool follow_active = follow_active_.load(std::memory_order_acquire);
    const double velocity_tolerance_deg_s = std::max(0.5, config_.max_velocity_deg_s * 0.02);
    if (state.fault || state.cia402_state == CiA402State::kQuickStopActive) {
        state.follow_mode_state = FollowModeState::kFault;
    } else if (follow_active && !state.enabled) {
        state.follow_mode_state = FollowModeState::kWaitingEnable;
    } else if (follow_active && state.motion_mode != MotionMode::kCyclicSyncVelocity) {
        state.follow_mode_state = FollowModeState::kEnteringCsvMode;
    } else if (follow_active) {
        state.follow_mode_state = FollowModeState::kFollowing;
    } else if (state.follow_mode_state == FollowModeState::kStopping &&
               std::fabs(state.actual_velocity_deg_s) > velocity_tolerance_deg_s) {
        state.follow_mode_state = FollowModeState::kStopping;
    } else {
        state.follow_mode_state = FollowModeState::kIdle;
    }

    state_buffer_.publish(state);
}

void ErobAxis::updateLastErrorCode(int error_code) {
    AxisState state = state_buffer_.load();
    state.last_error_code = error_code;
    state_buffer_.publish(state);
}

void ErobAxis::markProfilePositionTimeout() {
    AxisState state = state_buffer_.load();
    state.position_mode_state = PositionModeState::kTimeout;
    state.target_reached = false;
    state_buffer_.publish(state);
}

RxPdoUnified ErobAxis::buildRxPdoForCycle(int cycle_hz) {
    const AxisState state = state_buffer_.load();
    const AxisCommand command = command_buffer_.load();
    const bool control_transition_requested =
        command.quick_stop_requested ||
        command.disable_requested ||
        command.reset_fault_requested ||
        !command.enable_requested;

    if (command.sequence != last_cycle_sequence_) {
        if (!control_transition_requested && command.requested_mode == MotionMode::kProfilePosition) {
            target_position_count_ = angleToCount(command.target_angle_deg);
            pp_control_toggle_ ^= 0x0040U;
            pp_pulse_cycles_remaining_ = 1;
            setPositionModeState(PositionModeState::kSendingSetpoint);
        } else if (!control_transition_requested && command.requested_mode == MotionMode::kCyclicSyncVelocity) {
            interp_start_velocity_deg_s_ = interpolated_velocity_deg_s_;
            interp_target_velocity_deg_s_ = command.target_velocity_deg_s;
            interp_steps_ = std::max(1, cycle_hz / std::max(1, static_cast<int>(config_.follow.control_rate_hz)));
            interp_step_ = 0;
            if (follow_active_.load(std::memory_order_acquire)) {
                setFollowModeState(FollowModeState::kEnteringCsvMode);
            }
        } else if (control_transition_requested) {
            pp_pulse_cycles_remaining_ = 0;
        }
        last_cycle_sequence_ = command.sequence;
    }

    RxPdoUnified pdo{};
    pdo.mode_of_operation = MotionModeToCia402Value(command.requested_mode != MotionMode::kNone
            ? command.requested_mode
            : state.motion_mode);
    pdo.controlword = computeControlword(state, command);
    pdo.target_position = target_position_count_;

    if (!control_transition_requested &&
        command.requested_mode == MotionMode::kProfilePosition &&
        state.cia402_state == CiA402State::kOperationEnabled) {
        pdo.controlword = static_cast<uint16_t>(0x000FU | pp_control_toggle_);
    }

    if (command.requested_mode == MotionMode::kCyclicSyncVelocity) {
        if (interp_step_ < interp_steps_) {
            const double ratio = static_cast<double>(interp_step_) / static_cast<double>(interp_steps_);
            interpolated_velocity_deg_s_ =
                interp_start_velocity_deg_s_ +
                ratio * (interp_target_velocity_deg_s_ - interp_start_velocity_deg_s_);
            ++interp_step_;
        } else {
            interpolated_velocity_deg_s_ = interp_target_velocity_deg_s_;
        }
        interpolated_velocity_deg_s_ = applyVelocityLimiter(state, interpolated_velocity_deg_s_);
        pdo.target_velocity = velocityToCount(interpolated_velocity_deg_s_);
        pdo.target_position = state.actual_position_count;
    }

    if (!control_transition_requested &&
        command.requested_mode == MotionMode::kProfilePosition &&
        state.cia402_state == CiA402State::kOperationEnabled &&
        pp_pulse_cycles_remaining_ > 0) {
        pdo.controlword = static_cast<uint16_t>(0x001FU | pp_control_toggle_);
        --pp_pulse_cycles_remaining_;
    }

    AxisState next_state = state;
    next_state.controlword = pdo.controlword;
    state_buffer_.publish(next_state);

    return pdo;
}

bool ErobAxis::isNearPositiveLimit() const {
    return state_buffer_.load().near_positive_limit;
}

bool ErobAxis::isNearNegativeLimit() const {
    return state_buffer_.load().near_negative_limit;
}

bool ErobAxis::isFollowActive() const {
    return follow_active_.load(std::memory_order_acquire);
}

uint64_t ErobAxis::profileRequestId() const {
    return profile_request_id_.load(std::memory_order_acquire);
}

uint64_t ErobAxis::followRequestId() const {
    return follow_request_id_.load(std::memory_order_acquire);
}

uint16_t ErobAxis::slaveIndex() const {
    return config_.bound_motor.slave_index;
}

bool ErobAxis::hasBoundMotor() const {
    return config_.bound_motor.slave_index != 0;
}

bool ErobAxis::publishCommand(const AxisCommand& command) {
    AxisCommand latched_command = command;
    if (quick_stop_latched_.load(std::memory_order_acquire)) {
        latched_command.quick_stop_requested = true;
        latched_command.enable_requested = true;
        latched_command.disable_requested = false;
        latched_command.reset_fault_requested = false;
        latched_command.target_velocity_deg_s = 0.0;
    }
    command_buffer_.publish(latched_command);
    return true;
}

double ErobAxis::clampAngle(double angle_deg) const {
    return Clamp(angle_deg, config_.min_angle_deg, config_.max_angle_deg);
}

int32_t ErobAxis::angleToCount(double angle_deg) const {
    return static_cast<int32_t>(
        std::llround(config_.home_offset_count + clampAngle(angle_deg) * config_.counts_per_degree));
}

double ErobAxis::countToAngle(int32_t count) const {
    if (std::fabs(config_.counts_per_degree) < 1e-9) {
        return 0.0;
    }
    return static_cast<double>(count - config_.home_offset_count) / config_.counts_per_degree;
}

int32_t ErobAxis::velocityToCount(double velocity_deg_s) const {
    return static_cast<int32_t>(std::llround(velocity_deg_s * config_.counts_per_degree));
}

double ErobAxis::countToVelocity(int32_t count_per_second) const {
    if (std::fabs(config_.counts_per_degree) < 1e-9) {
        return 0.0;
    }
    return static_cast<double>(count_per_second) / config_.counts_per_degree;
}

uint16_t ErobAxis::computeControlword(const AxisState& state, const AxisCommand& command) const {
    if (command.quick_stop_requested) {
        return 0x0002;
    }
    if (command.reset_fault_requested && state.fault) {
        return 0x0080;
    }
    if (command.disable_requested) {
        return 0x0006;
    }
    if (!command.enable_requested) {
        return 0x0000;
    }

    switch (state.cia402_state) {
    case CiA402State::kSwitchOnDisabled:
        return 0x0006;
    case CiA402State::kReadyToSwitchOn:
        return 0x0007;
    case CiA402State::kSwitchedOn:
        return 0x000F;
    case CiA402State::kOperationEnabled:
        return 0x000F;
    case CiA402State::kQuickStopActive:
        return 0x000F;
    case CiA402State::kFault:
        return 0x0080;
    case CiA402State::kFaultReactionActive:
    case CiA402State::kNotReadyToSwitchOn:
    default:
        return 0x0006;
    }
}

double ErobAxis::applyVelocityLimiter(const AxisState& state, double velocity_deg_s) const {
    double limited = Clamp(
        velocity_deg_s,
        -config_.follow.max_velocity_deg_s,
        config_.follow.max_velocity_deg_s);
    const double margin = std::max(0.001, config_.follow.position_limit_margin_deg);

    if (state.actual_angle_deg > (config_.max_angle_deg - margin) && limited > 0.0) {
        const double remaining = config_.max_angle_deg - state.actual_angle_deg;
        const double scale = Clamp(remaining / margin, 0.0, 1.0);
        limited *= scale;
    }
    if (state.actual_angle_deg < (config_.min_angle_deg + margin) && limited < 0.0) {
        const double remaining = state.actual_angle_deg - config_.min_angle_deg;
        const double scale = Clamp(remaining / margin, 0.0, 1.0);
        limited *= scale;
    }
    const bool in_limit_zone =
        state.actual_angle_deg > (config_.max_angle_deg - margin) ||
        state.actual_angle_deg < (config_.min_angle_deg + margin);
    if (in_limit_zone && std::fabs(limited) > 1e-9 && std::fabs(limited) < 0.5) {
        limited = 0.0;
    }
    if (state.actual_angle_deg >= config_.max_angle_deg) {
        limited = std::min(limited, 0.0);
    }
    if (state.actual_angle_deg <= config_.min_angle_deg) {
        limited = std::max(limited, 0.0);
    }
    return limited;
}

void ErobAxis::setLastError(const std::string& message) {
    last_error_ = message;
}

void ErobAxis::setPositionModeState(PositionModeState state) {
    AxisState axis_state = state_buffer_.load();
    axis_state.position_mode_state = state;
    state_buffer_.publish(axis_state);
}

void ErobAxis::setFollowModeState(FollowModeState state) {
    AxisState axis_state = state_buffer_.load();
    axis_state.follow_mode_state = state;
    state_buffer_.publish(axis_state);
}

}  // namespace erob