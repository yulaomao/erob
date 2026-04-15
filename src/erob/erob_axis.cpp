
// 轴控制相关实现文件，负责单个电机轴的状态管理、指令下发、模式切换等核心逻辑。
#include "erob/erob_axis.h"
#include <iomanip>
#include <iostream>
#include <cmath>
#include <mutex>
#include <sstream>

namespace erob {

namespace {

// 跟随模式输入角度允许的最小/最大值（单位：度）
constexpr double kFollowInputMinAngleDeg = -130.0;
constexpr double kFollowInputMaxAngleDeg = 130.0;
// 跟随模式首次目标跳变允许的最大角度
constexpr double kFollowFirstJumpLimitDeg = 40.0;
// 跟随模式后续目标跳变允许的最大角度
constexpr double kFollowJumpLimitDeg = 20.0;
constexpr double kNsToSeconds = 1e-9;

// 线程安全的调试日志输出，便于定位轴的状态变化
void LogAxisDebug(const AxisConfig& config, const std::string& message) {
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cerr << "[erob-axis-debug] axis=" << config.logical_axis_id
              << " joint=" << config.joint_name
              << " slave=" << config.bound_motor.slave_index
              << " | " << message << std::endl;
}

double ModeSwitchVelocityToleranceDegS(const AxisConfig& config) {
    return std::min(0.5, config.max_velocity_deg_s * 0.02);
}

}  // namespace


// 构造函数：初始化轴配置、状态缓冲区和指令缓冲区
ErobAxis::ErobAxis(const AxisConfig& config)
        : config_(config),
            state_buffer_(AxisState{}),
            command_buffer_(AxisCommand{}) {}


// 绑定电机信息到当前轴，返回是否绑定成功
bool ErobAxis::bindMotor(const MotorIdentity& motor) {
    config_.bound_motor = motor;
    return config_.bound_motor.slave_index != 0;
}


// 获取当前轴的配置信息
const AxisConfig& ErobAxis::config() const {
    return config_;
}


// 获取当前轴的最新状态（线程安全）
AxisState ErobAxis::getState() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return state_buffer_.load();
}


// 获取最近一次错误信息
std::string ErobAxis::lastError() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return last_error_;
}


// 使能当前轴，准备进入工作状态
bool ErobAxis::enable() {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    quick_stop_latched_.store(false, std::memory_order_release);
    const AxisState state = state_buffer_.load();
    AxisCommand command = command_buffer_.load();
    if (command.requested_mode == MotionMode::kNone) {
        command.requested_mode = MotionMode::kProfilePosition;
    }
    command.target_angle_deg = state.actual_angle_deg;
    command.command_updated = false;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.reset_fault_requested = false;
    command.target_velocity_deg_s = 0.0;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    profile_transition_pending_ = false;
    pp_pulse_cycles_remaining_ = 0;
    pending_profile_target_deg_ = state.actual_angle_deg;
    pending_profile_velocity_deg_s_ = 0.0;
    target_position_count_ = state.actual_position_count;
    return publishCommand(command);
}


// 关闭当前轴，停止所有运动并重置相关状态
bool ErobAxis::disable() {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    quick_stop_latched_.store(false, std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    const AxisState state = state_buffer_.load();
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    pp_pulse_cycles_remaining_ = 0;
    resetFollowPlannerState(state);
    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kProfilePosition;
    command.target_angle_deg = state.actual_angle_deg;
    command.command_updated = false;
    command.disable_requested = true;
    command.enable_requested = false;
    command.quick_stop_requested = false;
    command.reset_fault_requested = false;
    command.target_velocity_deg_s = 0.0;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    pending_profile_target_deg_ = state.actual_angle_deg;
    pending_profile_velocity_deg_s_ = 0.0;
    target_position_count_ = state.actual_position_count;
    setPositionModeState(PositionModeState::kIdle);
    setFollowModeState(FollowModeState::kIdle);
    return publishCommand(command);
}

bool ErobAxis::resetFault() {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    quick_stop_latched_.store(false, std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    const AxisState state = state_buffer_.load();
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    pp_pulse_cycles_remaining_ = 0;
    resetFollowPlannerState(state);
    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kProfilePosition;
    command.target_angle_deg = state.actual_angle_deg;
    command.command_updated = false;
    command.reset_fault_requested = true;
    command.enable_requested = false;
    command.disable_requested = true;
    command.quick_stop_requested = false;
    command.target_velocity_deg_s = 0.0;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    pending_profile_target_deg_ = state.actual_angle_deg;
    pending_profile_velocity_deg_s_ = 0.0;
    target_position_count_ = state.actual_position_count;
    setPositionModeState(PositionModeState::kIdle);
    setFollowModeState(FollowModeState::kIdle);
    return publishCommand(command);
}

bool ErobAxis::quickStop() {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    quick_stop_latched_.store(true, std::memory_order_release);
    follow_active_.store(false, std::memory_order_release);
    const AxisState state = state_buffer_.load();
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    profile_transition_pending_ = false;
    pp_pulse_cycles_remaining_ = 0;
    resetFollowPlannerState(state);

    AxisCommand command = command_buffer_.load();
    command.target_angle_deg = state.actual_angle_deg;
    command.command_updated = false;
    command.quick_stop_requested = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.reset_fault_requested = false;
    command.target_velocity_deg_s = 0.0;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    pending_profile_target_deg_ = state.actual_angle_deg;
    pending_profile_velocity_deg_s_ = 0.0;
    target_position_count_ = state.actual_position_count;
    setPositionModeState(PositionModeState::kFault);
    setFollowModeState(FollowModeState::kFault);
    return publishCommand(command);
}

bool ErobAxis::setProfilePositionTarget(
    double angle_deg,
    double velocity_deg_s,
    ProfilePositionParams* params,
    uint64_t* request_id) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
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
    resetFollowPlannerState(state);

    command.requested_mode = MotionMode::kProfilePosition;
    command.target_angle_deg = clamped_angle;
    command.target_velocity_deg_s = limited_velocity;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    profile_transition_pending_ = true;
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
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "queue PP request_id=" << new_request_id
               << " sequence=" << command.sequence
               << " target_deg=" << clamped_angle
               << " target_count=" << angleToCount(clamped_angle)
               << " velocity_deg_s=" << limited_velocity
               << " profile_vel=" << params->profile_velocity
               << " profile_acc=" << params->profile_acceleration
               << " profile_dec=" << params->profile_deceleration;
        LogAxisDebug(config_, stream.str());
    }
    if (request_id != nullptr) {
        *request_id = new_request_id;
    }
    return publishCommand(command);
}

bool ErobAxis::enterFollowMode(uint64_t* request_id) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
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
    follow_first_update_pending_.store(true, std::memory_order_release);
    follow_positive_limit_hold_ = false;
    follow_negative_limit_hold_ = false;
    last_follow_target_ns_.store(SteadyClockNowNs(), std::memory_order_release);
    follow_target_deg_.store(state.actual_angle_deg, std::memory_order_release);
    resetFollowPlannerState(state);

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

bool ErobAxis::retriggerProfilePositionTarget(uint64_t request_id) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    if (profile_request_id_.load(std::memory_order_acquire) != request_id) {
        return false;
    }

    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kProfilePosition;
    command.target_angle_deg = pending_profile_target_deg_;
    command.target_velocity_deg_s = pending_profile_velocity_deg_s_;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    profile_transition_pending_ = true;
    setPositionModeState(PositionModeState::kSendingSetpoint);
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "retrigger PP request_id=" << request_id
               << " sequence=" << command.sequence
               << " target_deg=" << pending_profile_target_deg_
               << " target_count=" << angleToCount(pending_profile_target_deg_)
               << " velocity_deg_s=" << pending_profile_velocity_deg_s_;
        LogAxisDebug(config_, stream.str());
    }
    return publishCommand(command);
}

bool ErobAxis::updateFollowTarget(double angle_deg) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    if (!follow_active_.load(std::memory_order_acquire)) {
        setLastError("follow mode is not active");
        return false;
    }
    if (angle_deg < kFollowInputMinAngleDeg || angle_deg > kFollowInputMaxAngleDeg) {
        return true;
    }
    const AxisState state = state_buffer_.load();
    if (shouldRejectFollowTarget(angle_deg, state)) {
        return true;
    }
    updateTargetTrajectory(clampAngle(angle_deg), SteadyClockNowNs());
    follow_first_update_pending_.store(false, std::memory_order_release);
    return true;
}

bool ErobAxis::stopFollowMode(uint64_t* request_id) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    const AxisState state = state_buffer_.load();
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
    pp_pulse_cycles_remaining_ = 0;
    planner_accel_deg_s2_ = 0.0;

    AxisCommand command = command_buffer_.load();
    command.requested_mode = MotionMode::kCyclicSyncVelocity;
    command.target_angle_deg = state.actual_angle_deg;
    command.target_velocity_deg_s = 0.0;
    command.command_updated = true;
    command.enable_requested = true;
    command.disable_requested = false;
    command.quick_stop_requested = false;
    command.sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    pending_profile_target_deg_ = state.actual_angle_deg;
    pending_profile_velocity_deg_s_ = 0.0;
    target_position_count_ = state.actual_position_count;
    setFollowModeState(FollowModeState::kStopping);
    if (request_id != nullptr) {
        *request_id = new_request_id;
    }
    return publishCommand(command);
}

void ErobAxis::updateFeedback(const TxPdoCommon& txpdo, int al_status_code) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
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
    updateMeasuredMotion(&state);
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
        const double velocity_tolerance_deg_s = ModeSwitchVelocityToleranceDegS(config_);
        const bool within_position_tolerance =
            std::fabs(state.position_error_deg) <= position_tolerance_deg;
        const bool within_velocity_tolerance =
            std::fabs(state.actual_velocity_deg_s) <= velocity_tolerance_deg_s;
        const bool reached_bit = (txpdo.statusword & 0x0400U) != 0;
        const bool settled_at_target =
            !profile_transition_pending_ && pp_pulse_cycles_remaining_ == 0 &&
            within_position_tolerance && within_velocity_tolerance;
        state.target_reached = (reached_bit && within_position_tolerance) || settled_at_target;
    } else {
        state.position_error_deg = state.target_angle_deg - state.actual_angle_deg;
        state.target_reached = false;
    }
    state.follow_reference_angle_deg = planner_filtered_target_deg_;
    state.follow_reference_velocity_deg_s = target_track_velocity_deg_s_;
    state.follow_reference_accel_deg_s2 = target_track_accel_deg_s2_;
    state.follow_output_velocity_deg_s = planner_velocity_deg_s_;
    state.follow_output_accel_deg_s2 = planner_accel_deg_s2_;
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
    } else if (profile_transition_pending_ || pp_pulse_cycles_remaining_ > 0) {
        state.position_mode_state = PositionModeState::kSendingSetpoint;
    } else if (state.target_reached) {
        state.position_mode_state = PositionModeState::kTargetReached;
    } else {
        state.position_mode_state = PositionModeState::kMoving;
    }

    const bool follow_active = follow_active_.load(std::memory_order_acquire);
    const double velocity_tolerance_deg_s = ModeSwitchVelocityToleranceDegS(config_);
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
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    AxisState state = state_buffer_.load();
    state.last_error_code = error_code;
    state_buffer_.publish(state);
}

void ErobAxis::markProfilePositionTimeout() {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    AxisState state = state_buffer_.load();
    state.position_mode_state = PositionModeState::kTimeout;
    state.target_reached = false;
    state_buffer_.publish(state);
}

RxPdoUnified ErobAxis::buildRxPdoForCycle(int cycle_hz) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    const AxisState state = state_buffer_.load();
    const AxisCommand command = command_buffer_.load();
    const double dt = 1.0 / std::max(1.0, static_cast<double>(cycle_hz));
    constexpr uint16_t kPpControlwordBase = 0x000FU;
    constexpr uint16_t kPpControlwordNewSetpoint = 0x001FU;
    constexpr int kPpPulseCycles = 3;
    const bool control_transition_requested =
        command.quick_stop_requested ||
        command.disable_requested ||
        command.reset_fault_requested ||
        !command.enable_requested;

    if (command.sequence != last_cycle_sequence_) {
        if (!control_transition_requested &&
            command.command_updated &&
            command.requested_mode == MotionMode::kProfilePosition) {
            target_position_count_ = angleToCount(command.target_angle_deg);
            pp_control_toggle_ ^= 0x0040U;
            pp_pulse_cycles_remaining_ = 0;
            profile_transition_pending_ = true;
            setPositionModeState(PositionModeState::kSendingSetpoint);
            {
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(2)
                       << "arm PP cycle sequence=" << command.sequence
                       << " request_id=" << profile_request_id_.load(std::memory_order_acquire)
                       << " target_deg=" << command.target_angle_deg
                       << " target_count=" << target_position_count_
                       << " toggle=0x" << std::hex << pp_control_toggle_ << std::dec;
                LogAxisDebug(config_, stream.str());
            }
        } else if (!control_transition_requested &&
                   command.command_updated &&
                   command.requested_mode == MotionMode::kCyclicSyncVelocity) {
            planner_velocity_deg_s_ = state.actual_velocity_deg_s;
            planner_accel_deg_s2_ = state.actual_accel_deg_s2;
            interpolated_velocity_deg_s_ = state.actual_velocity_deg_s;
            interp_start_velocity_deg_s_ = state.actual_velocity_deg_s;
            interp_target_velocity_deg_s_ = command.target_velocity_deg_s;
            interp_step_ = 0;
            if (follow_active_.load(std::memory_order_acquire)) {
                setFollowModeState(FollowModeState::kEnteringCsvMode);
            }
        } else if (control_transition_requested) {
            pp_pulse_cycles_remaining_ = 0;
            profile_transition_pending_ = false;
            planner_accel_deg_s2_ = 0.0;
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
        pdo.controlword = static_cast<uint16_t>(kPpControlwordBase | pp_control_toggle_);
    }

    if (command.requested_mode == MotionMode::kCyclicSyncVelocity) {
        double csv_velocity_deg_s = 0.0;
        if (!control_transition_requested) {
            if (follow_active_.load(std::memory_order_acquire)) {
                csv_velocity_deg_s = planFollowVelocityForCycle(state, dt, SteadyClockNowNs());
            } else {
                const double desired_velocity = applyVelocityLimiter(state, command.target_velocity_deg_s);
                const double desired_accel =
                    (desired_velocity - planner_velocity_deg_s_) / std::max(dt, 1e-6);
                csv_velocity_deg_s = advanceVelocityPlanner(state, desired_velocity, desired_accel, dt);
            }
        } else {
            planner_accel_deg_s2_ = 0.0;
            planner_velocity_deg_s_ = 0.0;
        }
        interpolated_velocity_deg_s_ = csv_velocity_deg_s;
        pdo.target_velocity = velocityToCount(csv_velocity_deg_s);
        pdo.target_position = state.actual_position_count;
    }

    if (!control_transition_requested &&
        command.requested_mode == MotionMode::kProfilePosition &&
        state.cia402_state == CiA402State::kOperationEnabled) {
        if (profile_transition_pending_) {
            pdo.controlword = static_cast<uint16_t>(kPpControlwordBase | pp_control_toggle_);
            profile_transition_pending_ = false;
            pp_pulse_cycles_remaining_ = kPpPulseCycles;
            std::ostringstream stream;
            stream << "PP base controlword=0x" << std::hex << pdo.controlword << std::dec
                   << " pulse_cycles=" << pp_pulse_cycles_remaining_
                   << " target_count=" << pdo.target_position;
            LogAxisDebug(config_, stream.str());
        } else if (pp_pulse_cycles_remaining_ > 0) {
            pdo.controlword = static_cast<uint16_t>(kPpControlwordNewSetpoint | pp_control_toggle_);
            if (pp_pulse_cycles_remaining_ == kPpPulseCycles) {
                std::ostringstream stream;
                stream << "PP new-setpoint controlword=0x" << std::hex << pdo.controlword << std::dec
                       << " pulse_cycles=" << pp_pulse_cycles_remaining_
                       << " target_count=" << pdo.target_position;
                LogAxisDebug(config_, stream.str());
            }
            --pp_pulse_cycles_remaining_;
        }
    }

    AxisState next_state = state;
    next_state.controlword = pdo.controlword;
    next_state.follow_reference_angle_deg = planner_filtered_target_deg_;
    next_state.follow_reference_velocity_deg_s = target_track_velocity_deg_s_;
    next_state.follow_reference_accel_deg_s2 = target_track_accel_deg_s2_;
    next_state.follow_output_velocity_deg_s = planner_velocity_deg_s_;
    next_state.follow_output_accel_deg_s2 = planner_accel_deg_s2_;
    state_buffer_.publish(next_state);

    return pdo;
}

bool ErobAxis::isNearPositiveLimit() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return state_buffer_.load().near_positive_limit;
}

bool ErobAxis::isNearNegativeLimit() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return state_buffer_.load().near_negative_limit;
}

bool ErobAxis::isFollowActive() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return follow_active_.load(std::memory_order_acquire);
}

uint64_t ErobAxis::profileRequestId() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return profile_request_id_.load(std::memory_order_acquire);
}

uint64_t ErobAxis::followRequestId() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    return follow_request_id_.load(std::memory_order_acquire);
}

std::string ErobAxis::profilePositionDebugString() const {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    const AxisState state = state_buffer_.load();
    const AxisCommand command = command_buffer_.load();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "profile_req=" << profile_request_id_.load(std::memory_order_acquire)
           << " cmd_seq=" << command.sequence
           << " last_cycle_seq=" << last_cycle_sequence_
           << " profile_pending=" << (profile_transition_pending_ ? "true" : "false")
           << " pulse_cycles=" << pp_pulse_cycles_remaining_
           << " toggle=0x" << std::hex << pp_control_toggle_ << std::dec
           << " target_count=" << target_position_count_
           << " pending_target_deg=" << pending_profile_target_deg_
           << " pending_velocity_deg_s=" << pending_profile_velocity_deg_s_
           << " state_controlword=0x" << std::hex << state.controlword << std::dec
           << " state_statusword=0x" << std::hex << state.statusword << std::dec
            << " setpoint_ack=" << (((state.statusword & 0x1000U) != 0) ? "true" : "false")
           << " cmd_mode=" << MotionModeName(command.requested_mode)
           << " state_mode=" << MotionModeName(state.motion_mode);
    return stream.str();
}

uint16_t ErobAxis::slaveIndex() const {
    return config_.bound_motor.slave_index;
}

bool ErobAxis::hasBoundMotor() const {
    return config_.bound_motor.slave_index != 0;
}

bool ErobAxis::publishCommand(const AxisCommand& command) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
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

bool ErobAxis::shouldRejectFollowTarget(double angle_deg, const AxisState& state) const {
    const double jump_limit_deg = follow_first_update_pending_.load(std::memory_order_acquire)
        ? kFollowFirstJumpLimitDeg
        : kFollowJumpLimitDeg;
    return std::fabs(angle_deg - state.actual_angle_deg) > jump_limit_deg;
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

void ErobAxis::resetFollowPlannerState(const AxisState& state) {
    planner_filtered_target_deg_ = state.actual_angle_deg;
    planner_velocity_deg_s_ = state.actual_velocity_deg_s;
    planner_accel_deg_s2_ = state.actual_accel_deg_s2;
    target_track_velocity_deg_s_ = 0.0;
    target_track_accel_deg_s2_ = 0.0;
    last_follow_target_sample_deg_ = state.actual_angle_deg;
    interp_start_velocity_deg_s_ = state.actual_velocity_deg_s;
    interp_target_velocity_deg_s_ = state.actual_velocity_deg_s;
    interpolated_velocity_deg_s_ = state.actual_velocity_deg_s;
    interp_step_ = 0;
}

void ErobAxis::updateTargetTrajectory(double angle_deg, int64_t now_ns) {
    const double clamped_target = clampAngle(angle_deg);
    const bool first_sample = follow_first_update_pending_.load(std::memory_order_acquire);
    const int64_t previous_ns = last_follow_target_ns_.load(std::memory_order_acquire);
    const double nominal_dt = 1.0 / std::max(1.0, config_.follow.control_rate_hz);

    if (first_sample || previous_ns <= 0 || now_ns <= previous_ns) {
        target_track_velocity_deg_s_ = 0.0;
        target_track_accel_deg_s2_ = 0.0;
    } else {
        const double sample_dt = Clamp(
            static_cast<double>(now_ns - previous_ns) * kNsToSeconds,
            std::max(1e-4, nominal_dt * 0.25),
            std::max(nominal_dt * 4.0, nominal_dt));
        const double target_velocity_limit = std::max(
            config_.follow.max_velocity_deg_s,
            config_.max_velocity_deg_s * 0.75);
        const double previous_velocity = target_track_velocity_deg_s_;
        const double raw_velocity = Clamp(
            (clamped_target - last_follow_target_sample_deg_) / sample_dt,
            -target_velocity_limit,
            target_velocity_limit);
        const double velocity_alpha = Clamp(
            config_.follow.target_filter_alpha + sample_dt / std::max(nominal_dt, 1e-3) * 0.15,
            0.15,
            0.85);
        target_track_velocity_deg_s_ += velocity_alpha * (raw_velocity - target_track_velocity_deg_s_);

        const double accel_limit = std::max(config_.follow.max_accel_deg_s2, config_.follow.max_decel_deg_s2) * 1.5;
        const double raw_accel = Clamp(
            (target_track_velocity_deg_s_ - previous_velocity) / sample_dt,
            -accel_limit,
            accel_limit);
        const double accel_alpha = Clamp(velocity_alpha * 0.5, 0.08, 0.55);
        target_track_accel_deg_s2_ += accel_alpha * (raw_accel - target_track_accel_deg_s2_);
    }

    follow_target_deg_.store(clamped_target, std::memory_order_release);
    last_follow_target_sample_deg_ = clamped_target;
    last_follow_target_ns_.store(now_ns, std::memory_order_release);
}

void ErobAxis::updateMeasuredMotion(AxisState* state) {
    const int64_t now_ns = SteadyClockNowNs();
    if (last_feedback_ns_ <= 0) {
        last_feedback_ns_ = now_ns;
        last_feedback_velocity_deg_s_ = state->actual_velocity_deg_s;
        measured_accel_deg_s2_ = 0.0;
        measured_jerk_deg_s3_ = 0.0;
    } else {
        const double dt = static_cast<double>(now_ns - last_feedback_ns_) * kNsToSeconds;
        if (dt > 1e-6 && dt < 0.1) {
            const double previous_accel = measured_accel_deg_s2_;
            const double raw_accel = (state->actual_velocity_deg_s - last_feedback_velocity_deg_s_) / dt;
            const double accel_alpha = Clamp(dt * 120.0, 0.08, 0.35);
            measured_accel_deg_s2_ += accel_alpha * (raw_accel - measured_accel_deg_s2_);

            const double raw_jerk = (measured_accel_deg_s2_ - previous_accel) / dt;
            const double jerk_alpha = Clamp(dt * 80.0, 0.05, 0.25);
            measured_jerk_deg_s3_ += jerk_alpha * (raw_jerk - measured_jerk_deg_s3_);
        } else if (dt >= 0.1) {
            measured_accel_deg_s2_ = 0.0;
            measured_jerk_deg_s3_ = 0.0;
        }
        last_feedback_ns_ = now_ns;
        last_feedback_velocity_deg_s_ = state->actual_velocity_deg_s;
    }

    state->actual_accel_deg_s2 = measured_accel_deg_s2_;
    state->actual_jerk_deg_s3 = measured_jerk_deg_s3_;
}

double ErobAxis::planFollowVelocityForCycle(const AxisState& state, double dt, int64_t now_ns) {
    if (state.fault || state.cia402_state == CiA402State::kQuickStopActive) {
        setFollowModeState(FollowModeState::kFault);
        planner_velocity_deg_s_ = 0.0;
        planner_accel_deg_s2_ = 0.0;
        return 0.0;
    }
    if (!state.online || !state.enabled) {
        setFollowModeState(FollowModeState::kWaitingEnable);
        planner_velocity_deg_s_ = 0.0;
        planner_accel_deg_s2_ = 0.0;
        return 0.0;
    }

    const double clamped_target = clampAngle(follow_target_deg_.load(std::memory_order_acquire));
    const double nominal_target_dt = 1.0 / std::max(1.0, config_.follow.control_rate_hz);
    const double watchdog_sec = std::max(1e-3, config_.follow.watchdog_timeout_ms * 0.001);
    double stale_sec = std::max(
        0.0,
        static_cast<double>(now_ns - last_follow_target_ns_.load(std::memory_order_acquire)) * kNsToSeconds);
    const bool target_stale = stale_sec > watchdog_sec;

    double reference_target_deg = clamped_target;
    double reference_velocity_deg_s = target_track_velocity_deg_s_;
    double reference_accel_deg_s2 = target_track_accel_deg_s2_;
    if (target_stale) {
        reference_velocity_deg_s = 0.0;
        reference_accel_deg_s2 = 0.0;
        stale_sec = 0.0;
    } else {
        const double lookahead_sec = Clamp(
            nominal_target_dt * 0.5 + 2.0 * dt,
            2.0 * dt,
            std::min(0.03, watchdog_sec));
        const double prediction_sec = Clamp(stale_sec + lookahead_sec, 0.0, watchdog_sec);
        reference_target_deg +=
            reference_velocity_deg_s * prediction_sec +
            0.5 * reference_accel_deg_s2 * prediction_sec * prediction_sec;
        reference_velocity_deg_s += reference_accel_deg_s2 * prediction_sec;
    }

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

    if (target_at_positive_limit) {
        reference_target_deg = std::min(reference_target_deg, config_.max_angle_deg - limit_soft_target_offset_deg);
    } else if (target_at_negative_limit) {
        reference_target_deg = std::max(reference_target_deg, config_.min_angle_deg + limit_soft_target_offset_deg);
    }
    reference_target_deg = clampAngle(reference_target_deg);

    if (follow_positive_limit_hold_ && clamped_target < (config_.max_angle_deg - limit_release_band_deg)) {
        follow_positive_limit_hold_ = false;
    }
    if (follow_negative_limit_hold_ && clamped_target > (config_.min_angle_deg + limit_release_band_deg)) {
        follow_negative_limit_hold_ = false;
    }

    planner_filtered_target_deg_ = reference_target_deg;
    const double position_error_for_hold = std::fabs(reference_target_deg - state.actual_angle_deg);
    if (!(follow_positive_limit_hold_ || follow_negative_limit_hold_)) {
        const bool can_capture_positive_limit =
            target_near_positive_limit &&
            state.actual_angle_deg >= (config_.max_angle_deg - limit_hold_band_deg) &&
            (position_error_for_hold < limit_settle_deg ||
             (std::fabs(state.actual_velocity_deg_s) <= limit_capture_velocity_deg_s &&
              std::fabs(reference_velocity_deg_s) <= limit_capture_velocity_deg_s));
        const bool can_capture_negative_limit =
            target_near_negative_limit &&
            state.actual_angle_deg <= (config_.min_angle_deg + limit_hold_band_deg) &&
            (position_error_for_hold < limit_settle_deg ||
             (std::fabs(state.actual_velocity_deg_s) <= limit_capture_velocity_deg_s &&
              std::fabs(reference_velocity_deg_s) <= limit_capture_velocity_deg_s));
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
        reference_velocity_deg_s = 0.0;
        reference_accel_deg_s2 = 0.0;
    }

    const double position_error_deg = planner_filtered_target_deg_ - state.actual_angle_deg;
    const double position_error_abs_deg = std::fabs(position_error_deg);
    const double velocity_error_deg_s = reference_velocity_deg_s - state.actual_velocity_deg_s;
    const double accel_error_deg_s2 = reference_accel_deg_s2 - state.actual_accel_deg_s2;
    const double response_bandwidth = Clamp(1.35 * std::sqrt(std::max(0.25, config_.follow.kp)), 3.0, 10.0);
    const double damping_ratio = Clamp(0.82 + config_.follow.kd * 1.6, 0.75, 1.45);
    const double accel_feedback_gain = Clamp(0.12 * response_bandwidth, 0.0, 1.5);
    const double jerk_damping_gain = Clamp(0.03 * response_bandwidth, 0.02, 0.25);

    double desired_velocity_deg_s = reference_velocity_deg_s + response_bandwidth * position_error_deg;
    double desired_accel_deg_s2 =
        reference_accel_deg_s2 +
        2.0 * damping_ratio * response_bandwidth * velocity_error_deg_s +
        response_bandwidth * response_bandwidth * position_error_deg +
        accel_feedback_gain * accel_error_deg_s2 -
        jerk_damping_gain * state.actual_jerk_deg_s3 * dt;

    const double terminal_track_band_deg = 2.0;
    const double terminal_min_velocity_deg_s = 1.0;
    const double quasi_static_reference_deg_s = std::max(3.0, config_.follow.max_velocity_deg_s * 0.12);
    if (!holding_limit &&
        position_error_abs_deg > config_.follow.deadband_deg &&
        position_error_abs_deg < terminal_track_band_deg &&
        std::fabs(reference_velocity_deg_s) < quasi_static_reference_deg_s) {
        const double comfortable_terminal_decel = std::max(
            1e-3,
            std::min(config_.follow.max_accel_deg_s2, config_.follow.max_decel_deg_s2) * 0.35);
        const double terminal_velocity_limit_deg_s = std::sqrt(
            std::max(
                0.0,
                2.0 * comfortable_terminal_decel *
                    std::max(0.0, position_error_abs_deg - config_.follow.deadband_deg * 0.4)));
        const double terminal_floor_fade_start_deg = std::max(
            config_.follow.deadband_deg + 0.15,
            (terminal_min_velocity_deg_s * terminal_min_velocity_deg_s) /
                (2.0 * comfortable_terminal_decel));
        const double terminal_floor_ratio = Clamp(
            (position_error_abs_deg - config_.follow.deadband_deg) /
                std::max(terminal_floor_fade_start_deg - config_.follow.deadband_deg, 1e-3),
            0.0,
            1.0);
        const double terminal_velocity_floor_deg_s =
            terminal_min_velocity_deg_s * terminal_floor_ratio;
        double terminal_velocity_mag_deg_s = std::fabs(desired_velocity_deg_s);
        terminal_velocity_mag_deg_s = std::max(terminal_velocity_mag_deg_s, terminal_velocity_floor_deg_s);
        terminal_velocity_mag_deg_s = std::min(terminal_velocity_mag_deg_s, terminal_velocity_limit_deg_s);
        desired_velocity_deg_s = std::copysign(terminal_velocity_mag_deg_s, position_error_deg);

        const double terminal_accel_request_deg_s2 =
            (desired_velocity_deg_s - state.actual_velocity_deg_s) /
            std::max(3.0 * dt, 1e-3);
        desired_accel_deg_s2 = 0.75 * desired_accel_deg_s2 + 0.25 * terminal_accel_request_deg_s2;
    }

    if (holding_limit) {
        desired_velocity_deg_s = 0.0;
        desired_accel_deg_s2 = 0.0;
    } else if (position_error_abs_deg <= config_.follow.deadband_deg &&
               std::fabs(reference_velocity_deg_s) < 0.5 &&
               std::fabs(state.actual_velocity_deg_s) < 0.5) {
        desired_velocity_deg_s = 0.0;
        desired_accel_deg_s2 = -2.0 * damping_ratio * response_bandwidth * state.actual_velocity_deg_s;
    }

    if (state.motion_mode != MotionMode::kCyclicSyncVelocity) {
        setFollowModeState(FollowModeState::kEnteringCsvMode);
    } else {
        setFollowModeState(FollowModeState::kFollowing);
    }

    desired_velocity_deg_s = applyVelocityLimiter(state, desired_velocity_deg_s);
    return advanceVelocityPlanner(state, desired_velocity_deg_s, desired_accel_deg_s2, dt);
}

double ErobAxis::advanceVelocityPlanner(
    const AxisState& state,
    double desired_velocity_deg_s,
    double desired_accel_deg_s2,
    double dt) {
    const double max_accel = std::max(1e-3, config_.follow.max_accel_deg_s2);
    const double max_decel = std::max(1e-3, config_.follow.max_decel_deg_s2);
    const double nominal_target_dt = 1.0 / std::max(1.0, config_.follow.control_rate_hz);
    const double jerk_limit =
        std::max(max_accel, max_decel) / std::max(0.002, nominal_target_dt * 0.6);
    const double accel_step = jerk_limit * dt;
    const double accel_target = Clamp(desired_accel_deg_s2, -max_decel, max_accel);
    if (accel_target > planner_accel_deg_s2_) {
        planner_accel_deg_s2_ = std::min(accel_target, planner_accel_deg_s2_ + accel_step);
    } else {
        planner_accel_deg_s2_ = std::max(accel_target, planner_accel_deg_s2_ - accel_step);
    }

    double next_velocity_deg_s = planner_velocity_deg_s_ + planner_accel_deg_s2_ * dt;
    const double limited_desired_velocity = applyVelocityLimiter(state, desired_velocity_deg_s);
    next_velocity_deg_s = applyVelocityLimiter(state, next_velocity_deg_s);

    const double snap_velocity_band = std::max(0.05, std::fabs(planner_accel_deg_s2_) * dt * 1.5);
    if (std::fabs(limited_desired_velocity - next_velocity_deg_s) <= snap_velocity_band) {
        next_velocity_deg_s = limited_desired_velocity;
        planner_accel_deg_s2_ = 0.0;
    }
    if (std::fabs(limited_desired_velocity) < 0.02 &&
        std::fabs(next_velocity_deg_s) < 0.05 &&
        std::fabs(state.actual_velocity_deg_s) < 0.1) {
        next_velocity_deg_s = 0.0;
        planner_accel_deg_s2_ = 0.0;
    }

    planner_velocity_deg_s_ = next_velocity_deg_s;
    return next_velocity_deg_s;
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
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    last_error_ = message;
}

void ErobAxis::setPositionModeState(PositionModeState state) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    AxisState axis_state = state_buffer_.load();
    axis_state.position_mode_state = state;
    state_buffer_.publish(axis_state);
}

void ErobAxis::setFollowModeState(FollowModeState state) {
    std::lock_guard<std::recursive_mutex> lock(runtime_mutex_);
    AxisState axis_state = state_buffer_.load();
    axis_state.follow_mode_state = state;
    state_buffer_.publish(axis_state);
}

}  // namespace erob