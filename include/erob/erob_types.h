#ifndef EROB_TYPES_H_
#define EROB_TYPES_H_

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace erob {

enum class MotionMode {
    kNone = 0,
    kProfilePosition,
    kCyclicSyncVelocity,
    kCyclicSyncPosition,
    kCyclicSyncTorque,
};

enum class CiA402State {
    kNotReadyToSwitchOn,
    kSwitchOnDisabled,
    kReadyToSwitchOn,
    kSwitchedOn,
    kOperationEnabled,
    kQuickStopActive,
    kFaultReactionActive,
    kFault,
};

enum class FaultPolicy {
    kSingleAxisStop,
    kAllStop,
};

enum class PositionModeState {
    kIdle,
    kWaitingEnable,
    kSendingSetpoint,
    kMoving,
    kTargetReached,
    kTimeout,
    kFault,
};

enum class FollowModeState {
    kIdle,
    kWaitingEnable,
    kEnteringCsvMode,
    kFollowing,
    kStopping,
    kFault,
};

struct AdapterInfo {
    std::string name;
    std::string description;
    bool scan_success = false;
    int discovered_slave_count = 0;
};

struct MotorIdentity {
    std::string adapter_name;
    uint16_t slave_index = 0;
    uint16_t alias = 0;
    uint32_t eep_man = 0;
    uint32_t eep_id = 0;
    uint32_t eep_rev = 0;
    std::string name;
    std::string serial_number;
    bool is_erob_motor = false;
};

struct FollowParams {
    double control_rate_hz = 60.0;
    double kp = 8.0;
    double kd = 0.15;
    double deadband_deg = 0.05;
    double max_velocity_deg_s = 60.0;
    double max_accel_deg_s2 = 800.0;
    double max_decel_deg_s2 = 800.0;
    double target_filter_alpha = 0.4;
    double watchdog_timeout_ms = 50.0;
    double position_limit_margin_deg = 5.0;
};

struct AxisConfig {
    int logical_axis_id = -1;
    std::string joint_name;
    double counts_per_degree = 1.0;
    double home_offset_deg = 0.0;
    int32_t home_offset_count = 0;
    int profile_position_timeout_ms = 10000;
    double min_angle_deg = -130.0;
    double max_angle_deg = 130.0;
    double max_velocity_deg_s = 180.0;
    double max_accel_deg_s2 = 360.0;
    double max_decel_deg_s2 = 360.0;
    FollowParams follow;
    MotorIdentity bound_motor;
};

struct SystemConfig {
    std::string preferred_adapter;
    int ethercat_cycle_hz = 1000;
    int follow_control_hz = 60;
    double follow_max_velocity_deg_s = 60.0;
    FaultPolicy fault_policy = FaultPolicy::kAllStop;
    std::vector<AxisConfig> axes;
};

struct AxisBindingReport {
    int logical_axis_id = -1;
    std::string joint_name;
    bool configured_identity = false;
    bool bound = false;
    std::string detail;
    MotorIdentity configured_motor;
    MotorIdentity discovered_motor;
};

struct ProfilePositionParams {
    uint32_t profile_velocity = 0;
    uint32_t profile_acceleration = 0;
    uint32_t profile_deceleration = 0;
};

struct AxisCommand {
    MotionMode requested_mode = MotionMode::kNone;
    double target_angle_deg = 0.0;
    double target_velocity_deg_s = 0.0;
    double target_accel_deg_s2 = 0.0;
    bool command_updated = false;
    bool enable_requested = false;
    bool disable_requested = false;
    bool reset_fault_requested = false;
    bool quick_stop_requested = false;
    uint64_t sequence = 0;
};

struct AxisState {
    bool online = false;
    bool enabled = false;
    bool fault = false;
    uint16_t statusword = 0;
    uint16_t controlword = 0;
    MotionMode motion_mode = MotionMode::kNone;
    CiA402State cia402_state = CiA402State::kSwitchOnDisabled;
    PositionModeState position_mode_state = PositionModeState::kIdle;
    FollowModeState follow_mode_state = FollowModeState::kIdle;
    int32_t actual_position_count = 0;
    int32_t actual_velocity_count_s = 0;
    int16_t actual_torque = 0;
    double actual_angle_deg = 0.0;
    double actual_velocity_deg_s = 0.0;
    double target_angle_deg = 0.0;
    double position_error_deg = 0.0;
    bool target_reached = false;
    bool near_positive_limit = false;
    bool near_negative_limit = false;
    int al_status_code = 0;
    int last_error_code = 0;
};

struct RxPdoUnified {
    uint16_t controlword;
    int32_t target_position;
    int32_t target_velocity;
    int8_t mode_of_operation;
    uint8_t padding;
} __attribute__((packed));

struct TxPdoCommon {
    uint16_t statusword;
    int32_t actual_position;
    int32_t actual_velocity;
    int16_t actual_torque;
    int8_t mode_of_operation_display;
    uint8_t padding;
} __attribute__((packed));

template <typename T>
class AtomicDoubleBuffer {
public:
    AtomicDoubleBuffer() = default;

    explicit AtomicDoubleBuffer(const T& initial) {
        buffers_[0] = initial;
        buffers_[1] = initial;
    }

    void publish(const T& value) {
        const uint32_t current = front_index_.load(std::memory_order_relaxed);
        const uint32_t back = 1U - current;
        buffers_[back] = value;
        front_index_.store(back, std::memory_order_release);
    }

    T load() const {
        return buffers_[front_index_.load(std::memory_order_acquire)];
    }

private:
    alignas(64) std::array<T, 2> buffers_{};
    std::atomic<uint32_t> front_index_{0};
};

inline AxisConfig DefaultAxisConfig(int logical_axis_id) {
    AxisConfig axis;
    axis.logical_axis_id = logical_axis_id;
    axis.joint_name = "axis_" + std::to_string(logical_axis_id);

    if (logical_axis_id == 0) {
        axis.joint_name = "base";
        axis.counts_per_degree = 1456.3;
        axis.max_velocity_deg_s = 180.0;
        axis.max_accel_deg_s2 = 300.0;
        axis.max_decel_deg_s2 = 300.0;
        axis.follow.max_accel_deg_s2 = 600.0;
        axis.follow.max_decel_deg_s2 = 600.0;
    } else {
        axis.counts_per_degree = 1820.0;
        axis.max_velocity_deg_s = 150.0;
        axis.max_accel_deg_s2 = 240.0;
        axis.max_decel_deg_s2 = 240.0;
        axis.follow.kp = 7.5;
        axis.follow.kd = 0.18;
        axis.follow.max_accel_deg_s2 = 550.0;
        axis.follow.max_decel_deg_s2 = 550.0;
        if (logical_axis_id == 1) {
            axis.joint_name = "shoulder";
        } else if (logical_axis_id == 2) {
            axis.joint_name = "elbow";
        }
    }

    return axis;
}

inline SystemConfig DefaultSystemConfig() {
    SystemConfig config;
    config.preferred_adapter = "";
    return config;
}

inline int8_t MotionModeToCia402Value(MotionMode mode) {
    switch (mode) {
    case MotionMode::kProfilePosition:
        return 1;
    case MotionMode::kCyclicSyncPosition:
        return 8;
    case MotionMode::kCyclicSyncVelocity:
        return 9;
    case MotionMode::kCyclicSyncTorque:
        return 10;
    case MotionMode::kNone:
    default:
        return 0;
    }
}

inline MotionMode Cia402ValueToMotionMode(int8_t value) {
    switch (value) {
    case 1:
        return MotionMode::kProfilePosition;
    case 8:
        return MotionMode::kCyclicSyncPosition;
    case 9:
        return MotionMode::kCyclicSyncVelocity;
    case 10:
        return MotionMode::kCyclicSyncTorque;
    default:
        return MotionMode::kNone;
    }
}

inline CiA402State ParseCiA402State(uint16_t statusword) {
    if ((statusword & 0x004F) == 0x0000) {
        return CiA402State::kNotReadyToSwitchOn;
    }
    if ((statusword & 0x004F) == 0x0040) {
        return CiA402State::kSwitchOnDisabled;
    }
    if ((statusword & 0x006F) == 0x0021) {
        return CiA402State::kReadyToSwitchOn;
    }
    if ((statusword & 0x006F) == 0x0023) {
        return CiA402State::kSwitchedOn;
    }
    if ((statusword & 0x006F) == 0x0027) {
        return CiA402State::kOperationEnabled;
    }
    if ((statusword & 0x006F) == 0x0007) {
        return CiA402State::kQuickStopActive;
    }
    if ((statusword & 0x004F) == 0x000F) {
        return CiA402State::kFaultReactionActive;
    }
    if ((statusword & 0x004F) == 0x0008) {
        return CiA402State::kFault;
    }
    return CiA402State::kSwitchOnDisabled;
}

inline bool IsCia402Enabled(CiA402State state) {
    return state == CiA402State::kOperationEnabled;
}

inline const char* MotionModeName(MotionMode mode) {
    switch (mode) {
    case MotionMode::kProfilePosition:
        return "PP";
    case MotionMode::kCyclicSyncVelocity:
        return "CSV";
    case MotionMode::kCyclicSyncPosition:
        return "CSP";
    case MotionMode::kCyclicSyncTorque:
        return "CST";
    case MotionMode::kNone:
    default:
        return "None";
    }
}

inline const char* CiA402StateName(CiA402State state) {
    switch (state) {
    case CiA402State::kNotReadyToSwitchOn:
        return "NotReady";
    case CiA402State::kSwitchOnDisabled:
        return "SwitchOnDisabled";
    case CiA402State::kReadyToSwitchOn:
        return "ReadyToSwitchOn";
    case CiA402State::kSwitchedOn:
        return "SwitchedOn";
    case CiA402State::kOperationEnabled:
        return "OperationEnabled";
    case CiA402State::kQuickStopActive:
        return "QuickStopActive";
    case CiA402State::kFaultReactionActive:
        return "FaultReactionActive";
    case CiA402State::kFault:
        return "Fault";
    default:
        return "Unknown";
    }
}

inline const char* PositionModeStateName(PositionModeState state) {
    switch (state) {
    case PositionModeState::kIdle:
        return "Idle";
    case PositionModeState::kWaitingEnable:
        return "WaitingEnable";
    case PositionModeState::kSendingSetpoint:
        return "SendingSetpoint";
    case PositionModeState::kMoving:
        return "Moving";
    case PositionModeState::kTargetReached:
        return "TargetReached";
    case PositionModeState::kTimeout:
        return "Timeout";
    case PositionModeState::kFault:
        return "Fault";
    default:
        return "Unknown";
    }
}

inline const char* FollowModeStateName(FollowModeState state) {
    switch (state) {
    case FollowModeState::kIdle:
        return "Idle";
    case FollowModeState::kWaitingEnable:
        return "WaitingEnable";
    case FollowModeState::kEnteringCsvMode:
        return "EnteringCsvMode";
    case FollowModeState::kFollowing:
        return "Following";
    case FollowModeState::kStopping:
        return "Stopping";
    case FollowModeState::kFault:
        return "Fault";
    default:
        return "Unknown";
    }
}

inline int64_t SteadyClockNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

template <typename T>
inline T Clamp(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(max_value, value));
}

}  // namespace erob

#endif