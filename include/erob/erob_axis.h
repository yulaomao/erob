#ifndef EROB_AXIS_H_
#define EROB_AXIS_H_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "erob/erob_types.h"

namespace erob {

class ErobAxis {
public:
    explicit ErobAxis(const AxisConfig& config);

    bool bindMotor(const MotorIdentity& motor);
    const AxisConfig& config() const;
    AxisState getState() const;
    std::string lastError() const;

    bool enable();
    bool disable();
    bool resetFault();
    bool quickStop();

    bool setProfilePositionTarget(
        double angle_deg,
        double velocity_deg_s,
        ProfilePositionParams* params,
        uint64_t* request_id = nullptr);
    bool retriggerProfilePositionTarget(uint64_t request_id);
    bool enterFollowMode(uint64_t* request_id = nullptr);
    bool updateFollowTarget(double angle_deg);
    bool stopFollowMode(uint64_t* request_id = nullptr);
    void planFollowStep();

    void updateFeedback(const TxPdoCommon& txpdo, int al_status_code);
    void updateLastErrorCode(int error_code);
    void markProfilePositionTimeout();
    RxPdoUnified buildRxPdoForCycle(int cycle_hz);

    bool isNearPositiveLimit() const;
    bool isNearNegativeLimit() const;
    bool isFollowActive() const;
    uint64_t profileRequestId() const;
    uint64_t followRequestId() const;
    std::string profilePositionDebugString() const;
    uint16_t slaveIndex() const;
    bool hasBoundMotor() const;

private:
    bool publishCommand(const AxisCommand& command);
    bool shouldRejectFollowTarget(double angle_deg, const AxisState& state) const;
    double clampAngle(double angle_deg) const;
    int32_t angleToCount(double angle_deg) const;
    double countToAngle(int32_t count) const;
    int32_t velocityToCount(double velocity_deg_s) const;
    double countToVelocity(int32_t count_per_second) const;
    uint16_t computeControlword(const AxisState& state, const AxisCommand& command) const;
    double applyVelocityLimiter(const AxisState& state, double velocity_deg_s) const;
    void setPositionModeState(PositionModeState state);
    void setFollowModeState(FollowModeState state);
    void setLastError(const std::string& message);

    AxisConfig config_;
    AtomicDoubleBuffer<AxisState> state_buffer_;
    AtomicDoubleBuffer<AxisCommand> command_buffer_;

    std::atomic<double> follow_target_deg_{0.0};
    std::atomic<int64_t> last_follow_target_ns_{0};
    std::atomic<bool> follow_active_{false};
    std::atomic<bool> follow_first_update_pending_{false};
    std::atomic<uint64_t> profile_request_id_{0};
    std::atomic<uint64_t> next_profile_request_id_{1};
    std::atomic<uint64_t> follow_request_id_{0};
    std::atomic<uint64_t> next_follow_request_id_{1};
    std::atomic<bool> quick_stop_latched_{false};
    std::atomic<uint64_t> next_sequence_{1};

    mutable std::recursive_mutex runtime_mutex_;
    std::string last_error_;

    double planner_filtered_target_deg_ = 0.0;
    double planner_velocity_deg_s_ = 0.0;
    double planner_accel_deg_s2_ = 0.0;
    bool follow_settled_hold_ = false;
    bool follow_positive_limit_hold_ = false;
    bool follow_negative_limit_hold_ = false;
    bool profile_transition_pending_ = false;
    double pending_profile_target_deg_ = 0.0;
    double pending_profile_velocity_deg_s_ = 0.0;
    uint64_t last_cycle_sequence_ = 0;
    int pp_pulse_cycles_remaining_ = 0;
    uint16_t pp_control_toggle_ = 0;
    int32_t target_position_count_ = 0;
    double interp_start_velocity_deg_s_ = 0.0;
    double interp_target_velocity_deg_s_ = 0.0;
    double interpolated_velocity_deg_s_ = 0.0;
    int interp_steps_ = 1;
    int interp_step_ = 0;
};

}  // namespace erob

#endif