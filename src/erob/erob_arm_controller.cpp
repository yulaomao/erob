#include "erob/erob_arm_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <unistd.h>

namespace erob {
namespace {

bool SetCurrentThreadRealtime(int priority) {
    sched_param param{};
    param.sched_priority = priority;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
}

bool SetCurrentThreadAffinity(int preferred_cpu) {
    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count <= 0) {
        return false;
    }

    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(std::min<int>(preferred_cpu, static_cast<int>(cpu_count - 1)), &set);
    return pthread_setaffinity_np(pthread_self(), sizeof(set), &set) == 0;
}

bool HasConfiguredMotorIdentity(const AxisConfig& axis_config) {
    return !axis_config.bound_motor.serial_number.empty() ||
        (axis_config.bound_motor.eep_man != 0 &&
         axis_config.bound_motor.eep_id != 0 &&
         axis_config.bound_motor.eep_rev != 0);
}

bool MatchesConfiguredIdentity(const AxisConfig& axis_config, const MotorIdentity& motor) {
    if (!axis_config.bound_motor.serial_number.empty()) {
        return axis_config.bound_motor.serial_number == motor.serial_number;
    }
    if (axis_config.bound_motor.eep_man != 0 &&
        axis_config.bound_motor.eep_id != 0 &&
        axis_config.bound_motor.eep_rev != 0) {
        return axis_config.bound_motor.eep_man == motor.eep_man &&
            axis_config.bound_motor.eep_id == motor.eep_id &&
            axis_config.bound_motor.eep_rev == motor.eep_rev;
    }
    return false;
}

std::string ConfiguredIdentityText(const AxisConfig& axis_config) {
    if (!axis_config.bound_motor.serial_number.empty()) {
        return "serial " + axis_config.bound_motor.serial_number;
    }
    if (axis_config.bound_motor.eep_man != 0 ||
        axis_config.bound_motor.eep_id != 0 ||
        axis_config.bound_motor.eep_rev != 0) {
        std::ostringstream stream;
        stream << "eep(" << axis_config.bound_motor.eep_man
               << "," << axis_config.bound_motor.eep_id
               << "," << axis_config.bound_motor.eep_rev << ")";
        return stream.str();
    }
    return "unconfigured";
}

double Signum(double value) {
    if (value > 0.0) {
        return 1.0;
    }
    if (value < 0.0) {
        return -1.0;
    }
    return 0.0;
}

double EstimateProfilePositionMotionSec(
    const AxisConfig& axis_config,
    const AxisState& state,
    double target_angle_deg,
    double velocity_deg_s) {
    const double clamped_target_angle = Clamp(
        target_angle_deg,
        axis_config.min_angle_deg,
        axis_config.max_angle_deg);
    const double max_velocity = std::max(
        1e-3,
        Clamp(velocity_deg_s, 0.0, axis_config.max_velocity_deg_s));
    const double accel = std::max(1e-3, axis_config.max_accel_deg_s2);
    const double decel = std::max(1e-3, axis_config.max_decel_deg_s2);

    double effective_distance_deg = std::fabs(clamped_target_angle - state.actual_angle_deg);
    const double direction = Signum(clamped_target_angle - state.actual_angle_deg);
    const double current_velocity_deg_s = state.actual_velocity_deg_s;
    const double current_speed_deg_s = std::fabs(current_velocity_deg_s);
    double initial_speed_deg_s = 0.0;
    double precondition_time_sec = 0.0;

    if (direction == 0.0) {
        precondition_time_sec = current_speed_deg_s / decel;
        effective_distance_deg += (current_speed_deg_s * current_speed_deg_s) / (2.0 * decel);
    } else if ((current_velocity_deg_s * direction) < 0.0) {
        precondition_time_sec = current_speed_deg_s / decel;
        effective_distance_deg += (current_speed_deg_s * current_speed_deg_s) / (2.0 * decel);
    } else {
        initial_speed_deg_s = std::min(current_speed_deg_s, max_velocity);
    }

    const double distance_to_max_velocity_deg =
        std::max(0.0, (max_velocity * max_velocity - initial_speed_deg_s * initial_speed_deg_s) / (2.0 * accel));
    const double distance_from_max_velocity_to_stop_deg =
        (max_velocity * max_velocity) / (2.0 * decel);

    double motion_time_sec = 0.0;
    if (effective_distance_deg <= (distance_to_max_velocity_deg + distance_from_max_velocity_to_stop_deg)) {
        const double peak_velocity_squared =
            ((2.0 * effective_distance_deg * accel * decel) + (decel * initial_speed_deg_s * initial_speed_deg_s)) /
            (accel + decel);
        const double peak_velocity_deg_s = std::sqrt(std::max(initial_speed_deg_s * initial_speed_deg_s, peak_velocity_squared));
        motion_time_sec =
            std::max(0.0, peak_velocity_deg_s - initial_speed_deg_s) / accel +
            peak_velocity_deg_s / decel;
    } else {
        const double cruise_distance_deg =
            effective_distance_deg - distance_to_max_velocity_deg - distance_from_max_velocity_to_stop_deg;
        motion_time_sec =
            std::max(0.0, max_velocity - initial_speed_deg_s) / accel +
            cruise_distance_deg / max_velocity +
            max_velocity / decel;
    }

    return precondition_time_sec + motion_time_sec;
}

int EstimateProfilePositionTimeoutMs(
    const AxisConfig& axis_config,
    const AxisState& state,
    double target_angle_deg,
    double velocity_deg_s) {
    const double estimated_motion_sec = EstimateProfilePositionMotionSec(
        axis_config,
        state,
        target_angle_deg,
        velocity_deg_s);
    const double timeout_sec = Clamp(estimated_motion_sec * 3.0 + 0.5, 2.0, 300.0);
    return static_cast<int>(std::ceil(timeout_sec * 1000.0));
}

}  // namespace

ErobArmController::ErobArmController(std::string config_path)
    : config_(DefaultSystemConfig()), config_path_(std::move(config_path)) {}

ErobArmController::~ErobArmController() {
    shutdown();
}

bool ErobArmController::initialize() {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }

    if (!config_path_.empty()) {
        loadConfig(config_path_);
    }
    loadDiscoveryCache();

    if (!connectAndBind()) {
        return false;
    }
    if (!master_.configurePdos()) {
        setLastError(master_.lastError());
        return false;
    }
    if (!master_.requestSafeOperational()) {
        setLastError(master_.lastError());
        return false;
    }
    const int64_t cycle_ns = 1000000000LL / std::max(1, config_.ethercat_cycle_hz);
    if (!master_.configureDistributedClocks(cycle_ns)) {
        setLastError(master_.lastError());
        return false;
    }
    if (!master_.requestOperational()) {
        const std::string op_error = master_.lastError();
        const bool sync_related =
            op_error.find("Synchronization error") != std::string::npos ||
            op_error.find("AL=0x1a") != std::string::npos ||
            op_error.find("AL=0x30") != std::string::npos ||
            op_error.find("DC ") != std::string::npos;
        if (!sync_related) {
            setLastError(op_error);
            return false;
        }

        if (!master_.configureDistributedClocks(0)) {
            setLastError(master_.lastError());
            return false;
        }
        if (!master_.requestSafeOperational()) {
            setLastError(master_.lastError());
            return false;
        }
        if (!master_.requestOperational()) {
            setLastError(master_.lastError() + " | retry_without_dc=failed");
            return false;
        }
    }
    if (!startThreads()) {
        return false;
    }

    initialized_.store(true, std::memory_order_release);
    return true;
}

bool ErobArmController::shutdown() {
    stopThreads();
    master_.disconnect();
    initialized_.store(false, std::memory_order_release);
    return true;
}

bool ErobArmController::loadConfig(const std::string& path) {
    SystemConfig loaded = config_;
    if (!config_manager_.loadFromFile(path, &loaded)) {
        setLastError("failed to load config from " + path);
        return false;
    }
    config_ = loaded;
    syncAxisControlRates();
    config_path_ = path;
    return true;
}

bool ErobArmController::saveConfig(const std::string& path) const {
    return config_manager_.saveToFile(path, config_);
}

std::vector<AdapterInfo> ErobArmController::scanAdapters() {
    return master_.scanAdapters();
}

std::vector<MotorIdentity> ErobArmController::scanMotorsOnAllAdapters() {
    return master_.scanMotorsOnAllAdapters();
}

std::vector<MotorIdentity> ErobArmController::scanAndBind() {
    if (!canRescan()) {
        setLastError("scan requires controller shutdown and all axes disabled");
        return {};
    }

    discovered_motors_ = scanMotorsOnAllAdapters();
    if (!discovered_motors_.empty()) {
        config_.preferred_adapter = discovered_motors_.front().adapter_name;
    }
    config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
    autoBindDiscoveredMotors(discovered_motors_);
    return discovered_motors_;
}

std::vector<MotorIdentity> ErobArmController::rescan() {
    if (!config_.preferred_adapter.empty() || (master_.isConnected() && !master_.adapterName().empty())) {
        return rescanCurrentAdapter();
    }
    return rescanAllAdapters();
}

std::vector<MotorIdentity> ErobArmController::rescanCurrentAdapter() {
    if (!canRescan()) {
        setLastError("rescan requires controller shutdown and all axes disabled");
        return {};
    }

    const std::string adapter_name = !config_.preferred_adapter.empty()
        ? config_.preferred_adapter
        : master_.adapterName();
    if (adapter_name.empty()) {
        setLastError("no preferred adapter available for current-adapter rescan");
        return {};
    }

    EthercatMasterSession scanner;
    if (!scanner.connect(adapter_name)) {
        setLastError(scanner.lastError());
        return {};
    }

    std::vector<MotorIdentity> motors;
    if (!scanner.discoverMotors(&motors)) {
        setLastError(scanner.lastError());
        scanner.disconnect();
        return {};
    }

    scanner.disconnect();
    discovered_motors_ = motors;
    config_.preferred_adapter = adapter_name;
    config_manager_.saveDiscoveryCache(discoveryCachePath(), adapter_name, discovered_motors_);
    autoBindDiscoveredMotors(discovered_motors_);
    return discovered_motors_;
}

std::vector<MotorIdentity> ErobArmController::rescanAllAdapters() {
    if (!canRescan()) {
        setLastError("rescan requires controller shutdown and all axes disabled");
        return {};
    }
    discovered_motors_ = scanMotorsOnAllAdapters();
    if (!discovered_motors_.empty()) {
        config_.preferred_adapter = discovered_motors_.front().adapter_name;
    }
    if (discovered_motors_.empty()) {
        setLastError("no EtherCAT motors discovered during full rescan");
    }
    config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
    autoBindDiscoveredMotors(discovered_motors_);
    return discovered_motors_;
}

bool ErobArmController::connect(const std::string& adapter_name) {
    if (!master_.connect(adapter_name)) {
        setLastError(master_.lastError());
        return false;
    }

    std::vector<MotorIdentity> motors;
    if (!master_.discoverMotors(&motors)) {
        setLastError(master_.lastError());
        master_.disconnect();
        return false;
    }

    discovered_motors_ = motors;
    config_.preferred_adapter = adapter_name;
    return autoBindDiscoveredMotors(discovered_motors_);
}

bool ErobArmController::bindAxis(int axis_id, const MotorIdentity& motor) {
    if (axis_id < 0 || axis_id >= static_cast<int>(axes_.size())) {
        setLastError("axis id out of range");
        return false;
    }
    if (motor.slave_index == 0) {
        setLastError("cannot bind axis to empty motor identity");
        return false;
    }

    for (int other_axis_id = 0; other_axis_id < static_cast<int>(axes_.size()); ++other_axis_id) {
        if (other_axis_id == axis_id || axes_[other_axis_id] == nullptr) {
            continue;
        }
        const MotorIdentity& bound_motor = axes_[other_axis_id]->config().bound_motor;
        if (bound_motor.slave_index == motor.slave_index && motor.slave_index != 0) {
            setLastError("motor slave is already bound to another axis");
            return false;
        }
        if (!motor.serial_number.empty() && motor.serial_number == bound_motor.serial_number) {
            setLastError("motor serial is already bound to another axis");
            return false;
        }
    }

    config_.axes[axis_id].bound_motor = motor;
    config_.axes[axis_id].logical_axis_id = axis_id;
    config_.axes[axis_id].follow.control_rate_hz = static_cast<double>(config_.follow_control_hz);
    config_.axes[axis_id].follow.max_velocity_deg_s = config_.follow_max_velocity_deg_s;
    if (axis_command_mutexes_[axis_id] == nullptr) {
        axis_command_mutexes_[axis_id] = std::make_unique<std::mutex>();
    }
    axes_[axis_id] = std::make_unique<ErobAxis>(config_.axes[axis_id]);
    const bool bound = axes_[axis_id]->bindMotor(motor);
    if (bound) {
        std::vector<MotorIdentity> motors_for_audit = discovered_motors_;
        bool already_listed = false;
        for (const MotorIdentity& discovered_motor : motors_for_audit) {
            if (discovered_motor.slave_index == motor.slave_index &&
                discovered_motor.adapter_name == motor.adapter_name) {
                already_listed = true;
                break;
            }
        }
        if (!already_listed) {
            motors_for_audit.push_back(motor);
        }
        auditBindingState(motors_for_audit);
    }
    return bound;
}

bool ErobArmController::connectAndBind() {
    loadDiscoveryCache();

    std::vector<std::string> failure_details;

    if (!config_.preferred_adapter.empty()) {
        if (connect(config_.preferred_adapter)) {
            config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
            return true;
        }
        failure_details.push_back(
            "preferred adapter " + config_.preferred_adapter + ": " + lastError());
    }

    std::vector<AdapterInfo> adapters = scanAdapters();
    if (adapters.empty()) {
        setLastError("no adapters found");
        return false;
    }

    for (const AdapterInfo& adapter : adapters) {
        if (connect(adapter.name)) {
            config_manager_.saveDiscoveryCache(discoveryCachePath(), adapter.name, discovered_motors_);
            return true;
        }
        failure_details.push_back(adapter.name + ": " + lastError());
    }

    std::ostringstream stream;
    stream << "failed to connect to any adapter with EtherCAT slaves";
    if (!failure_details.empty()) {
        stream << " | ";
        for (std::size_t index = 0; index < failure_details.size(); ++index) {
            if (index != 0) {
                stream << " ; ";
            }
            stream << failure_details[index];
        }
    }
    setLastError(stream.str());
    return false;
}

bool ErobArmController::enableAxis(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }

    AxisState state = target_axis->getState();
    if (!state.online) {
        setLastError("axis is not online yet");
        return false;
    }

    if (state.fault) {
        if (!target_axis->resetFault()) {
            setLastError(target_axis->lastError());
            return false;
        }
        const auto reset_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
        while (std::chrono::steady_clock::now() < reset_deadline) {
            state = target_axis->getState();
            if (!state.fault) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (state.fault) {
            setLastError("axis fault did not clear during enableAxis");
            return false;
        }
    }

    if (!target_axis->enable()) {
        setLastError(target_axis->lastError());
        return false;
    }

    const auto enable_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (std::chrono::steady_clock::now() < enable_deadline) {
        state = target_axis->getState();
        if (!state.online) {
            setLastError("axis went offline during enableAxis");
            return false;
        }
        if (state.enabled) {
            return true;
        }
        if (state.fault) {
            std::ostringstream stream;
            stream << "axis fault during enableAxis, error_code=0x" << std::hex << state.last_error_code;
            setLastError(stream.str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setLastError("enableAxis timed out before reaching operation enabled");
    return false;
}

bool ErobArmController::disableAxis(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }
    return target_axis->disable();
}

bool ErobArmController::resetFault(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }
    return target_axis->resetFault();
}

bool ErobArmController::enableAll() {
    bool ok = !isDegraded();
    for (int axis_id = 0; axis_id < static_cast<int>(axes_.size()); ++axis_id) {
        if (axis(axis_id) != nullptr) {
            ok = enableAxis(axis_id) && ok;
        }
    }
    if (!ok && isDegraded()) {
        setLastError("controller is degraded; enableAll only applied to bound axes");
    }
    return ok;
}

bool ErobArmController::disableAll() {
    bool ok = true;
    for (int axis_id = 0; axis_id < static_cast<int>(axes_.size()); ++axis_id) {
        if (axis(axis_id) != nullptr) {
            ok = disableAxis(axis_id) && ok;
        }
    }
    return ok;
}

bool ErobArmController::moveTo(int axis_id, double angle_deg, double velocity_deg_s) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }

    const AxisState state = target_axis->getState();
    if (!state.online) {
        setLastError("axis is not online yet");
        return false;
    }
    if (!state.enabled) {
        setLastError("axis must be enabled before moveTo");
        return false;
    }

    ProfilePositionParams params;
    uint64_t request_id = 0;
    {
        std::lock_guard<std::mutex> lock(*axis_command_mutexes_[axis_id]);
        if (!target_axis->setProfilePositionTarget(angle_deg, velocity_deg_s, &params, &request_id)) {
            setLastError(target_axis->lastError());
            return false;
        }
        if (target_axis->profileRequestId() != request_id) {
            setLastError("moveTo was superseded before profile position parameters were applied");
            return false;
        }
        if (!applyProfilePositionParams(target_axis->slaveIndex(), params)) {
            return false;
        }
        if (target_axis->profileRequestId() != request_id) {
            setLastError("moveTo was superseded by a newer request");
            return false;
        }
    }

    const AxisConfig& axis_config = target_axis->config();
    const int timeout_ms = EstimateProfilePositionTimeoutMs(
        axis_config,
        state,
        angle_deg,
        velocity_deg_s);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        const AxisState current_state = target_axis->getState();
        if (target_axis->profileRequestId() != request_id) {
            setLastError("moveTo was superseded by a newer position request");
            return false;
        }
        if (!current_state.online) {
            setLastError("axis went offline during moveTo");
            return false;
        }
        if (!current_state.enabled || current_state.cia402_state == CiA402State::kQuickStopActive) {
            setLastError("axis left operation-enabled state during moveTo");
            return false;
        }
        if (current_state.fault) {
            std::ostringstream stream;
            stream << "axis fault during moveTo, error_code=0x" << std::hex << current_state.last_error_code;
            setLastError(stream.str());
            return false;
        }
        if (current_state.target_reached) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const AxisState timed_out_state = target_axis->getState();
    target_axis->markProfilePositionTimeout();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "moveTo timed out before target reached after " << timeout_ms
           << " ms (dynamic profile estimate)"
           << " | actual_deg=" << timed_out_state.actual_angle_deg
           << " | target_deg=" << timed_out_state.target_angle_deg
           << " | error_deg=" << timed_out_state.position_error_deg
           << " | velocity_deg_s=" << timed_out_state.actual_velocity_deg_s
           << " | statusword=0x" << std::hex << timed_out_state.statusword << std::dec
           << " | cia402=" << CiA402StateName(timed_out_state.cia402_state)
           << " | mode=" << MotionModeName(timed_out_state.motion_mode)
           << " | position_state=" << PositionModeStateName(timed_out_state.position_mode_state)
           << " | target_reached=" << (timed_out_state.target_reached ? "true" : "false");
    setLastError(stream.str());
    return false;
}

bool ErobArmController::startFollowMode(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }
    const AxisState state = target_axis->getState();
    if (!state.online) {
        setLastError("axis is not online yet");
        return false;
    }
    if (!state.enabled) {
        setLastError("axis must be enabled before follow mode");
        return false;
    }
    uint64_t request_id = 0;
    if (!target_axis->enterFollowMode(&request_id)) {
        setLastError(target_axis->lastError());
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(2000);
    while (std::chrono::steady_clock::now() < deadline) {
        const AxisState current_state = target_axis->getState();
        if (target_axis->followRequestId() != request_id) {
            setLastError("startFollowMode was superseded by a newer follow request");
            return false;
        }
        if (!current_state.online) {
            setLastError("axis went offline during startFollowMode");
            return false;
        }
        if (!current_state.enabled || current_state.cia402_state == CiA402State::kQuickStopActive) {
            setLastError("axis left operation-enabled state during startFollowMode");
            return false;
        }
        if (current_state.fault) {
            std::ostringstream stream;
            stream << "axis fault during startFollowMode, error_code=0x" << std::hex
                   << current_state.last_error_code;
            setLastError(stream.str());
            return false;
        }
        if (current_state.motion_mode == MotionMode::kCyclicSyncVelocity &&
            target_axis->isFollowActive()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setLastError("startFollowMode timed out before drive entered CSV display mode");
    return false;
}

bool ErobArmController::updateFollowTarget(int axis_id, double angle_deg) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }
    if (!target_axis->updateFollowTarget(angle_deg)) {
        setLastError(target_axis->lastError());
        return false;
    }
    return true;
}

bool ErobArmController::stopFollowMode(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }

    uint64_t request_id = 0;
    if (!target_axis->stopFollowMode(&request_id)) {
        setLastError(target_axis->lastError());
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < deadline) {
        const AxisState current_state = target_axis->getState();
        const uint64_t current_request_id = target_axis->followRequestId();
        if (current_request_id != request_id) {
            if (target_axis->isFollowActive()) {
                setLastError("stopFollowMode was superseded by a newer start request");
                return false;
            }
            return true;
        }

        if (!current_state.online) {
            setLastError("axis went offline during stopFollowMode");
            return false;
        }
        if (current_state.fault) {
            std::ostringstream stream;
            stream << "axis fault during stopFollowMode, error_code=0x" << std::hex
                   << current_state.last_error_code;
            setLastError(stream.str());
            return false;
        }

        const bool velocity_stopped = std::fabs(current_state.actual_velocity_deg_s) <=
            std::max(0.5, target_axis->config().max_velocity_deg_s * 0.02);
        const bool stopping_complete =
            !target_axis->isFollowActive() &&
            current_state.follow_mode_state == FollowModeState::kIdle &&
            velocity_stopped;
        if (stopping_complete) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setLastError("stopFollowMode timed out before follow stop completed");
    return false;
}

AxisState ErobArmController::getAxisState(int axis_id) const {
    const ErobAxis* target_axis = axis(axis_id);
    return target_axis != nullptr ? target_axis->getState() : AxisState{};
}

std::vector<AxisState> ErobArmController::getAllAxisStates() const {
    std::vector<AxisState> states(axes_.size());
    for (int axis_id = 0; axis_id < static_cast<int>(states.size()); ++axis_id) {
        states[axis_id] = getAxisState(axis_id);
    }
    return states;
}

std::vector<AxisBindingReport> ErobArmController::getBindingReports() const {
    return binding_reports_;
}

std::vector<MotorIdentity> ErobArmController::discoveredMotors() const {
    return discovered_motors_;
}

bool ErobArmController::isDegraded() const {
    return degraded_.load(std::memory_order_acquire);
}

const SystemConfig& ErobArmController::config() const {
    return config_;
}

void ErobArmController::setPreferredAdapter(const std::string& adapter_name) {
    config_.preferred_adapter = adapter_name;
}

bool ErobArmController::quickStopAxis(int axis_id) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        setLastError("axis id out of range");
        return false;
    }
    if (!target_axis->hasBoundMotor()) {
        setLastError("axis is not bound to a motor");
        return false;
    }
    return target_axis->quickStop();
}

bool ErobArmController::quickStopAll() {
    bool ok = true;
    for (int axis_id = 0; axis_id < static_cast<int>(axes_.size()); ++axis_id) {
        if (axis(axis_id) != nullptr) {
            ok = quickStopAxis(axis_id) && ok;
        }
    }
    return ok;
}

std::string ErobArmController::lastError() const {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    return last_error_;
}

ErobAxis* ErobArmController::axis(int axis_id) {
    if (axis_id < 0 || axis_id >= static_cast<int>(axes_.size())) {
        return nullptr;
    }
    return axes_[axis_id].get();
}

const ErobAxis* ErobArmController::axis(int axis_id) const {
    if (axis_id < 0 || axis_id >= static_cast<int>(axes_.size())) {
        return nullptr;
    }
    return axes_[axis_id].get();
}

bool ErobArmController::startThreads() {
    if (running_.exchange(true, std::memory_order_acq_rel)) {
        return true;
    }

    cycle_thread_ = std::thread(&ErobArmController::cycleLoop, this);
    follow_thread_ = std::thread(&ErobArmController::followLoop, this);
    monitor_thread_ = std::thread(&ErobArmController::monitorLoop, this);
    return true;
}

void ErobArmController::stopThreads() {
    if (!running_.exchange(false, std::memory_order_acq_rel)) {
        return;
    }
    if (cycle_thread_.joinable()) {
        cycle_thread_.join();
    }
    if (follow_thread_.joinable()) {
        follow_thread_.join();
    }
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

void ErobArmController::cycleLoop() {
    SetCurrentThreadRealtime(49);
    SetCurrentThreadAffinity(0);
    const auto cycle_time = std::chrono::nanoseconds(1000000000LL / std::max(1, config_.ethercat_cycle_hz));
    auto next_tick = std::chrono::steady_clock::now();

    while (running_.load(std::memory_order_acquire)) {
        next_tick += cycle_time;
        const int wkc = master_.receiveProcessData(EC_TIMEOUTRET);
        if (master_.expectedWkc() > 0 && wkc < master_.expectedWkc()) {
            wkc_miss_count_.fetch_add(1, std::memory_order_relaxed);
        } else {
            wkc_miss_count_.store(0, std::memory_order_relaxed);
        }

        for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
            if (axis_ptr == nullptr || !axis_ptr->hasBoundMotor()) {
                continue;
            }

            TxPdoCommon feedback{};
            if (master_.readAxisFeedback(axis_ptr->slaveIndex(), &feedback)) {
                axis_ptr->updateFeedback(feedback, master_.slaveAlStatusCode(axis_ptr->slaveIndex()));
            }
        }

        for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
            if (axis_ptr == nullptr || !axis_ptr->hasBoundMotor()) {
                continue;
            }
            const RxPdoUnified command = axis_ptr->buildRxPdoForCycle(config_.ethercat_cycle_hz);
            master_.writeAxisCommand(axis_ptr->slaveIndex(), command);
        }

        master_.sendProcessData();

        std::this_thread::sleep_until(next_tick);
    }
}

void ErobArmController::followLoop() {
    SetCurrentThreadRealtime(20);
    SetCurrentThreadAffinity(1);
    const auto follow_time = std::chrono::microseconds(1000000 / std::max(1, config_.follow_control_hz));
    auto next_tick = std::chrono::steady_clock::now();

    while (running_.load(std::memory_order_acquire)) {
        next_tick += follow_time;
        for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
            if (axis_ptr != nullptr) {
                axis_ptr->planFollowStep();
            }
        }
        std::this_thread::sleep_until(next_tick);
    }
}

void ErobArmController::monitorLoop() {
    SetCurrentThreadAffinity(2);
    const auto monitor_time = std::chrono::milliseconds(100);
    int monitor_iteration = 0;
    while (running_.load(std::memory_order_acquire)) {
        const bool communication_issue =
            wkc_miss_count_.load(std::memory_order_relaxed) >= 3 ||
            !master_.allSlavesOperational();

        if (communication_issue) {
            const bool recovered = master_.recoverSlaves();
            if (!master_.allSlavesOperational()) {
                master_.requestOperational();
            }
            if (recovered || master_.allSlavesOperational()) {
                recovery_fail_count_.store(0, std::memory_order_relaxed);
            } else {
                const int failures = recovery_fail_count_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (failures >= 10) {
                    setLastError("communication recovery failed repeatedly");
                    quickStopAll();
                }
            }
        } else {
            recovery_fail_count_.store(0, std::memory_order_relaxed);
        }

        if (config_.fault_policy == FaultPolicy::kAllStop) {
            for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
                if (axis_ptr != nullptr && axis_ptr->getState().fault) {
                    quickStopAll();
                    break;
                }
            }
        }

        const bool read_fault_codes = (monitor_iteration % 10) == 0;
        for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
            if (axis_ptr == nullptr || !axis_ptr->hasBoundMotor()) {
                continue;
            }
            const AxisState state = axis_ptr->getState();
            if (!read_fault_codes && !state.fault && state.last_error_code == 0) {
                continue;
            }

            uint16_t error_code = 0;
            if (master_.sdoReadU16(axis_ptr->slaveIndex(), 0x603F, 0x00, &error_code)) {
                axis_ptr->updateLastErrorCode(static_cast<int>(error_code));
            }
        }
        ++monitor_iteration;
        std::this_thread::sleep_for(monitor_time);
    }
}

bool ErobArmController::loadDiscoveryCache() {
    std::string cached_adapter;
    std::vector<MotorIdentity> cached_motors;
    if (!config_manager_.loadDiscoveryCache(discoveryCachePath(), &cached_adapter, &cached_motors)) {
        return false;
    }

    if (config_.preferred_adapter.empty() && !cached_adapter.empty()) {
        config_.preferred_adapter = cached_adapter;
    }
    if (discovered_motors_.empty() && !cached_motors.empty()) {
        discovered_motors_ = cached_motors;
    }
    return true;
}

bool ErobArmController::canRescan() const {
    if (initialized_.load(std::memory_order_acquire) || running_.load(std::memory_order_acquire)) {
        return false;
    }

    for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
        if (axis_ptr == nullptr) {
            continue;
        }
        const AxisState state = axis_ptr->getState();
        if (state.enabled || axis_ptr->isFollowActive()) {
            return false;
        }
    }
    return true;
}

bool ErobArmController::applyProfilePositionParams(
    uint16_t slave_index,
    const ProfilePositionParams& params) {
    if (!master_.sdoWriteU32(slave_index, 0x6081, 0x00, params.profile_velocity) ||
        !master_.sdoWriteU32(slave_index, 0x6083, 0x00, params.profile_acceleration) ||
        !master_.sdoWriteU32(slave_index, 0x6084, 0x00, params.profile_deceleration)) {
        setLastError("failed to update profile position parameters");
        return false;
    }
    return true;
}

bool ErobArmController::autoBindDiscoveredMotors(const std::vector<MotorIdentity>& motors) {
    rebuildAxesFromDiscoveredMotors(motors);

    auditBindingState(motors);

    bool has_any_axis = false;
    for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
        has_any_axis = has_any_axis || (axis_ptr != nullptr && axis_ptr->hasBoundMotor());
    }
    if (!has_any_axis) {
        setLastError("no axis could be bound to discovered motors");
    }
    return has_any_axis;
}

void ErobArmController::rebuildAxesFromDiscoveredMotors(const std::vector<MotorIdentity>& motors) {
    const std::vector<AxisConfig> previous_axes = config_.axes;
    config_.axes.clear();
    axes_.clear();
    axis_command_mutexes_.clear();
    binding_reports_.clear();

    for (int axis_id = 0; axis_id < static_cast<int>(motors.size()); ++axis_id) {
        AxisConfig axis_config = axis_id < static_cast<int>(previous_axes.size())
            ? previous_axes[axis_id]
            : DefaultAxisConfig(axis_id);
        axis_config.logical_axis_id = axis_id;
        axis_config.follow.control_rate_hz = static_cast<double>(config_.follow_control_hz);
        axis_config.follow.max_velocity_deg_s = config_.follow_max_velocity_deg_s;
        axis_config.bound_motor = motors[axis_id];
        if (axis_config.joint_name.empty()) {
            axis_config.joint_name = "axis_" + std::to_string(axis_id);
        }

        config_.axes.push_back(axis_config);
        axis_command_mutexes_.push_back(std::make_unique<std::mutex>());
        auto axis_ptr = std::make_unique<ErobAxis>(axis_config);
        axis_ptr->bindMotor(motors[axis_id]);
        axes_.push_back(std::move(axis_ptr));
        binding_reports_.push_back(AxisBindingReport{});
    }
}

void ErobArmController::auditBindingState(const std::vector<MotorIdentity>& motors) {
    bool degraded = false;
    for (int axis_id = 0; axis_id < static_cast<int>(config_.axes.size()); ++axis_id) {
        const AxisConfig& axis_config = config_.axes[axis_id];
        AxisBindingReport report;
        report.logical_axis_id = axis_id;
        report.joint_name = axis_config.joint_name;
        report.configured_identity = HasConfiguredMotorIdentity(axis_config);
        report.configured_motor = axis_config.bound_motor;

        if (axes_[axis_id] != nullptr && axes_[axis_id]->hasBoundMotor()) {
            report.bound = true;
            report.discovered_motor = axes_[axis_id]->config().bound_motor;
            std::ostringstream detail;
            detail << "bound to adapter " << report.discovered_motor.adapter_name
                   << " slave " << report.discovered_motor.slave_index;
            if (!report.discovered_motor.serial_number.empty()) {
                detail << " serial " << report.discovered_motor.serial_number;
            }
            report.detail = detail.str();
        } else if (report.configured_identity) {
            report.detail = "configured motor " + ConfiguredIdentityText(axis_config) + " not found on current bus";
            degraded = true;
        } else {
            const bool any_candidate = !motors.empty();
            report.detail = any_candidate
                ? "axis has no configured motor identity; left unbound"
                : "no motor discovered for this axis";
            degraded = true;
        }

        if (axis_id >= static_cast<int>(binding_reports_.size())) {
            binding_reports_.push_back(report);
        } else {
            binding_reports_[axis_id] = report;
        }
    }
    degraded_.store(degraded, std::memory_order_release);
}

void ErobArmController::syncAxisControlRates() {
    axes_.resize(config_.axes.size());
    binding_reports_.resize(config_.axes.size());
    axis_command_mutexes_.resize(config_.axes.size());
    for (int axis_id = 0; axis_id < static_cast<int>(config_.axes.size()); ++axis_id) {
        config_.axes[axis_id].logical_axis_id = axis_id;
        config_.axes[axis_id].follow.control_rate_hz = static_cast<double>(config_.follow_control_hz);
        config_.axes[axis_id].follow.max_velocity_deg_s = config_.follow_max_velocity_deg_s;
        if (axis_command_mutexes_[axis_id] == nullptr) {
            axis_command_mutexes_[axis_id] = std::make_unique<std::mutex>();
        }
    }
}

std::string ErobArmController::discoveryCachePath() const {
    return "config/discovered_motors.json";
}

void ErobArmController::setLastError(const std::string& message) {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    last_error_ = message;
}

}  // namespace erob