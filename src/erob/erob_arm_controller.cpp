
// 机械臂控制器主实现文件，负责系统初始化、配置加载、轴管理、运动指令下发等核心流程。
#include "erob/erob_arm_controller.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <unistd.h>

namespace erob {
namespace {

// 设置当前线程为实时优先级
bool SetCurrentThreadRealtime(int priority) {
    sched_param param{};
    param.sched_priority = priority;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0;
}

// 设置当前线程绑定到指定CPU核
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

// 判断轴配置是否已绑定电机身份
bool HasConfiguredMotorIdentity(const AxisConfig& axis_config) {
    return !axis_config.bound_motor.serial_number.empty() ||
        (axis_config.bound_motor.eep_man != 0 &&
         axis_config.bound_motor.eep_id != 0 &&
         axis_config.bound_motor.eep_rev != 0);
}

// 判断发现的电机身份是否与配置匹配
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

// 获取配置中电机身份的文本描述
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

// 返回数值的符号（正1/负1/0）
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

double ModeSwitchVelocityToleranceDegS(const AxisConfig& axis_config) {
    return std::min(0.5, axis_config.max_velocity_deg_s * 0.02);
}

bool IsProfilePositionStillSettling(const AxisConfig& axis_config, const AxisState& state) {
    const bool position_command_active =
        state.position_mode_state == PositionModeState::kSendingSetpoint ||
        state.position_mode_state == PositionModeState::kMoving ||
        state.position_mode_state == PositionModeState::kTimeout;
    if (!position_command_active) {
        return false;
    }
    if (state.target_reached) {
        return false;
    }
    return std::fabs(state.actual_velocity_deg_s) > ModeSwitchVelocityToleranceDegS(axis_config);
}

bool IsFollowModeBusy(const AxisState& state, const ErobAxis* axis_ptr) {
    return axis_ptr->isFollowActive() ||
        state.follow_mode_state != FollowModeState::kIdle;
}

bool IsAxisBusyState(const AxisState& state, const ErobAxis* axis_ptr) {
    if (axis_ptr == nullptr) {
        return false;
    }
    if (axis_ptr->isFollowActive() || state.follow_mode_state != FollowModeState::kIdle) {
        return true;
    }
    switch (state.position_mode_state) {
    case PositionModeState::kWaitingEnable:
    case PositionModeState::kSendingSetpoint:
    case PositionModeState::kMoving:
    case PositionModeState::kTimeout:
        return true;
    case PositionModeState::kIdle:
    case PositionModeState::kTargetReached:
    case PositionModeState::kFault:
    default:
        return false;
    }
}

bool IsSyncRelatedOperationalFailure(const std::string& error) {
    return error.find("Synchronization error") != std::string::npos ||
        error.find("AL=0x1a") != std::string::npos ||
        error.find("AL=0x1b") != std::string::npos ||
        error.find("AL=0x30") != std::string::npos ||
        error.find("Sync manager watchdog") != std::string::npos ||
        error.find("DC ") != std::string::npos;
}

void LogControllerDebug(int axis_id, const std::string& message) {
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cerr << "[erob-controller-debug] axis=" << axis_id
              << " | " << message << std::endl;
}

std::string JoinMessages(const std::vector<std::string>& messages) {
    std::ostringstream stream;
    for (std::size_t index = 0; index < messages.size(); ++index) {
        if (index != 0) {
            stream << " ; ";
        }
        stream << messages[index];
    }
    return stream.str();
}

std::string DisplayAdapterName(const std::string& adapter_name) {
    return adapter_name.empty() ? std::string("-") : adapter_name;
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

    LogControllerDebug(
        -1,
        "initialize begin | preferred_adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | cached_motors=" + std::to_string(discovered_motors_.size()));

    if (initializeNoRecovery()) {
        LogControllerDebug(
            -1,
            "initialize success | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
                " | discovered_motors=" + std::to_string(discovered_motors_.size()));
        return true;
    }

    const std::string first_error = lastError();
    LogControllerDebug(-1, "initialize primary attempt failed | error=" + first_error);

    std::string recovery_summary;
    if (!recoverBusAndRescanImpl(&recovery_summary)) {
        const std::string recovery_error = lastError();
        setLastError(first_error + " | recovery_rescan_failed=" + recovery_error);
        return false;
    }

    LogControllerDebug(-1, "initialize retry after bus recovery/rescan | " + recovery_summary);
    if (!initializeNoRecovery()) {
        const std::string retry_error = lastError();
        setLastError(first_error + " | recovery_rescan_succeeded | retry_failed=" + retry_error);
        return false;
    }

    LogControllerDebug(
        -1,
        "initialize success after bus recovery/rescan | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));
    return true;
}

bool ErobArmController::shutdown() {
    stopThreads();
    master_.disconnect();
    initialized_.store(false, std::memory_order_release);
    return true;
}

bool ErobArmController::recoverBusAndRescan() {
    return recoverBusAndRescanImpl(nullptr);
}

bool ErobArmController::initializeNoRecovery() {
    LogControllerDebug(-1, "initialize step=connect_and_bind begin");
    if (!connectAndBind()) {
        const std::string error = lastError();
        setLastError("initialize step=connect_and_bind failed | " + error);
        return false;
    }
    LogControllerDebug(
        -1,
        "initialize step=connect_and_bind success | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));

    LogControllerDebug(-1, "initialize step=configure_pdos begin");
    if (!master_.configurePdos()) {
        setLastError("initialize step=configure_pdos failed | " + master_.lastError());
        return false;
    }

    LogControllerDebug(-1, "initialize step=request_safe_operational begin");
    if (!master_.requestSafeOperational()) {
        setLastError("initialize step=request_safe_operational failed | " + master_.lastError());
        return false;
    }

    const int64_t cycle_ns = 1000000000LL / std::max(1, config_.ethercat_cycle_hz);
    LogControllerDebug(
        -1,
        "initialize step=configure_distributed_clocks begin | cycle_ns=" + std::to_string(cycle_ns));
    if (!master_.configureDistributedClocks(cycle_ns)) {
        setLastError("initialize step=configure_distributed_clocks failed | " + master_.lastError());
        return false;
    }

    LogControllerDebug(-1, "initialize step=request_operational begin");
    if (!master_.requestOperational()) {
        const std::string op_error = master_.lastError();
        const bool sync_related = IsSyncRelatedOperationalFailure(op_error);
        if (!sync_related) {
            setLastError("initialize step=request_operational failed | " + op_error);
            return false;
        }

        LogControllerDebug(-1, "initialize OP retry without DC after: " + op_error);

        if (!master_.configureDistributedClocks(0)) {
            setLastError("initialize step=configure_distributed_clocks_retry_without_dc failed | " + master_.lastError());
            return false;
        }
        if (!master_.requestSafeOperational()) {
            setLastError("initialize step=request_safe_operational_retry_without_dc failed | " + master_.lastError());
            return false;
        }
        if (!master_.requestOperational()) {
            setLastError(
                "initialize step=request_operational_retry_without_dc failed | " +
                master_.lastError() + " | retry_without_dc=failed");
            return false;
        }

        LogControllerDebug(-1, "initialize OP retry without DC succeeded");
    }
    if (!startThreads()) {
        setLastError("initialize step=start_threads failed");
        return false;
    }

    initialized_.store(true, std::memory_order_release);
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

bool ErobArmController::scanAndInitialize() {
    const std::vector<MotorIdentity> motors = scanAndBind();
    if (motors.empty()) {
        if (lastError().empty()) {
            setLastError("scanAndInitialize discovered no motors");
        }
        return false;
    }

    if (initialize()) {
        return true;
    }

    const std::string init_error = lastError();
    shutdown();
    setLastError("scan succeeded but auto initialize failed | " + init_error);
    return false;
}

std::vector<MotorIdentity> ErobArmController::scanAndBind() {
    if (!canRescan()) {
        setLastError("scan requires controller shutdown and all axes disabled");
        return {};
    }

    LogControllerDebug(-1, "scanAndBind begin | mode=all_adapters");
    discovered_motors_ = scanMotorsOnAllAdapters();
    if (!discovered_motors_.empty()) {
        config_.preferred_adapter = discovered_motors_.front().adapter_name;
    }
    config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
    autoBindDiscoveredMotors(discovered_motors_);
    LogControllerDebug(
        -1,
        "scanAndBind complete | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));
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

    std::vector<MotorIdentity> motors;
    std::string detail;
    if (!discoverMotorsOnAdapter(adapter_name, &motors, &detail)) {
        setLastError(detail);
        return {};
    }

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
    LogControllerDebug(-1, "rescanAllAdapters begin");
    discovered_motors_ = scanMotorsOnAllAdapters();
    if (!discovered_motors_.empty()) {
        config_.preferred_adapter = discovered_motors_.front().adapter_name;
    }
    if (discovered_motors_.empty()) {
        setLastError("no EtherCAT motors discovered during full rescan");
    }
    config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
    autoBindDiscoveredMotors(discovered_motors_);
    LogControllerDebug(
        -1,
        "rescanAllAdapters complete | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));
    return discovered_motors_;
}

bool ErobArmController::connect(const std::string& adapter_name) {
    LogControllerDebug(-1, "connect begin | adapter=" + DisplayAdapterName(adapter_name));
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
    LogControllerDebug(
        -1,
        "connect discover success | adapter=" + DisplayAdapterName(adapter_name) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));
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
        LogControllerDebug(
            -1,
            "connectAndBind preferred adapter attempt | adapter=" + DisplayAdapterName(config_.preferred_adapter));
        if (connect(config_.preferred_adapter)) {
            config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
            LogControllerDebug(
                -1,
                "connectAndBind preferred adapter success | adapter=" +
                    DisplayAdapterName(config_.preferred_adapter) +
                    " | discovered_motors=" + std::to_string(discovered_motors_.size()));
            return true;
        }
        failure_details.push_back(
            "preferred adapter " + config_.preferred_adapter + ": " + lastError());
    }

    LogControllerDebug(-1, "connectAndBind fallback scanAdapters begin");
    std::vector<AdapterInfo> adapters = scanAdapters();
    if (adapters.empty()) {
        setLastError("no adapters found");
        return false;
    }

    for (const AdapterInfo& adapter : adapters) {
        LogControllerDebug(
            -1,
            "connectAndBind adapter attempt | adapter=" + DisplayAdapterName(adapter.name) +
                " | probe_scan_success=" + (adapter.scan_success ? std::string("true") : std::string("false")) +
                " | probe_discovered_slave_count=" + std::to_string(adapter.discovered_slave_count));
        if (connect(adapter.name)) {
            config_manager_.saveDiscoveryCache(discoveryCachePath(), adapter.name, discovered_motors_);
            LogControllerDebug(
                -1,
                "connectAndBind adapter success | adapter=" + DisplayAdapterName(adapter.name) +
                    " | discovered_motors=" + std::to_string(discovered_motors_.size()));
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
            target_axis->disable();
            setLastError("axis went offline during enableAxis");
            return false;
        }
        if (state.enabled) {
            return true;
        }
        if (state.fault) {
            target_axis->disable();
            std::ostringstream stream;
            stream << "axis fault during enableAxis, error_code=0x" << std::hex << state.last_error_code;
            setLastError(stream.str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    target_axis->disable();
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

    AxisState state = target_axis->getState();
    if (!state.online) {
        setLastError("axis is not online yet");
        return false;
    }
    if (!target_axis->disable()) {
        setLastError(target_axis->lastError());
        return false;
    }

    const auto disable_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < disable_deadline) {
        state = target_axis->getState();
        if (!state.online) {
            setLastError("axis went offline during disableAxis");
            return false;
        }
        if (!state.enabled) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    setLastError("disableAxis timed out before drive left operation-enabled state");
    return false;
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

    AxisState state = target_axis->getState();
    if (!state.online) {
        setLastError("axis is not online yet");
        return false;
    }
    if (!target_axis->resetFault()) {
        setLastError(target_axis->lastError());
        return false;
    }

    const auto reset_deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < reset_deadline) {
        state = target_axis->getState();
        if (!state.online) {
            setLastError("axis went offline during resetFault");
            return false;
        }
        if (!state.fault && state.cia402_state != CiA402State::kFaultReactionActive) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream stream;
    stream << "resetFault timed out, state=" << CiA402StateName(state.cia402_state)
           << ", error_code=0x" << std::hex << state.last_error_code;
    setLastError(stream.str());
    return false;
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

MoveCommandStatus ErobArmController::issueMoveTo(int axis_id, double angle_deg, double velocity_deg_s) {
    uint64_t request_id = 0;
    int timeout_ms = 0;
    return issueProfilePositionMove(
        axis_id,
        angle_deg,
        velocity_deg_s,
        false,
        &request_id,
        &timeout_ms);
}

MoveCommandStatus ErobArmController::moveTo(int axis_id, double angle_deg, double velocity_deg_s) {
    uint64_t request_id = 0;
    int timeout_ms = 0;
    const MoveCommandStatus issue_status = issueProfilePositionMove(
            axis_id,
            angle_deg,
            velocity_deg_s,
            true,
            &request_id,
            &timeout_ms);
    if (issue_status != MoveCommandStatus::kIssued) {
        return issue_status;
    }
    return waitForProfilePositionMove(axis_id, request_id, timeout_ms);
}

bool ErobArmController::isAxisBusy(int axis_id) const {
    const ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr || !target_axis->hasBoundMotor()) {
        return false;
    }
    return IsAxisBusyState(target_axis->getState(), target_axis);
}

MoveCommandStatus ErobArmController::moveGroup(
    const std::vector<AxisMoveRequest>& requests,
    bool wait_all,
    bool strict_mode) {
    if (requests.empty()) {
        return wait_all ? MoveCommandStatus::kCompleted : MoveCommandStatus::kIssued;
    }

    std::vector<bool> seen_axes(axes_.size(), false);
    std::vector<int> needs_follow_stop(requests.size(), 0);
    std::vector<int> timeouts_ms(requests.size(), 0);
    for (const AxisMoveRequest& request : requests) {
        if (request.axis_id < 0 || request.axis_id >= static_cast<int>(axes_.size())) {
            return rejectMoveCommand("moveGroup axis id out of range");
        }
        ErobAxis* target_axis = axis(request.axis_id);
        if (target_axis == nullptr || !target_axis->hasBoundMotor()) {
            return rejectMoveCommand("moveGroup contains an unbound axis");
        }
        if (seen_axes[request.axis_id]) {
            return rejectMoveCommand("moveGroup contains duplicate axis ids");
        }
        seen_axes[request.axis_id] = true;
    }

    if (strict_mode) {
        for (std::size_t index = 0; index < requests.size(); ++index) {
            bool axis_needs_follow_stop = false;
            if (!validateProfilePositionMove(
                    requests[index].axis_id,
                    requests[index].angle_deg,
                    requests[index].velocity_deg_s,
                    true,
                    &axis_needs_follow_stop,
                    &timeouts_ms[index])) {
                std::ostringstream stream;
                stream << "moveGroup strict precheck failed on axis " << requests[index].axis_id;
                const std::string error = lastError();
                if (!error.empty()) {
                    stream << ": " << error;
                }
                return rejectMoveCommand(stream.str());
            }
            needs_follow_stop[index] = axis_needs_follow_stop ? 1 : 0;
        }

        for (std::size_t index = 0; index < requests.size(); ++index) {
            if (needs_follow_stop[index] == 0) {
                continue;
            }
            if (!stopFollowMode(requests[index].axis_id)) {
                std::ostringstream stream;
                stream << "moveGroup strict follow stop failed on axis " << requests[index].axis_id;
                const std::string error = lastError();
                if (!error.empty()) {
                    stream << ": " << error;
                }
                return rejectMoveCommand(stream.str());
            }
        }

        for (std::size_t index = 0; index < requests.size(); ++index) {
            bool follow_stop_not_needed = false;
            if (!validateProfilePositionMove(
                    requests[index].axis_id,
                    requests[index].angle_deg,
                    requests[index].velocity_deg_s,
                    false,
                    &follow_stop_not_needed,
                    &timeouts_ms[index])) {
                std::ostringstream stream;
                stream << "moveGroup strict recheck failed on axis " << requests[index].axis_id;
                const std::string error = lastError();
                if (!error.empty()) {
                    stream << ": " << error;
                }
                return rejectMoveCommand(stream.str());
            }
        }
    }

    std::vector<int> axis_ids;
    std::vector<uint64_t> request_ids;
    int longest_timeout_ms = 0;
    axis_ids.reserve(requests.size());
    request_ids.reserve(requests.size());

    for (std::size_t index = 0; index < requests.size(); ++index) {
        const AxisMoveRequest& request = requests[index];
        uint64_t request_id = 0;
        int timeout_ms = 0;
        const MoveCommandStatus issue_status = strict_mode
            ? issuePreparedProfilePositionMove(
                request.axis_id,
                request.angle_deg,
                request.velocity_deg_s,
                &request_id,
                &timeout_ms)
            : issueProfilePositionMove(
                request.axis_id,
                request.angle_deg,
                request.velocity_deg_s,
                true,
                &request_id,
                &timeout_ms);
        if (issue_status != MoveCommandStatus::kIssued) {
            std::ostringstream stream;
            stream << "moveGroup issue failed on axis " << request.axis_id;
            const std::string error = lastError();
            if (!error.empty()) {
                stream << ": " << error;
            }
            setLastError(stream.str());
            return issue_status;
        }
        axis_ids.push_back(request.axis_id);
        request_ids.push_back(request_id);
        longest_timeout_ms = std::max(longest_timeout_ms, strict_mode ? timeouts_ms[index] : timeout_ms);
    }

    if (!wait_all) {
        return MoveCommandStatus::kIssued;
    }
    return waitForProfilePositionGroup(axis_ids, request_ids, longest_timeout_ms);
}

bool ErobArmController::validateProfilePositionMove(
    int axis_id,
    double angle_deg,
    double velocity_deg_s,
    bool allow_follow_transition,
    bool* needs_follow_stop,
    int* timeout_ms) {
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
    if (needs_follow_stop != nullptr) {
        *needs_follow_stop = false;
    }
    if (IsFollowModeBusy(state, target_axis)) {
        if (!allow_follow_transition) {
            setLastError("axis is in follow mode; stop follow before issueMoveTo");
            return false;
        }
        if (std::fabs(state.actual_velocity_deg_s) > ModeSwitchVelocityToleranceDegS(target_axis->config())) {
            setLastError("axis is in follow mode and still moving; wait until velocity is near zero before moveTo");
            return false;
        }
        if (needs_follow_stop != nullptr) {
            *needs_follow_stop = true;
        }
    }

    const AxisConfig& axis_config = target_axis->config();
    const int local_timeout_ms = EstimateProfilePositionTimeoutMs(
        axis_config,
        state,
        angle_deg,
        velocity_deg_s);
    if (timeout_ms != nullptr) {
        *timeout_ms = local_timeout_ms;
    }
    return true;
}

MoveCommandStatus ErobArmController::issuePreparedProfilePositionMove(
    int axis_id,
    double angle_deg,
    double velocity_deg_s,
    uint64_t* request_id,
    int* timeout_ms) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        return rejectMoveCommand("axis id out of range");
    }
    const AxisState ready_state = target_axis->getState();
    if (!ready_state.online) {
        return rejectMoveCommand("axis went offline before moveTo");
    }
    if (!ready_state.enabled) {
        return rejectMoveCommand("axis must remain enabled before moveTo");
    }

    ProfilePositionParams params;
    uint64_t local_request_id = 0;
    {
        std::lock_guard<std::mutex> lock(*axis_command_mutexes_[axis_id]);
        std::lock_guard<std::mutex> bus_lock(master_.busMutex());
        if (!target_axis->setProfilePositionTarget(angle_deg, velocity_deg_s, &params, &local_request_id)) {
            return rejectMoveCommand(target_axis->lastError());
        }
        if (target_axis->profileRequestId() != local_request_id) {
            return supersedeMoveCommand("moveTo was superseded before profile position parameters were applied");
        }
        if (!applyProfilePositionParams(target_axis->slaveIndex(), params)) {
            return rejectMoveCommand(lastError());
        }
        if (target_axis->profileRequestId() != local_request_id) {
            return supersedeMoveCommand("moveTo was superseded by a newer request");
        }
    }
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "moveTo request issued"
               << " request_id=" << local_request_id
               << " actual_deg=" << ready_state.actual_angle_deg
               << " target_deg=" << angle_deg
               << " velocity_deg_s=" << velocity_deg_s
               << " statusword=0x" << std::hex << ready_state.statusword << std::dec
               << " cia402=" << CiA402StateName(ready_state.cia402_state)
               << " mode=" << MotionModeName(ready_state.motion_mode)
               << " | " << target_axis->profilePositionDebugString();
        LogControllerDebug(axis_id, stream.str());
    }

    const AxisConfig& axis_config = target_axis->config();
    const int local_timeout_ms = EstimateProfilePositionTimeoutMs(
        axis_config,
        ready_state,
        angle_deg,
        velocity_deg_s);
    if (request_id != nullptr) {
        *request_id = local_request_id;
    }
    if (timeout_ms != nullptr) {
        *timeout_ms = local_timeout_ms;
    }
    return MoveCommandStatus::kIssued;
}

MoveCommandStatus ErobArmController::issueProfilePositionMove(
    int axis_id,
    double angle_deg,
    double velocity_deg_s,
    bool allow_follow_transition,
    uint64_t* request_id,
    int* timeout_ms) {
    bool needs_follow_stop = false;
    if (!validateProfilePositionMove(
            axis_id,
            angle_deg,
            velocity_deg_s,
            allow_follow_transition,
            &needs_follow_stop,
            timeout_ms)) {
        return MoveCommandStatus::kRejected;
    }
    if (needs_follow_stop) {
        if (!stopFollowMode(axis_id)) {
            if (lastError().empty()) {
                setLastError("failed to stop follow mode before moveTo");
            }
            return MoveCommandStatus::kRejected;
        }
        bool follow_stop_not_needed = false;
        if (!validateProfilePositionMove(
                axis_id,
                angle_deg,
                velocity_deg_s,
                false,
                &follow_stop_not_needed,
                timeout_ms)) {
            return MoveCommandStatus::kRejected;
        }
    }
    return issuePreparedProfilePositionMove(axis_id, angle_deg, velocity_deg_s, request_id, timeout_ms);
}

MoveCommandStatus ErobArmController::waitForProfilePositionMove(
    int axis_id,
    uint64_t request_id,
    int timeout_ms) {
    ErobAxis* target_axis = axis(axis_id);
    if (target_axis == nullptr) {
        return rejectMoveCommand("axis id out of range");
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    auto next_retrigger_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    auto next_debug_log = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    const AxisState initial_state = target_axis->getState();
    double last_progress_angle_deg = initial_state.actual_angle_deg;
    auto last_progress_time = std::chrono::steady_clock::now();
    int retrigger_count = 0;
    constexpr int kMaxRetriggerCount = 6;

    while (std::chrono::steady_clock::now() < deadline) {
        const AxisState current_state = target_axis->getState();
        const auto now = std::chrono::steady_clock::now();
        if (target_axis->profileRequestId() != request_id) {
            std::ostringstream stream;
            stream << "axis " << axis_id << " move request was superseded while waiting";
            return supersedeMoveCommand(stream.str());
        }
        if (!current_state.online) {
            return interruptMoveCommand("axis went offline during moveTo");
        }
        if (!current_state.enabled || current_state.cia402_state == CiA402State::kQuickStopActive) {
            return interruptMoveCommand("axis left operation-enabled state during moveTo");
        }
        if (current_state.fault) {
            std::ostringstream stream;
            stream << "axis fault during moveTo, error_code=0x" << std::hex << current_state.last_error_code;
            return interruptMoveCommand(stream.str());
        }
        if (current_state.target_reached) {
            return MoveCommandStatus::kCompleted;
        }
        const bool has_progressed =
            std::fabs(current_state.actual_angle_deg - last_progress_angle_deg) > 0.2 ||
            std::fabs(current_state.actual_velocity_deg_s) > 0.8;
        if (has_progressed) {
            last_progress_angle_deg = current_state.actual_angle_deg;
            last_progress_time = now;
        }
        const bool stalled_with_error =
            std::fabs(current_state.position_error_deg) > 0.5 &&
            std::fabs(current_state.actual_velocity_deg_s) <= 0.5 &&
            now >= next_retrigger_deadline &&
            (now - last_progress_time) >= std::chrono::milliseconds(180);
        if (stalled_with_error && retrigger_count < kMaxRetriggerCount) {
            ++retrigger_count;
            next_retrigger_deadline = now + std::chrono::milliseconds(250);
            last_progress_time = now;
            last_progress_angle_deg = current_state.actual_angle_deg;
            {
                std::ostringstream stream;
                stream << std::fixed << std::setprecision(2)
                       << "moveTo retrigger"
                       << " request_id=" << request_id
                       << " retrigger_count=" << retrigger_count
                       << " actual_deg=" << current_state.actual_angle_deg
                       << " target_deg=" << current_state.target_angle_deg
                       << " error_deg=" << current_state.position_error_deg
                       << " velocity_deg_s=" << current_state.actual_velocity_deg_s
                       << " statusword=0x" << std::hex << current_state.statusword << std::dec
                       << " | " << target_axis->profilePositionDebugString();
                LogControllerDebug(axis_id, stream.str());
            }
            if (!target_axis->retriggerProfilePositionTarget(request_id)) {
                return interruptMoveCommand("moveTo failed to retrigger profile position set-point");
            }
        }
        if (now >= next_debug_log) {
            next_debug_log = now + std::chrono::milliseconds(250);
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(2)
                   << "moveTo waiting"
                   << " request_id=" << request_id
                   << " retrigger_count=" << retrigger_count
                   << " actual_deg=" << current_state.actual_angle_deg
                   << " target_deg=" << current_state.target_angle_deg
                   << " error_deg=" << current_state.position_error_deg
                   << " velocity_deg_s=" << current_state.actual_velocity_deg_s
                   << " statusword=0x" << std::hex << current_state.statusword << std::dec
                   << " controlword=0x" << std::hex << current_state.controlword << std::dec
                   << " cia402=" << CiA402StateName(current_state.cia402_state)
                   << " mode=" << MotionModeName(current_state.motion_mode)
                   << " position_state=" << PositionModeStateName(current_state.position_mode_state)
                   << " target_reached=" << (current_state.target_reached ? "true" : "false")
                   << " | " << target_axis->profilePositionDebugString();
            LogControllerDebug(axis_id, stream.str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const AxisState timed_out_state = target_axis->getState();
    target_axis->markProfilePositionTimeout();
    {
        std::ostringstream debug_stream;
        debug_stream << std::fixed << std::setprecision(2)
                     << "moveTo timeout"
                     << " request_id=" << request_id
                     << " actual_deg=" << timed_out_state.actual_angle_deg
                     << " target_deg=" << timed_out_state.target_angle_deg
                     << " error_deg=" << timed_out_state.position_error_deg
                     << " velocity_deg_s=" << timed_out_state.actual_velocity_deg_s
                     << " statusword=0x" << std::hex << timed_out_state.statusword << std::dec
                     << " controlword=0x" << std::hex << timed_out_state.controlword << std::dec
                     << " cia402=" << CiA402StateName(timed_out_state.cia402_state)
                     << " mode=" << MotionModeName(timed_out_state.motion_mode)
                     << " position_state=" << PositionModeStateName(timed_out_state.position_mode_state)
                     << " target_reached=" << (timed_out_state.target_reached ? "true" : "false")
                     << " | " << target_axis->profilePositionDebugString();
        LogControllerDebug(axis_id, debug_stream.str());
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "moveTo timed out before target reached after " << timeout_ms
           << " ms (dynamic profile estimate)"
           << " | actual_deg=" << timed_out_state.actual_angle_deg
           << " | target_deg=" << timed_out_state.target_angle_deg
           << " | error_deg=" << timed_out_state.position_error_deg
           << " | velocity_deg_s=" << timed_out_state.actual_velocity_deg_s
           << " | statusword=0x" << std::hex << timed_out_state.statusword << std::dec
           << " | controlword=0x" << std::hex << timed_out_state.controlword << std::dec
           << " | cia402=" << CiA402StateName(timed_out_state.cia402_state)
           << " | mode=" << MotionModeName(timed_out_state.motion_mode)
           << " | position_state=" << PositionModeStateName(timed_out_state.position_mode_state)
           << " | target_reached=" << (timed_out_state.target_reached ? "true" : "false")
           << " | " << target_axis->profilePositionDebugString();
    return timeoutMoveCommand(stream.str());
}

MoveCommandStatus ErobArmController::waitForProfilePositionGroup(
    const std::vector<int>& axis_ids,
    const std::vector<uint64_t>& request_ids,
    int timeout_ms) {
    if (axis_ids.size() != request_ids.size()) {
        return rejectMoveCommand("moveGroup wait vectors size mismatch");
    }
    if (axis_ids.empty()) {
        return MoveCommandStatus::kCompleted;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::vector<double> last_progress_angle_deg(axis_ids.size(), 0.0);
    std::vector<std::chrono::steady_clock::time_point> last_progress_time(
        axis_ids.size(),
        std::chrono::steady_clock::now());
    std::vector<int> retrigger_count(axis_ids.size(), 0);
    std::vector<bool> completed(axis_ids.size(), false);
    for (std::size_t index = 0; index < axis_ids.size(); ++index) {
        const ErobAxis* target_axis = axis(axis_ids[index]);
        if (target_axis == nullptr) {
            return rejectMoveCommand("moveGroup wait encountered an invalid axis");
        }
        last_progress_angle_deg[index] = target_axis->getState().actual_angle_deg;
    }

    auto next_retrigger_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
    auto next_debug_log = std::chrono::steady_clock::now() + std::chrono::milliseconds(250);
    constexpr int kMaxRetriggerCount = 6;

    while (std::chrono::steady_clock::now() < deadline) {
        bool all_completed = true;
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t index = 0; index < axis_ids.size(); ++index) {
            if (completed[index]) {
                continue;
            }

            ErobAxis* target_axis = axis(axis_ids[index]);
            if (target_axis == nullptr) {
                return rejectMoveCommand("moveGroup wait encountered an invalid axis");
            }
            const AxisState current_state = target_axis->getState();
            if (target_axis->profileRequestId() != request_ids[index]) {
                std::ostringstream stream;
                stream << "moveGroup request on axis " << axis_ids[index] << " was superseded while waiting";
                return supersedeMoveCommand(stream.str());
            }
            if (!current_state.online) {
                std::ostringstream stream;
                stream << "axis " << axis_ids[index] << " went offline during moveGroup";
                return interruptMoveCommand(stream.str());
            }
            if (!current_state.enabled || current_state.cia402_state == CiA402State::kQuickStopActive) {
                std::ostringstream stream;
                stream << "axis " << axis_ids[index] << " left operation-enabled state during moveGroup";
                return interruptMoveCommand(stream.str());
            }
            if (current_state.fault) {
                std::ostringstream stream;
                stream << "axis " << axis_ids[index] << " fault during moveGroup, error_code=0x"
                       << std::hex << current_state.last_error_code;
                return interruptMoveCommand(stream.str());
            }
            if (current_state.target_reached) {
                completed[index] = true;
                continue;
            }

            all_completed = false;
            const bool has_progressed =
                std::fabs(current_state.actual_angle_deg - last_progress_angle_deg[index]) > 0.2 ||
                std::fabs(current_state.actual_velocity_deg_s) > 0.8;
            if (has_progressed) {
                last_progress_angle_deg[index] = current_state.actual_angle_deg;
                last_progress_time[index] = now;
            }

            const bool stalled_with_error =
                std::fabs(current_state.position_error_deg) > 0.5 &&
                std::fabs(current_state.actual_velocity_deg_s) <= 0.5 &&
                now >= next_retrigger_deadline &&
                (now - last_progress_time[index]) >= std::chrono::milliseconds(180);
            if (stalled_with_error && retrigger_count[index] < kMaxRetriggerCount) {
                ++retrigger_count[index];
                next_retrigger_deadline = now + std::chrono::milliseconds(250);
                last_progress_time[index] = now;
                last_progress_angle_deg[index] = current_state.actual_angle_deg;
                if (!target_axis->retriggerProfilePositionTarget(request_ids[index])) {
                    std::ostringstream stream;
                    stream << "moveGroup failed to retrigger axis " << axis_ids[index] << " profile position set-point";
                    return interruptMoveCommand(stream.str());
                }
            }
        }

        if (all_completed) {
            return MoveCommandStatus::kCompleted;
        }

        if (now >= next_debug_log) {
            next_debug_log = now + std::chrono::milliseconds(250);
            std::ostringstream stream;
            stream << "moveGroup waiting";
            for (std::size_t index = 0; index < axis_ids.size(); ++index) {
                const ErobAxis* target_axis = axis(axis_ids[index]);
                if (target_axis == nullptr) {
                    continue;
                }
                const AxisState current_state = target_axis->getState();
                stream << " | axis=" << axis_ids[index]
                       << " actual_deg=" << std::fixed << std::setprecision(2) << current_state.actual_angle_deg
                       << " target_deg=" << current_state.target_angle_deg
                       << " error_deg=" << current_state.position_error_deg
                       << " velocity_deg_s=" << current_state.actual_velocity_deg_s
                       << " target_reached=" << (current_state.target_reached ? "true" : "false");
            }
            LogControllerDebug(-1, stream.str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::ostringstream stream;
    stream << "moveGroup timed out after " << timeout_ms << " ms";
    for (std::size_t index = 0; index < axis_ids.size(); ++index) {
        const ErobAxis* target_axis = axis(axis_ids[index]);
        if (target_axis == nullptr) {
            continue;
        }
        const AxisState state = target_axis->getState();
        stream << " | axis=" << axis_ids[index]
               << " actual_deg=" << std::fixed << std::setprecision(2) << state.actual_angle_deg
               << " target_deg=" << state.target_angle_deg
               << " error_deg=" << state.position_error_deg
               << " velocity_deg_s=" << state.actual_velocity_deg_s
               << " target_reached=" << (state.target_reached ? "true" : "false");
    }
    return timeoutMoveCommand(stream.str());
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
    if (IsProfilePositionStillSettling(target_axis->config(), state)) {
        setLastError("axis is still moving in profile position mode; wait until target reached or velocity is near zero before follow mode");
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
            ModeSwitchVelocityToleranceDegS(target_axis->config());
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
        {
            std::lock_guard<std::mutex> bus_lock(master_.busMutex());
            const int wkc = master_.receiveProcessDataNoLock(EC_TIMEOUTRET);
            const int expected_wkc = master_.expectedWkcNoLock();
            if (expected_wkc > 0 && wkc < expected_wkc) {
                wkc_miss_count_.fetch_add(1, std::memory_order_relaxed);
            } else {
                wkc_miss_count_.store(0, std::memory_order_relaxed);
            }

            for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
                if (axis_ptr == nullptr || !axis_ptr->hasBoundMotor()) {
                    continue;
                }

                TxPdoCommon feedback{};
                if (master_.readAxisFeedbackNoLock(axis_ptr->slaveIndex(), &feedback)) {
                    axis_ptr->updateFeedback(
                        feedback,
                        master_.slaveAlStatusCodeNoLock(axis_ptr->slaveIndex()));
                }
            }

            for (const std::unique_ptr<ErobAxis>& axis_ptr : axes_) {
                if (axis_ptr == nullptr || !axis_ptr->hasBoundMotor()) {
                    continue;
                }
                const RxPdoUnified command = axis_ptr->buildRxPdoForCycle(config_.ethercat_cycle_hz);
                master_.writeAxisCommandNoLock(axis_ptr->slaveIndex(), command);
            }

            master_.sendProcessDataNoLock();
        }

        std::this_thread::sleep_until(next_tick);
    }
}

void ErobArmController::monitorLoop() {
    SetCurrentThreadAffinity(2);
    const auto monitor_time = std::chrono::milliseconds(100);
    int monitor_iteration = 0;
    while (running_.load(std::memory_order_acquire)) {
        bool communication_issue = false;
        {
            std::lock_guard<std::mutex> bus_lock(master_.busMutex());
            communication_issue =
                wkc_miss_count_.load(std::memory_order_relaxed) >= 3 ||
                !master_.allSlavesOperationalNoLock();
        }

        if (communication_issue) {
            bool recovered = false;
            bool all_operational = false;
            {
                std::lock_guard<std::mutex> bus_lock(master_.busMutex());
                recovered = master_.recoverSlavesNoLock();
                all_operational = master_.allSlavesOperationalNoLock();
                if (!all_operational) {
                    master_.requestOperationalNoLock();
                    all_operational = master_.allSlavesOperationalNoLock();
                }
            }
            if (recovered || all_operational) {
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
            bool read_ok = false;
            {
                std::lock_guard<std::mutex> bus_lock(master_.busMutex());
                read_ok = master_.sdoReadU16NoLock(axis_ptr->slaveIndex(), 0x603F, 0x00, &error_code);
            }
            if (read_ok) {
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

bool ErobArmController::discoverMotorsOnAdapter(
    const std::string& adapter_name,
    std::vector<MotorIdentity>* motors,
    std::string* detail) {
    EthercatMasterSession scanner;
    LogControllerDebug(-1, "rescan adapter begin | adapter=" + DisplayAdapterName(adapter_name));
    if (!scanner.connect(adapter_name)) {
        if (detail != nullptr) {
            *detail = "adapter " + adapter_name + " connect failed: " + scanner.lastError();
        }
        LogControllerDebug(-1, "rescan adapter connect failed | adapter=" + DisplayAdapterName(adapter_name) +
            " | error=" + scanner.lastError());
        return false;
    }

    std::vector<MotorIdentity> discovered;
    if (!scanner.discoverMotors(&discovered)) {
        const std::string error = scanner.lastError();
        scanner.disconnect();
        if (detail != nullptr) {
            *detail = "adapter " + adapter_name + " discover failed: " + error;
        }
        LogControllerDebug(-1, "rescan adapter discover failed | adapter=" + DisplayAdapterName(adapter_name) +
            " | error=" + error);
        return false;
    }

    scanner.disconnect();
    if (motors != nullptr) {
        *motors = discovered;
    }
    if (detail != nullptr) {
        *detail = "adapter " + adapter_name + " discovered " + std::to_string(discovered.size()) + " slave(s)";
    }
    LogControllerDebug(
        -1,
        "rescan adapter success | adapter=" + DisplayAdapterName(adapter_name) +
            " | discovered_motors=" + std::to_string(discovered.size()));
    return true;
}

bool ErobArmController::recoverBusAndRescanImpl(std::string* recovery_summary) {
    if (!canRescan()) {
        setLastError("bus recovery/rescan requires controller shutdown and all axes disabled");
        return false;
    }

    const std::string preferred_adapter = !config_.preferred_adapter.empty()
        ? config_.preferred_adapter
        : master_.adapterName();
    LogControllerDebug(
        -1,
        "bus recovery begin | preferred_adapter=" + DisplayAdapterName(preferred_adapter) +
            " | discovered_motors_before=" + std::to_string(discovered_motors_.size()));

    master_.disconnect();
    LogControllerDebug(-1, "bus recovery clear complete | requested disconnect and INIT on previous session");

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::vector<std::string> failure_details;
    std::vector<MotorIdentity> recovered_motors;

    if (!preferred_adapter.empty()) {
        std::string detail;
        if (!discoverMotorsOnAdapter(preferred_adapter, &recovered_motors, &detail)) {
            failure_details.push_back(detail);
        }
    }

    if (recovered_motors.empty()) {
        LogControllerDebug(-1, "bus recovery fallback full rescan begin");
        const std::vector<AdapterInfo> adapters = scanAdapters();
        if (adapters.empty()) {
            failure_details.push_back("no adapters found during bus recovery full rescan");
        }
        for (const AdapterInfo& adapter : adapters) {
            if (!preferred_adapter.empty() && adapter.name == preferred_adapter) {
                continue;
            }

            std::string detail;
            if (discoverMotorsOnAdapter(adapter.name, &recovered_motors, &detail)) {
                break;
            }
            failure_details.push_back(detail);
        }
    }

    if (recovered_motors.empty()) {
        const std::string detail = failure_details.empty()
            ? std::string("bus recovery/rescan found no EtherCAT motors")
            : JoinMessages(failure_details);
        setLastError("bus recovery/rescan failed | " + detail);
        return false;
    }

    discovered_motors_ = recovered_motors;
    config_.preferred_adapter = recovered_motors.front().adapter_name;
    config_manager_.saveDiscoveryCache(discoveryCachePath(), config_.preferred_adapter, discovered_motors_);
    if (!autoBindDiscoveredMotors(discovered_motors_)) {
        setLastError(
            "bus recovery/rescan discovered motors but auto-bind failed | adapter=" +
            DisplayAdapterName(config_.preferred_adapter));
        return false;
    }

    if (recovery_summary != nullptr) {
        *recovery_summary =
            "adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size());
    }
    LogControllerDebug(
        -1,
        "bus recovery success | adapter=" + DisplayAdapterName(config_.preferred_adapter) +
            " | discovered_motors=" + std::to_string(discovered_motors_.size()));
    return true;
}

bool ErobArmController::applyProfilePositionParams(
    uint16_t slave_index,
    const ProfilePositionParams& params) {
    if (!master_.sdoWriteU32NoLock(slave_index, 0x6081, 0x00, params.profile_velocity) ||
        !master_.sdoWriteU32NoLock(slave_index, 0x6083, 0x00, params.profile_acceleration) ||
        !master_.sdoWriteU32NoLock(slave_index, 0x6084, 0x00, params.profile_deceleration)) {
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

MoveCommandStatus ErobArmController::rejectMoveCommand(const std::string& message) {
    setLastError(message);
    return MoveCommandStatus::kRejected;
}

MoveCommandStatus ErobArmController::interruptMoveCommand(const std::string& message) {
    setLastError(message);
    return MoveCommandStatus::kInterrupted;
}

MoveCommandStatus ErobArmController::timeoutMoveCommand(const std::string& message) {
    setLastError(message);
    return MoveCommandStatus::kTimedOut;
}

MoveCommandStatus ErobArmController::supersedeMoveCommand(const std::string& message) {
    setLastError(message);
    return MoveCommandStatus::kSuperseded;
}

void ErobArmController::setLastError(const std::string& message) {
    std::lock_guard<std::mutex> lock(last_error_mutex_);
    last_error_ = message;
}

}  // namespace erob