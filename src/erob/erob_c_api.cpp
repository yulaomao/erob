#include "erob/erob_c_api.h"

#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "erob/erob_arm_controller.h"

struct ErobControllerHandle {
    explicit ErobControllerHandle(std::string config_path)
        : controller(std::move(config_path)) {}

    erob::ErobArmController controller;
};

namespace {

thread_local std::string g_result_buffer;

template <typename Callable>
int GuardBool(ErobControllerHandle* handle, Callable&& callable) {
    if (handle == nullptr) {
        g_result_buffer = "invalid controller handle";
        return 0;
    }
    try {
        return callable() ? 1 : 0;
    } catch (const std::exception& error) {
        g_result_buffer = error.what();
        return 0;
    } catch (...) {
        g_result_buffer = "unknown controller exception";
        return 0;
    }
}

ErobMoveCommandStatus ConvertMoveStatus(erob::MoveCommandStatus status) {
    switch (status) {
    case erob::MoveCommandStatus::kCompleted:
        return EROB_MOVE_STATUS_COMPLETED;
    case erob::MoveCommandStatus::kIssued:
        return EROB_MOVE_STATUS_ISSUED;
    case erob::MoveCommandStatus::kSuperseded:
        return EROB_MOVE_STATUS_SUPERSEDED;
    case erob::MoveCommandStatus::kRejected:
        return EROB_MOVE_STATUS_REJECTED;
    case erob::MoveCommandStatus::kTimedOut:
        return EROB_MOVE_STATUS_TIMED_OUT;
    case erob::MoveCommandStatus::kInterrupted:
        return EROB_MOVE_STATUS_INTERRUPTED;
    default:
        return EROB_MOVE_STATUS_REJECTED;
    }
}

template <typename Callable>
ErobMoveCommandStatus GuardMoveStatus(ErobControllerHandle* handle, Callable&& callable) {
    if (handle == nullptr) {
        g_result_buffer = "invalid controller handle";
        return EROB_MOVE_STATUS_REJECTED;
    }
    try {
        return ConvertMoveStatus(callable());
    } catch (const std::exception& error) {
        g_result_buffer = error.what();
        return EROB_MOVE_STATUS_REJECTED;
    } catch (...) {
        g_result_buffer = "unknown controller exception";
        return EROB_MOVE_STATUS_REJECTED;
    }
}

template <typename Callable>
const char* GuardString(ErobControllerHandle* handle, Callable&& callable, const char* fallback = "[]") {
    if (handle == nullptr) {
        g_result_buffer = "invalid controller handle";
        return g_result_buffer.c_str();
    }
    try {
        g_result_buffer = callable();
    } catch (const std::exception& error) {
        g_result_buffer = error.what();
    } catch (...) {
        g_result_buffer = fallback;
    }
    return g_result_buffer.c_str();
}

void CopyString(char* destination, std::size_t destination_size, const std::string& value) {
    if (destination == nullptr || destination_size == 0) {
        return;
    }
    std::snprintf(destination, destination_size, "%s", value.c_str());
}

std::string JsonEscape(const std::string& value) {
    std::ostringstream stream;
    for (const char ch : value) {
        switch (ch) {
        case '\\':
            stream << "\\\\";
            break;
        case '"':
            stream << "\\\"";
            break;
        case '\n':
            stream << "\\n";
            break;
        case '\r':
            stream << "\\r";
            break;
        case '\t':
            stream << "\\t";
            break;
        default:
            stream << ch;
            break;
        }
    }
    return stream.str();
}

std::string SerializeMotors(const std::vector<erob::MotorIdentity>& motors) {
    std::ostringstream stream;
    stream << '[';
    for (std::size_t index = 0; index < motors.size(); ++index) {
        const erob::MotorIdentity& motor = motors[index];
        if (index != 0) {
            stream << ',';
        }
        stream
            << "{\"adapter_name\":\"" << JsonEscape(motor.adapter_name)
            << "\",\"slave_index\":" << motor.slave_index
            << ",\"alias\":" << motor.alias
            << ",\"eep_man\":" << motor.eep_man
            << ",\"eep_id\":" << motor.eep_id
            << ",\"eep_rev\":" << motor.eep_rev
            << ",\"name\":\"" << JsonEscape(motor.name)
            << "\",\"serial_number\":\"" << JsonEscape(motor.serial_number)
            << "\",\"is_erob_motor\":" << (motor.is_erob_motor ? "true" : "false")
            << '}';
    }
    stream << ']';
    return stream.str();
}

std::string SerializeAdapters(const std::vector<erob::AdapterInfo>& adapters) {
    std::ostringstream stream;
    stream << '[';
    for (std::size_t index = 0; index < adapters.size(); ++index) {
        const erob::AdapterInfo& adapter = adapters[index];
        if (index != 0) {
            stream << ',';
        }
        stream
            << "{\"name\":\"" << JsonEscape(adapter.name)
            << "\",\"description\":\"" << JsonEscape(adapter.description)
            << "\",\"scan_success\":" << (adapter.scan_success ? "true" : "false")
            << ",\"discovered_slave_count\":" << adapter.discovered_slave_count
            << '}';
    }
    stream << ']';
    return stream.str();
}

std::string SerializeBindingReports(const std::vector<erob::AxisBindingReport>& reports) {
    std::ostringstream stream;
    stream << '[';
    for (std::size_t index = 0; index < reports.size(); ++index) {
        const erob::AxisBindingReport& report = reports[index];
        if (index != 0) {
            stream << ',';
        }
        stream
            << "{\"logical_axis_id\":" << report.logical_axis_id
            << ",\"joint_name\":\"" << JsonEscape(report.joint_name)
            << "\",\"configured_identity\":" << (report.configured_identity ? "true" : "false")
            << ",\"bound\":" << (report.bound ? "true" : "false")
            << ",\"detail\":\"" << JsonEscape(report.detail)
            << "\",\"configured_serial\":\"" << JsonEscape(report.configured_motor.serial_number)
            << "\",\"discovered_adapter\":\"" << JsonEscape(report.discovered_motor.adapter_name)
            << "\",\"discovered_slave_index\":" << report.discovered_motor.slave_index
            << ",\"discovered_serial\":\"" << JsonEscape(report.discovered_motor.serial_number)
            << "\"}";
    }
    stream << ']';
    return stream.str();
}

void FillAxisStateValue(const erob::AxisState& state, ErobAxisStateValue* out_state) {
    std::memset(out_state, 0, sizeof(*out_state));
    out_state->online = state.online ? 1 : 0;
    out_state->enabled = state.enabled ? 1 : 0;
    out_state->fault = state.fault ? 1 : 0;
    out_state->statusword = state.statusword;
    out_state->controlword = state.controlword;
    out_state->motion_mode = static_cast<int>(state.motion_mode);
    out_state->cia402_state = static_cast<int>(state.cia402_state);
    out_state->position_mode_state = static_cast<int>(state.position_mode_state);
    out_state->follow_mode_state = static_cast<int>(state.follow_mode_state);
    out_state->actual_position_count = state.actual_position_count;
    out_state->actual_velocity_count_s = state.actual_velocity_count_s;
    out_state->actual_torque = state.actual_torque;
    out_state->actual_angle_deg = state.actual_angle_deg;
    out_state->actual_velocity_deg_s = state.actual_velocity_deg_s;
    out_state->target_angle_deg = state.target_angle_deg;
    out_state->position_error_deg = state.position_error_deg;
    out_state->target_reached = state.target_reached ? 1 : 0;
    out_state->near_positive_limit = state.near_positive_limit ? 1 : 0;
    out_state->near_negative_limit = state.near_negative_limit ? 1 : 0;
    out_state->al_status_code = state.al_status_code;
    out_state->last_error_code = state.last_error_code;
    CopyString(out_state->motion_mode_name, sizeof(out_state->motion_mode_name), erob::MotionModeName(state.motion_mode));
    CopyString(out_state->cia402_state_name, sizeof(out_state->cia402_state_name), erob::CiA402StateName(state.cia402_state));
    CopyString(out_state->position_mode_name, sizeof(out_state->position_mode_name), erob::PositionModeStateName(state.position_mode_state));
    CopyString(out_state->follow_mode_name, sizeof(out_state->follow_mode_name), erob::FollowModeStateName(state.follow_mode_state));
}

}  // namespace

extern "C" {

ErobControllerHandle* erob_controller_create(const char* config_path) {
    try {
        return new ErobControllerHandle(config_path != nullptr ? config_path : "config/erob_arm.yaml");
    } catch (...) {
        g_result_buffer = "failed to create controller";
        return nullptr;
    }
}

void erob_controller_destroy(ErobControllerHandle* handle) {
    delete handle;
}

int erob_controller_initialize(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.initialize(); });
}

int erob_controller_shutdown(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.shutdown(); });
}

int erob_controller_scan_and_initialize(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.scanAndInitialize(); });
}

const char* erob_controller_recover_bus_and_rescan_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        if (!handle->controller.recoverBusAndRescan()) {
            return SerializeMotors(std::vector<erob::MotorIdentity>{});
        }
        return SerializeMotors(handle->controller.discoveredMotors());
    });
}

int erob_controller_enable_axis(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.enableAxis(axis_id); });
}

int erob_controller_disable_axis(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.disableAxis(axis_id); });
}

int erob_controller_reset_fault(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.resetFault(axis_id); });
}

int erob_controller_enable_all(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.enableAll(); });
}

int erob_controller_disable_all(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.disableAll(); });
}

int erob_controller_quick_stop_axis(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.quickStopAxis(axis_id); });
}

int erob_controller_quick_stop_all(ErobControllerHandle* handle) {
    return GuardBool(handle, [&]() { return handle->controller.quickStopAll(); });
}

ErobMoveCommandStatus erob_controller_issue_move_to(
    ErobControllerHandle* handle,
    int axis_id,
    double angle_deg,
    double velocity_deg_s) {
    return GuardMoveStatus(handle, [&]() { return handle->controller.issueMoveTo(axis_id, angle_deg, velocity_deg_s); });
}

ErobMoveCommandStatus erob_controller_move_to(
    ErobControllerHandle* handle,
    int axis_id,
    double angle_deg,
    double velocity_deg_s) {
    return GuardMoveStatus(handle, [&]() { return handle->controller.moveTo(axis_id, angle_deg, velocity_deg_s); });
}

int erob_controller_start_follow(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.startFollowMode(axis_id); });
}

int erob_controller_is_axis_busy(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.isAxisBusy(axis_id); });
}

ErobMoveCommandStatus erob_controller_move_group(
    ErobControllerHandle* handle,
    const ErobAxisMoveRequest* requests,
    int request_count,
    int wait_all,
    int strict_mode) {
    return GuardMoveStatus(handle, [&]() {
        if (request_count < 0) {
            return erob::MoveCommandStatus::kRejected;
        }
        if (request_count > 0 && requests == nullptr) {
            return erob::MoveCommandStatus::kRejected;
        }
        std::vector<erob::AxisMoveRequest> cpp_requests;
        cpp_requests.reserve(static_cast<std::size_t>(request_count));
        for (int index = 0; index < request_count; ++index) {
            cpp_requests.push_back(erob::AxisMoveRequest{
                requests[index].axis_id,
                requests[index].angle_deg,
                requests[index].velocity_deg_s,
            });
        }
        return handle->controller.moveGroup(cpp_requests, wait_all != 0, strict_mode != 0);
    });
}

int erob_controller_update_follow_target(ErobControllerHandle* handle, int axis_id, double angle_deg) {
    return GuardBool(handle, [&]() { return handle->controller.updateFollowTarget(axis_id, angle_deg); });
}

int erob_controller_stop_follow(ErobControllerHandle* handle, int axis_id) {
    return GuardBool(handle, [&]() { return handle->controller.stopFollowMode(axis_id); });
}

int erob_controller_axis_count(ErobControllerHandle* handle) {
    if (handle == nullptr) {
        return 0;
    }
    return static_cast<int>(handle->controller.config().axes.size());
}

int erob_controller_get_axis_metadata(
    ErobControllerHandle* handle,
    int axis_id,
    ErobAxisMetadata* out_metadata) {
    if (handle == nullptr || out_metadata == nullptr) {
        return 0;
    }
    return GuardBool(handle, [&]() {
        const erob::SystemConfig& config = handle->controller.config();
        if (axis_id < 0 || axis_id >= static_cast<int>(config.axes.size())) {
            return false;
        }
        const erob::AxisConfig& axis = config.axes[axis_id];
        std::memset(out_metadata, 0, sizeof(*out_metadata));
        out_metadata->logical_axis_id = axis.logical_axis_id;
        out_metadata->min_angle_deg = axis.min_angle_deg;
        out_metadata->max_angle_deg = axis.max_angle_deg;
        out_metadata->max_velocity_deg_s = axis.max_velocity_deg_s;
        out_metadata->max_accel_deg_s2 = axis.max_accel_deg_s2;
        out_metadata->max_decel_deg_s2 = axis.max_decel_deg_s2;
        out_metadata->configured_eep_man = axis.bound_motor.eep_man;
        out_metadata->configured_eep_id = axis.bound_motor.eep_id;
        out_metadata->configured_eep_rev = axis.bound_motor.eep_rev;
        CopyString(out_metadata->joint_name, sizeof(out_metadata->joint_name), axis.joint_name);
        CopyString(out_metadata->configured_serial, sizeof(out_metadata->configured_serial), axis.bound_motor.serial_number);
        return true;
    });
}

int erob_controller_get_axis_state(
    ErobControllerHandle* handle,
    int axis_id,
    ErobAxisStateValue* out_state) {
    if (handle == nullptr || out_state == nullptr) {
        return 0;
    }
    return GuardBool(handle, [&]() {
        if (axis_id < 0 || axis_id >= erob_controller_axis_count(handle)) {
            return false;
        }
        FillAxisStateValue(handle->controller.getAxisState(axis_id), out_state);
        return true;
    });
}

int erob_controller_is_degraded(ErobControllerHandle* handle) {
    if (handle == nullptr) {
        return 0;
    }
    return handle->controller.isDegraded() ? 1 : 0;
}

void erob_controller_set_preferred_adapter(ErobControllerHandle* handle, const char* adapter_name) {
    if (handle == nullptr) {
        return;
    }
    handle->controller.setPreferredAdapter(adapter_name != nullptr ? adapter_name : "");
}

const char* erob_controller_get_preferred_adapter(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return handle->controller.config().preferred_adapter;
    }, "");
}

const char* erob_controller_last_error(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return handle->controller.lastError();
    }, "");
}

const char* erob_controller_scan_adapters_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeAdapters(handle->controller.scanAdapters());
    });
}

const char* erob_controller_scan_motors_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeMotors(handle->controller.scanAndBind());
    });
}

const char* erob_controller_rescan_current_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeMotors(handle->controller.rescanCurrentAdapter());
    });
}

const char* erob_controller_rescan_all_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeMotors(handle->controller.rescanAllAdapters());
    });
}

const char* erob_controller_get_discovered_motors_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeMotors(handle->controller.discoveredMotors());
    });
}

const char* erob_controller_get_binding_reports_json(ErobControllerHandle* handle) {
    return GuardString(handle, [&]() {
        return SerializeBindingReports(handle->controller.getBindingReports());
    });
}

}  // extern "C"