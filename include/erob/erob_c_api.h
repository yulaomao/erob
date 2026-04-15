#ifndef EROB_C_API_H_
#define EROB_C_API_H_

#include <stdint.h>

#ifdef _WIN32
#ifdef EROB_C_API_BUILD
#define EROB_C_API_EXPORT __declspec(dllexport)
#else
#define EROB_C_API_EXPORT __declspec(dllimport)
#endif
#else
#define EROB_C_API_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum {
    EROB_TEXT_SMALL = 32,
    EROB_TEXT_MEDIUM = 64,
    EROB_TEXT_LARGE = 256,
};

typedef struct ErobControllerHandle ErobControllerHandle;

typedef struct ErobAxisMetadata {
    int logical_axis_id;
    char joint_name[EROB_TEXT_MEDIUM];
    double min_angle_deg;
    double max_angle_deg;
    double max_velocity_deg_s;
    double max_accel_deg_s2;
    double max_decel_deg_s2;
    char configured_serial[EROB_TEXT_MEDIUM];
    uint32_t configured_eep_man;
    uint32_t configured_eep_id;
    uint32_t configured_eep_rev;
} ErobAxisMetadata;

typedef struct ErobAxisStateValue {
    int online;
    int enabled;
    int fault;
    uint16_t statusword;
    uint16_t controlword;
    int motion_mode;
    int cia402_state;
    int position_mode_state;
    int follow_mode_state;
    int32_t actual_position_count;
    int32_t actual_velocity_count_s;
    int16_t actual_torque;
    double actual_angle_deg;
    double actual_velocity_deg_s;
    double target_angle_deg;
    double position_error_deg;
    int target_reached;
    int near_positive_limit;
    int near_negative_limit;
    int al_status_code;
    int last_error_code;
    char motion_mode_name[EROB_TEXT_SMALL];
    char cia402_state_name[EROB_TEXT_MEDIUM];
    char position_mode_name[EROB_TEXT_MEDIUM];
    char follow_mode_name[EROB_TEXT_MEDIUM];
    double actual_accel_deg_s2;
    double actual_jerk_deg_s3;
    double follow_reference_angle_deg;
    double follow_reference_velocity_deg_s;
    double follow_reference_accel_deg_s2;
    double follow_output_velocity_deg_s;
    double follow_output_accel_deg_s2;
} ErobAxisStateValue;

typedef struct ErobAxisMoveRequest {
    int axis_id;
    double angle_deg;
    double velocity_deg_s;
} ErobAxisMoveRequest;

typedef enum ErobMoveCommandStatus {
    EROB_MOVE_STATUS_COMPLETED = 0,
    EROB_MOVE_STATUS_ISSUED = 1,
    EROB_MOVE_STATUS_SUPERSEDED = 2,
    EROB_MOVE_STATUS_REJECTED = 3,
    EROB_MOVE_STATUS_TIMED_OUT = 4,
    EROB_MOVE_STATUS_INTERRUPTED = 5,
} ErobMoveCommandStatus;

EROB_C_API_EXPORT ErobControllerHandle* erob_controller_create(const char* config_path);
EROB_C_API_EXPORT void erob_controller_destroy(ErobControllerHandle* handle);

EROB_C_API_EXPORT int erob_controller_initialize(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_shutdown(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_scan_and_initialize(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_recover_bus_and_rescan_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_enable_axis(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT int erob_controller_disable_axis(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT int erob_controller_reset_fault(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT int erob_controller_enable_all(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_disable_all(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_quick_stop_axis(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT int erob_controller_quick_stop_all(ErobControllerHandle* handle);
EROB_C_API_EXPORT ErobMoveCommandStatus erob_controller_issue_move_to(
    ErobControllerHandle* handle,
    int axis_id,
    double angle_deg,
    double velocity_deg_s);
EROB_C_API_EXPORT ErobMoveCommandStatus erob_controller_move_to(
    ErobControllerHandle* handle,
    int axis_id,
    double angle_deg,
    double velocity_deg_s);
EROB_C_API_EXPORT int erob_controller_is_axis_busy(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT ErobMoveCommandStatus erob_controller_move_group(
    ErobControllerHandle* handle,
    const ErobAxisMoveRequest* requests,
    int request_count,
    int wait_all,
    int strict_mode);
EROB_C_API_EXPORT int erob_controller_start_follow(ErobControllerHandle* handle, int axis_id);
EROB_C_API_EXPORT int erob_controller_update_follow_target(
    ErobControllerHandle* handle,
    int axis_id,
    double angle_deg);
EROB_C_API_EXPORT int erob_controller_stop_follow(ErobControllerHandle* handle, int axis_id);

EROB_C_API_EXPORT int erob_controller_axis_count(ErobControllerHandle* handle);
EROB_C_API_EXPORT int erob_controller_get_axis_metadata(
    ErobControllerHandle* handle,
    int axis_id,
    ErobAxisMetadata* out_metadata);
EROB_C_API_EXPORT int erob_controller_get_axis_state(
    ErobControllerHandle* handle,
    int axis_id,
    ErobAxisStateValue* out_state);
EROB_C_API_EXPORT int erob_controller_is_degraded(ErobControllerHandle* handle);
EROB_C_API_EXPORT void erob_controller_set_preferred_adapter(
    ErobControllerHandle* handle,
    const char* adapter_name);
EROB_C_API_EXPORT const char* erob_controller_get_preferred_adapter(ErobControllerHandle* handle);

EROB_C_API_EXPORT const char* erob_controller_last_error(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_scan_adapters_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_scan_motors_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_rescan_current_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_rescan_all_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_get_discovered_motors_json(ErobControllerHandle* handle);
EROB_C_API_EXPORT const char* erob_controller_get_binding_reports_json(ErobControllerHandle* handle);

#ifdef __cplusplus
}
#endif

#endif