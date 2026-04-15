#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "erob/erob_arm_controller.h"

namespace {

using SteadyClock = std::chrono::steady_clock;
using SystemClock = std::chrono::system_clock;

std::atomic<bool> g_stop_requested{false};

struct FailureBucket {
    uint64_t count = 0;
    std::string last_error;
};

struct FailureStats {
    std::map<std::string, FailureBucket> interval;
    std::map<std::string, FailureBucket> cumulative;
    uint64_t interval_total = 0;
    uint64_t cumulative_total = 0;
};

struct DemoOptions {
    std::string config_path = "config/erob_arm.yaml";
    std::string axis_selector = "all";
    double position_step_deg = 12.0;
    double velocity_deg_s = 20.0;
    double follow_step_deg = 6.0;
    int switch_cycles = 3;
    int duration_minutes = 10;
    std::string log_path = "erob_mode_switch_long_run.txt";
};

struct DemoContext {
    erob::ErobArmController* controller = nullptr;
    std::vector<int> axis_ids;
    std::map<int, erob::AxisConfig> axis_configs;
    std::map<int, double> reference_angles_deg;
    DemoOptions options;
    FailureStats stats;
    uint64_t completed_rounds = 0;
    uint64_t completed_scenarios = 0;
};

struct AxisScenarioTargets {
    int axis_id = -1;
    double base_target_deg = 0.0;
    double interrupt_first_target_deg = 0.0;
    double interrupt_second_target_deg = 0.0;
    double issued_target_deg = 0.0;
    double single_issue_target_deg = 0.0;
    std::vector<double> follow_targets_deg;
};

void HandleSignal(int) {
    g_stop_requested.store(true, std::memory_order_release);
}

std::string NowString() {
    const auto now = SystemClock::now();
    const std::time_t now_time = SystemClock::to_time_t(now);
    std::tm tm_now{};
    localtime_r(&now_time, &tm_now);
    std::ostringstream stream;
    stream << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    return stream.str();
}

std::string Trim(std::string text) {
    const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char value) {
        return std::isspace(value) != 0;
    });
    const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char value) {
        return std::isspace(value) != 0;
    }).base();
    if (first >= last) {
        return "";
    }
    return std::string(first, last);
}

std::string AxisIdsString(const std::vector<int>& axis_ids) {
    std::ostringstream stream;
    for (std::size_t index = 0; index < axis_ids.size(); ++index) {
        if (index != 0) {
            stream << ',';
        }
        stream << axis_ids[index];
    }
    return stream.str();
}

double ClampTarget(double angle_deg, const erob::AxisConfig& axis_config) {
    return erob::Clamp(angle_deg, axis_config.min_angle_deg, axis_config.max_angle_deg);
}

const erob::AxisConfig* FindAxisConfig(const DemoContext& context, int axis_id) {
    const auto it = context.axis_configs.find(axis_id);
    return it == context.axis_configs.end() ? nullptr : &it->second;
}

double ReferenceAngleDeg(const DemoContext& context, int axis_id) {
    const auto it = context.reference_angles_deg.find(axis_id);
    return it == context.reference_angles_deg.end() ? 0.0 : it->second;
}

std::vector<int> RequestAxisIds(const std::vector<erob::AxisMoveRequest>& requests) {
    std::vector<int> axis_ids;
    axis_ids.reserve(requests.size());
    for (const erob::AxisMoveRequest& request : requests) {
        axis_ids.push_back(request.axis_id);
    }
    return axis_ids;
}

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  erob_mode_switch_demo [config_path] [axis_selector] [position_step_deg] [velocity_deg_s] [follow_step_deg] [switch_cycles] [duration_minutes] [log_path]\n\n"
        << "Defaults:\n"
        << "  config_path=config/erob_arm.yaml\n"
        << "  axis_selector=all (all bound axes; also supports auto or comma list like 0,1,2)\n"
        << "  position_step_deg=12\n"
        << "  velocity_deg_s=20\n"
        << "  follow_step_deg=6\n"
        << "  switch_cycles=3\n"
        << "  duration_minutes=10 (0 means run until interrupted)\n"
        << "  log_path=erob_mode_switch_long_run.txt\n";
}

void PrintDiscoveredMotors(const std::vector<erob::MotorIdentity>& motors) {
    std::cout << "[scan] discovered motors: " << motors.size() << '\n';
    for (const erob::MotorIdentity& motor : motors) {
        std::cout
            << "[scan] adapter=" << motor.adapter_name
            << ", slave=" << motor.slave_index
            << ", name=" << motor.name
            << ", serial=" << motor.serial_number
            << ", erob=" << motor.is_erob_motor
            << '\n';
    }
}

void PrintBindingReports(const erob::ErobArmController& controller) {
    const auto reports = controller.getBindingReports();
    std::cout << "[binding] reports: " << reports.size() << '\n';
    for (const erob::AxisBindingReport& report : reports) {
        std::cout
            << "[binding] axis=" << report.logical_axis_id
            << ", joint=" << report.joint_name
            << ", bound=" << report.bound
            << ", detail=" << report.detail
            << '\n';
    }
}

void PrintAxisState(const erob::ErobArmController& controller, int axis_id, const std::string& stage) {
    const erob::AxisState state = controller.getAxisState(axis_id);
    std::cout
        << '[' << stage << "] axis=" << axis_id
        << ", online=" << state.online
        << ", enabled=" << state.enabled
        << ", fault=" << state.fault
        << ", mode=" << erob::MotionModeName(state.motion_mode)
        << ", cia402=" << erob::CiA402StateName(state.cia402_state)
        << ", position_state=" << erob::PositionModeStateName(state.position_mode_state)
        << ", follow_state=" << erob::FollowModeStateName(state.follow_mode_state)
        << ", actual_deg=" << state.actual_angle_deg
        << ", target_deg=" << state.target_angle_deg
        << ", error_deg=" << state.position_error_deg
        << ", velocity_deg_s=" << state.actual_velocity_deg_s
        << ", target_reached=" << state.target_reached
        << ", statusword=0x" << std::hex << state.statusword << std::dec
        << ", al_status=0x" << std::hex << state.al_status_code << std::dec
        << '\n';
}

void PrintAxisStates(
    const erob::ErobArmController& controller,
    const std::vector<int>& axis_ids,
    const std::string& stage) {
    for (const int axis_id : axis_ids) {
        PrintAxisState(controller, axis_id, stage);
    }
}

std::vector<int> ParseAxisSelector(const std::string& selector) {
    std::vector<int> axis_ids;
    std::stringstream stream(selector);
    std::string token;
    while (std::getline(stream, token, ',')) {
        const std::string trimmed = Trim(token);
        if (trimmed.empty()) {
            continue;
        }
        const int axis_id = std::atoi(trimmed.c_str());
        if (std::find(axis_ids.begin(), axis_ids.end(), axis_id) == axis_ids.end()) {
            axis_ids.push_back(axis_id);
        }
    }
    return axis_ids;
}

std::vector<int> ResolveAxisIds(const erob::ErobArmController& controller, const std::string& axis_selector) {
    const auto reports = controller.getBindingReports();
    std::vector<int> bound_axis_ids;
    for (const erob::AxisBindingReport& report : reports) {
        if (report.bound) {
            bound_axis_ids.push_back(report.logical_axis_id);
        }
    }

    const std::string normalized_selector = Trim(axis_selector);
    if (normalized_selector.empty() || normalized_selector == "all" || normalized_selector == "auto") {
        return bound_axis_ids;
    }

    const std::vector<int> requested_axis_ids = ParseAxisSelector(normalized_selector);
    std::vector<int> resolved_axis_ids;
    for (const int axis_id : requested_axis_ids) {
        if (std::find(bound_axis_ids.begin(), bound_axis_ids.end(), axis_id) != bound_axis_ids.end()) {
            resolved_axis_ids.push_back(axis_id);
        }
    }
    return resolved_axis_ids;
}

void SleepMs(int delay_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

bool StopRequested() {
    return g_stop_requested.load(std::memory_order_acquire);
}

bool IsBlockingMoveSuccess(erob::MoveCommandStatus status) {
    return status == erob::MoveCommandStatus::kCompleted;
}

bool IsNonBlockingMoveSuccess(erob::MoveCommandStatus status) {
    return status == erob::MoveCommandStatus::kIssued;
}

void RecordFailure(FailureStats* stats, const std::string& command_type, const std::string& error) {
    if (stats == nullptr) {
        return;
    }
    FailureBucket& interval_bucket = stats->interval[command_type];
    ++interval_bucket.count;
    interval_bucket.last_error = error;
    FailureBucket& cumulative_bucket = stats->cumulative[command_type];
    ++cumulative_bucket.count;
    cumulative_bucket.last_error = error;
    ++stats->interval_total;
    ++stats->cumulative_total;
}

void AppendMinuteSummary(
    const DemoContext& context,
    const SteadyClock::time_point start_time,
    FailureStats* stats,
    bool force) {
    if (stats == nullptr) {
        return;
    }
    if (!force && stats->interval.empty() && stats->interval_total == 0) {
        return;
    }

    std::ofstream output(context.options.log_path, std::ios::app);
    if (!output.is_open()) {
        std::cerr << "[summary] failed to open log file: " << context.options.log_path << '\n';
        return;
    }

    const auto elapsed_sec =
        std::chrono::duration_cast<std::chrono::seconds>(SteadyClock::now() - start_time).count();
    output << '[' << NowString() << "] elapsed_sec=" << elapsed_sec
           << " axes=" << AxisIdsString(context.axis_ids)
           << " rounds=" << context.completed_rounds
           << " scenarios=" << context.completed_scenarios
           << " interval_failures_total=" << stats->interval_total
           << " cumulative_failures_total=" << stats->cumulative_total
           << '\n';
    if (stats->interval.empty()) {
        output << "  interval_failure none\n";
    } else {
        for (const auto& entry : stats->interval) {
            output << "  interval_failure type=" << entry.first
                   << " count=" << entry.second.count
                   << " last_error=" << entry.second.last_error
                   << '\n';
        }
    }
    if (stats->cumulative.empty()) {
        output << "  cumulative_failure none\n";
    } else {
        for (const auto& entry : stats->cumulative) {
            output << "  cumulative_failure type=" << entry.first
                   << " count=" << entry.second.count
                   << " last_error=" << entry.second.last_error
                   << '\n';
        }
    }
    output << '\n';

    stats->interval.clear();
    stats->interval_total = 0;
}

void MaybeFlushMinuteSummary(
    const DemoContext& context,
    const SteadyClock::time_point start_time,
    FailureStats* stats,
    SteadyClock::time_point* next_flush_time) {
    if (stats == nullptr || next_flush_time == nullptr) {
        return;
    }
    const auto now = SteadyClock::now();
    if (now < *next_flush_time) {
        return;
    }

    AppendMinuteSummary(context, start_time, stats, true);
    while (*next_flush_time <= now) {
        *next_flush_time += std::chrono::minutes(1);
    }
}

int EstimateAxisMoveTimeoutMs(
    const DemoContext& context,
    int axis_id,
    double target_deg,
    double velocity_deg_s) {
    const erob::AxisConfig* axis_config = FindAxisConfig(context, axis_id);
    if (axis_config == nullptr || context.controller == nullptr) {
        return 5000;
    }

    const erob::AxisState state = context.controller->getAxisState(axis_id);
    const double clamped_target_deg = ClampTarget(target_deg, *axis_config);
    const double max_velocity_deg_s = std::max(
        1e-3,
        erob::Clamp(std::fabs(velocity_deg_s), 0.0, axis_config->max_velocity_deg_s));
    const double distance_deg = std::fabs(clamped_target_deg - state.actual_angle_deg);
    const double accel_deg_s2 = std::max(1e-3, axis_config->max_accel_deg_s2);
    const double estimated_motion_sec = distance_deg / max_velocity_deg_s + max_velocity_deg_s / accel_deg_s2;
    const double timeout_sec = erob::Clamp(estimated_motion_sec * 3.0 + 1.0, 3.0, 90.0);
    return static_cast<int>(std::ceil(timeout_sec * 1000.0));
}

int EstimateGroupMoveTimeoutMs(
    const DemoContext& context,
    const std::vector<erob::AxisMoveRequest>& requests) {
    int timeout_ms = 3000;
    for (const erob::AxisMoveRequest& request : requests) {
        timeout_ms = std::max(
            timeout_ms,
            EstimateAxisMoveTimeoutMs(context, request.axis_id, request.angle_deg, request.velocity_deg_s));
    }
    return timeout_ms;
}

bool WaitForAxesProfileSettle(
    DemoContext* context,
    const std::vector<int>& axis_ids,
    const std::string& label,
    int timeout_ms) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    while (SteadyClock::now() < deadline) {
        bool all_settled = true;
        for (const int axis_id : axis_ids) {
            const erob::AxisState state = context->controller->getAxisState(axis_id);
            if (!state.online || state.fault) {
                PrintAxisState(*context->controller, axis_id, label + "-fault-or-offline");
                return false;
            }
            const bool stationary = std::fabs(state.actual_velocity_deg_s) <= 0.5;
            const bool position_settled =
                state.position_mode_state == erob::PositionModeState::kIdle ||
                state.position_mode_state == erob::PositionModeState::kTargetReached ||
                (stationary && std::fabs(state.position_error_deg) <= 0.2);
            const bool follow_idle = state.follow_mode_state == erob::FollowModeState::kIdle;
            const bool not_busy = !context->controller->isAxisBusy(axis_id);
            if (!(position_settled && follow_idle && not_busy)) {
                all_settled = false;
                break;
            }
        }
        if (all_settled) {
            return true;
        }
        SleepMs(20);
    }

    PrintAxisStates(*context->controller, axis_ids, label + "-not-settled");
    return false;
}

bool WaitForAnyAxisVelocityAbove(
    DemoContext* context,
    const std::vector<int>& axis_ids,
    const std::string& label,
    double min_abs_velocity_deg_s,
    int timeout_ms) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    while (SteadyClock::now() < deadline) {
        for (const int axis_id : axis_ids) {
            const erob::AxisState state = context->controller->getAxisState(axis_id);
            if (!state.online || state.fault) {
                PrintAxisState(*context->controller, axis_id, label + "-fault-or-offline");
                return false;
            }
            if (std::fabs(state.actual_velocity_deg_s) >= min_abs_velocity_deg_s) {
                return true;
            }
        }
        SleepMs(20);
    }

    PrintAxisStates(*context->controller, axis_ids, label + "-velocity-timeout");
    return false;
}

bool VerifyAxesReachedRequests(
    DemoContext* context,
    const std::vector<erob::AxisMoveRequest>& requests,
    const std::string& label,
    const std::string& command_type,
    double tolerance_deg) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    for (const erob::AxisMoveRequest& request : requests) {
        const erob::AxisState state = context->controller->getAxisState(request.axis_id);
        const double error_deg = std::fabs(state.actual_angle_deg - request.angle_deg);
        if (error_deg <= tolerance_deg) {
            continue;
        }

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
               << "axis " << request.axis_id
               << " settled at " << state.actual_angle_deg
               << " deg, expected " << request.angle_deg
               << " deg, error=" << error_deg << " deg";
        const std::string error = stream.str();
        std::cerr << '[' << label << "] verify failed: " << error << '\n';
        PrintAxisState(*context->controller, request.axis_id, label + "-verify-failed");
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    return true;
}

void PrintMoveRequests(const std::string& label, const std::vector<erob::AxisMoveRequest>& requests) {
    for (const erob::AxisMoveRequest& request : requests) {
        std::cout << '[' << label << "] axis=" << request.axis_id
                  << ", target_deg=" << request.angle_deg
                  << ", velocity_deg_s=" << request.velocity_deg_s
                  << '\n';
    }
}

bool RunMoveTo(
    DemoContext* context,
    int axis_id,
    const std::string& command_type,
    const std::string& label,
    double target_deg,
    double velocity_deg_s,
    bool count_scenario_completion) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    std::cout << '[' << label << "] axis=" << axis_id
              << ", moveTo target_deg=" << target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';
    const erob::MoveCommandStatus move_status =
        context->controller->moveTo(axis_id, target_deg, velocity_deg_s);
    if (!IsBlockingMoveSuccess(move_status)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] moveTo failed: status="
                  << erob::MoveCommandStatusName(move_status)
                  << ", error=" << error << '\n';
        PrintAxisState(*context->controller, axis_id, label + "-failed");
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    SleepMs(200);
    PrintAxisState(*context->controller, axis_id, label + "-completed");
    if (count_scenario_completion) {
        ++context->completed_scenarios;
    }
    return true;
}

bool RunIssuedMoveTo(
    DemoContext* context,
    int axis_id,
    const std::string& command_type,
    const std::string& label,
    double target_deg,
    double velocity_deg_s,
    bool count_scenario_completion) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    std::cout << '[' << label << "] axis=" << axis_id
              << ", issueMoveTo target_deg=" << target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';
    const erob::MoveCommandStatus move_status =
        context->controller->issueMoveTo(axis_id, target_deg, velocity_deg_s);
    if (!IsNonBlockingMoveSuccess(move_status)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] issueMoveTo failed: status="
                  << erob::MoveCommandStatusName(move_status)
                  << ", error=" << error << '\n';
        PrintAxisState(*context->controller, axis_id, label + "-failed");
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    const int timeout_ms = EstimateAxisMoveTimeoutMs(*context, axis_id, target_deg, velocity_deg_s);
    if (!WaitForAxesProfileSettle(context, {axis_id}, label, timeout_ms)) {
        std::string error = context->controller->lastError();
        if (error.empty()) {
            error = "issueMoveTo did not settle within timeout";
        }
        std::cerr << '[' << label << "] issueMoveTo wait failed: " << error << '\n';
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    SleepMs(200);
    PrintAxisState(*context->controller, axis_id, label + "-completed");
    if (count_scenario_completion) {
        ++context->completed_scenarios;
    }
    return true;
}

bool RunMoveGroup(
    DemoContext* context,
    const std::string& command_type,
    const std::string& label,
    const std::vector<erob::AxisMoveRequest>& requests,
    bool wait_all,
    bool strict_mode,
    bool count_scenario_completion) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    std::cout << '[' << label << "] moveGroup wait_all=" << wait_all
              << ", strict_mode=" << strict_mode
              << ", axis_count=" << requests.size() << '\n';
    PrintMoveRequests(label, requests);

    const erob::MoveCommandStatus move_status =
        context->controller->moveGroup(requests, wait_all, strict_mode);
    const bool status_ok = wait_all
        ? IsBlockingMoveSuccess(move_status)
        : IsNonBlockingMoveSuccess(move_status);
    if (!status_ok) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] moveGroup failed: status="
                  << erob::MoveCommandStatusName(move_status)
                  << ", error=" << error << '\n';
        PrintAxisStates(*context->controller, RequestAxisIds(requests), label + "-failed");
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    if (!wait_all) {
        const int timeout_ms = EstimateGroupMoveTimeoutMs(*context, requests);
        if (!WaitForAxesProfileSettle(context, RequestAxisIds(requests), label, timeout_ms)) {
            std::string error = context->controller->lastError();
            if (error.empty()) {
                error = "non-blocking moveGroup did not settle within timeout";
            }
            std::cerr << '[' << label << "] moveGroup wait failed: " << error << '\n';
            RecordFailure(&context->stats, command_type, error);
            return false;
        }
    }

    SleepMs(250);
    PrintAxisStates(*context->controller, RequestAxisIds(requests), label + "-completed");
    if (count_scenario_completion) {
        ++context->completed_scenarios;
    }
    return true;
}

bool RunOverriddenNonBlockingMoveGroupScenario(
    DemoContext* context,
    int round_index,
    const std::vector<erob::AxisMoveRequest>& first_requests,
    const std::vector<erob::AxisMoveRequest>& second_requests) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const std::string label =
        "round-" + std::to_string(round_index) + "-group-pp-issued-override";
    std::cout << '[' << label << "] issuing first non-blocking moveGroup" << '\n';
    PrintMoveRequests(label + "-first", first_requests);
    const erob::MoveCommandStatus first_status =
        context->controller->moveGroup(first_requests, false, true);
    if (!IsNonBlockingMoveSuccess(first_status)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] first moveGroup failed: status="
                  << erob::MoveCommandStatusName(first_status)
                  << ", error=" << error << '\n';
        PrintAxisStates(*context->controller, RequestAxisIds(first_requests), label + "-first-failed");
        RecordFailure(&context->stats, "move_group_issue_override_primary", error);
        return false;
    }

    SleepMs(180);

    std::cout << '[' << label << "] issuing override non-blocking moveGroup" << '\n';
    PrintMoveRequests(label + "-second", second_requests);
    const erob::MoveCommandStatus second_status =
        context->controller->moveGroup(second_requests, false, true);
    if (!IsNonBlockingMoveSuccess(second_status)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] override moveGroup failed: status="
                  << erob::MoveCommandStatusName(second_status)
                  << ", error=" << error << '\n';
        PrintAxisStates(*context->controller, RequestAxisIds(second_requests), label + "-second-failed");
        RecordFailure(&context->stats, "move_group_issue_override_secondary", error);
        return false;
    }

    const int timeout_ms = std::max(
        EstimateGroupMoveTimeoutMs(*context, first_requests),
        EstimateGroupMoveTimeoutMs(*context, second_requests));
    if (!WaitForAxesProfileSettle(context, RequestAxisIds(second_requests), label, timeout_ms)) {
        std::string error = context->controller->lastError();
        if (error.empty()) {
            error = "overridden non-blocking moveGroup did not settle within timeout";
        }
        std::cerr << '[' << label << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_override_wait", error);
        return false;
    }
    if (!VerifyAxesReachedRequests(
            context,
            second_requests,
            label,
            "move_group_issue_override_verify",
            0.8)) {
        return false;
    }

    SleepMs(250);
    PrintAxisStates(*context->controller, RequestAxisIds(second_requests), label + "-completed");
    ++context->completed_scenarios;
    return true;
}

bool RunInterruptedProfilePositionScenario(
    DemoContext* context,
    int axis_id,
    int round_index,
    double first_target_deg,
    double second_target_deg,
    double velocity_deg_s) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const std::string label_prefix =
        "round-" + std::to_string(round_index) + "-axis-" + std::to_string(axis_id) + "-pp-interrupt";
    std::cout << '[' << label_prefix << "] first_target_deg=" << first_target_deg
              << ", second_target_deg=" << second_target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';

    auto first_move = std::async(std::launch::async, [context, axis_id, first_target_deg, velocity_deg_s]() {
        return context->controller->moveTo(axis_id, first_target_deg, velocity_deg_s);
    });

    SleepMs(180);
    const erob::MoveCommandStatus second_status =
        context->controller->moveTo(axis_id, second_target_deg, velocity_deg_s);
    const std::string second_error = context->controller->lastError();
    const erob::MoveCommandStatus first_status = first_move.get();
    const std::string first_error = context->controller->lastError();

    const bool first_ok =
        first_status == erob::MoveCommandStatus::kCompleted ||
        first_status == erob::MoveCommandStatus::kSuperseded;
    if (!first_ok) {
        std::cerr << '[' << label_prefix << "] first move failed: status="
                  << erob::MoveCommandStatusName(first_status)
                  << ", error=" << first_error << '\n';
        PrintAxisState(*context->controller, axis_id, label_prefix + "-first-failed");
        RecordFailure(&context->stats, "move_to_interrupt_primary", first_error);
    }
    if (second_status != erob::MoveCommandStatus::kCompleted) {
        std::cerr << '[' << label_prefix << "] second move failed: status="
                  << erob::MoveCommandStatusName(second_status)
                  << ", error=" << second_error << '\n';
        PrintAxisState(*context->controller, axis_id, label_prefix + "-second-failed");
        RecordFailure(&context->stats, "move_to_interrupt_secondary", second_error);
        return false;
    }

    SleepMs(250);
    PrintAxisState(*context->controller, axis_id, label_prefix + "-completed");
    ++context->completed_scenarios;
    return true;
}

void BestEffortStopFollow(DemoContext* context, const std::vector<int>& axis_ids) {
    if (context == nullptr || context->controller == nullptr) {
        return;
    }
    for (const int axis_id : axis_ids) {
        context->controller->stopFollowMode(axis_id);
    }
}

bool StartFollowOnAxes(DemoContext* context, const std::vector<int>& axis_ids, const std::string& label_prefix) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    constexpr int kFollowStartAttempts = 6;
    for (int attempt = 1; attempt <= kFollowStartAttempts; ++attempt) {
        bool all_started = true;
        for (const int axis_id : axis_ids) {
            if (context->controller->startFollowMode(axis_id)) {
                continue;
            }

            const std::string error = context->controller->lastError();
            if (error.find("axis is still moving in profile position mode") != std::string::npos) {
                all_started = false;
                continue;
            }

            std::cerr << '[' << label_prefix << "] axis=" << axis_id
                      << " start failed: " << error << '\n';
            PrintAxisState(*context->controller, axis_id, label_prefix + "-start-failed");
            RecordFailure(&context->stats, "follow_start", error);
            BestEffortStopFollow(context, axis_ids);
            return false;
        }

        if (all_started) {
            return true;
        }

        std::cout << '[' << label_prefix << "] start delayed by PP settle, attempt=" << attempt << '\n';
        SleepMs(120);
    }

    const std::string error = "follow start still blocked after retries";
    std::cerr << '[' << label_prefix << "] start failed: " << error << '\n';
    RecordFailure(&context->stats, "follow_start", error);
    BestEffortStopFollow(context, axis_ids);
    return false;
}

bool StopFollowOnAxes(DemoContext* context, const std::vector<int>& axis_ids, const std::string& label_prefix) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    bool ok = true;
    for (const int axis_id : axis_ids) {
        if (context->controller->stopFollowMode(axis_id)) {
            continue;
        }
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label_prefix << "] axis=" << axis_id
                  << " stop failed: " << error << '\n';
        PrintAxisState(*context->controller, axis_id, label_prefix + "-stop-failed");
        RecordFailure(&context->stats, "follow_stop", error);
        ok = false;
    }
    return ok;
}

bool RunFollowScenario(
    DemoContext* context,
    int round_index,
    const std::vector<AxisScenarioTargets>& targets,
    int segment_duration_ms,
    int update_period_ms) {
    if (context == nullptr || context->controller == nullptr || targets.empty()) {
        return false;
    }

    const std::string label_prefix = "round-" + std::to_string(round_index) + "-follow-group";
    if (!WaitForAxesProfileSettle(context, context->axis_ids, label_prefix, 2000)) {
        const std::string error = "profile position did not settle before multi-axis follow start";
        std::cerr << '[' << label_prefix << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "follow_precheck", error);
        return false;
    }

    std::cout << '[' << label_prefix << "] starting follow mode on axes="
              << AxisIdsString(context->axis_ids) << '\n';
    if (!StartFollowOnAxes(context, context->axis_ids, label_prefix)) {
        return false;
    }

    SleepMs(std::max(10, update_period_ms));
    PrintAxisStates(*context->controller, context->axis_ids, label_prefix + "-started");

    std::vector<double> last_targets_deg;
    last_targets_deg.reserve(targets.size());
    for (const AxisScenarioTargets& target : targets) {
        last_targets_deg.push_back(context->controller->getAxisState(target.axis_id).actual_angle_deg);
    }

    const std::size_t segment_count = targets.front().follow_targets_deg.size();
    for (std::size_t segment_index = 0; segment_index < segment_count; ++segment_index) {
        const int steps = std::max(1, segment_duration_ms / std::max(1, update_period_ms));
        for (int step = 1; step <= steps; ++step) {
            if (StopRequested()) {
                BestEffortStopFollow(context, context->axis_ids);
                return false;
            }
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            for (std::size_t axis_index = 0; axis_index < targets.size(); ++axis_index) {
                const double target_deg = targets[axis_index].follow_targets_deg[segment_index];
                const double streamed_target_deg =
                    last_targets_deg[axis_index] + ratio * (target_deg - last_targets_deg[axis_index]);
                if (!context->controller->updateFollowTarget(targets[axis_index].axis_id, streamed_target_deg)) {
                    const std::string error = context->controller->lastError();
                    std::cerr << '[' << label_prefix << "] axis=" << targets[axis_index].axis_id
                              << " update failed: " << error << '\n';
                    PrintAxisState(
                        *context->controller,
                        targets[axis_index].axis_id,
                        label_prefix + "-update-failed");
                    RecordFailure(&context->stats, "follow_update", error);
                    BestEffortStopFollow(context, context->axis_ids);
                    return false;
                }
            }
            SleepMs(update_period_ms);
        }

        PrintAxisStates(
            *context->controller,
            context->axis_ids,
            label_prefix + "-segment-" + std::to_string(static_cast<unsigned long long>(segment_index)));
        for (std::size_t axis_index = 0; axis_index < targets.size(); ++axis_index) {
            last_targets_deg[axis_index] = targets[axis_index].follow_targets_deg[segment_index];
        }
    }

    if (!StopFollowOnAxes(context, context->axis_ids, label_prefix)) {
        return false;
    }

    SleepMs(400);
    PrintAxisStates(*context->controller, context->axis_ids, label_prefix + "-stopped");
    ++context->completed_scenarios;
    return true;
}

bool RunFollowMovingRejectedMoveGroupScenario(
    DemoContext* context,
    int round_index,
    const std::vector<erob::AxisMoveRequest>& requests) {
    if (context == nullptr || context->controller == nullptr || requests.empty()) {
        return false;
    }

    const std::string label =
        "round-" + std::to_string(round_index) + "-follow-moving-group-issued";
    if (!WaitForAxesProfileSettle(context, context->axis_ids, label, 2000)) {
        const std::string error = "axes did not settle before follow-moving rejection test";
        std::cerr << '[' << label << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_follow_moving_precheck", error);
        return false;
    }
    if (!StartFollowOnAxes(context, context->axis_ids, label)) {
        return false;
    }

    for (std::size_t axis_index = 0; axis_index < context->axis_ids.size(); ++axis_index) {
        const int axis_id = context->axis_ids[axis_index];
        const erob::AxisConfig* axis_config = FindAxisConfig(*context, axis_id);
        if (axis_config == nullptr) {
            BestEffortStopFollow(context, context->axis_ids);
            RecordFailure(&context->stats, "move_group_issue_follow_moving_target", "missing axis config");
            return false;
        }

        const erob::AxisState state = context->controller->getAxisState(axis_id);
        const double direction = ((round_index + static_cast<int>(axis_index)) % 2 == 0) ? 1.0 : -1.0;
        const double follow_target_deg = ClampTarget(
            state.actual_angle_deg + direction * context->options.follow_step_deg * (1.6 + 0.10 * axis_index),
            *axis_config);
        if (!context->controller->updateFollowTarget(axis_id, follow_target_deg)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << label << "] axis=" << axis_id
                      << " update failed: " << error << '\n';
            PrintAxisState(*context->controller, axis_id, label + "-update-failed");
            RecordFailure(&context->stats, "move_group_issue_follow_moving_target", error);
            BestEffortStopFollow(context, context->axis_ids);
            return false;
        }
    }

    if (!WaitForAnyAxisVelocityAbove(context, context->axis_ids, label, 1.0, 1200)) {
        const std::string error = "follow mode did not reach moving state before rejection test";
        std::cerr << '[' << label << "] motion wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_follow_moving_wait", error);
        BestEffortStopFollow(context, context->axis_ids);
        return false;
    }

    const erob::MoveCommandStatus move_status =
        context->controller->moveGroup(requests, false, true);
    if (move_status != erob::MoveCommandStatus::kRejected) {
        const std::string error =
            "expected rejected while follow was moving, got status=" +
            std::string(erob::MoveCommandStatusName(move_status));
        std::cerr << '[' << label << "] moveGroup status mismatch: " << error << '\n';
        PrintAxisStates(*context->controller, RequestAxisIds(requests), label + "-unexpected-status");
        RecordFailure(&context->stats, "move_group_issue_follow_moving_status", error);
        BestEffortStopFollow(context, context->axis_ids);
        return false;
    }

    std::cout << '[' << label << "] moveGroup rejected as expected: "
              << context->controller->lastError() << '\n';
    if (!StopFollowOnAxes(context, context->axis_ids, label)) {
        return false;
    }
    if (!WaitForAxesProfileSettle(context, context->axis_ids, label + "-stopped", 2500)) {
        const std::string error = "follow mode did not settle after rejection scenario";
        std::cerr << '[' << label << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_follow_moving_stop", error);
        return false;
    }

    SleepMs(200);
    PrintAxisStates(*context->controller, context->axis_ids, label + "-completed");
    ++context->completed_scenarios;
    return true;
}

bool RunFollowIdleIssuedMoveGroupScenario(
    DemoContext* context,
    int round_index,
    const std::vector<erob::AxisMoveRequest>& requests) {
    if (context == nullptr || context->controller == nullptr || requests.empty()) {
        return false;
    }

    const std::string label =
        "round-" + std::to_string(round_index) + "-follow-idle-group-issued";
    if (!WaitForAxesProfileSettle(context, context->axis_ids, label, 2000)) {
        const std::string error = "axes did not settle before follow-idle issue test";
        std::cerr << '[' << label << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_follow_idle_precheck", error);
        return false;
    }
    if (!StartFollowOnAxes(context, context->axis_ids, label)) {
        return false;
    }

    SleepMs(180);
    const erob::MoveCommandStatus move_status =
        context->controller->moveGroup(requests, false, true);
    if (!IsNonBlockingMoveSuccess(move_status)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] moveGroup failed: status="
                  << erob::MoveCommandStatusName(move_status)
                  << ", error=" << error << '\n';
        PrintAxisStates(*context->controller, RequestAxisIds(requests), label + "-failed");
        RecordFailure(&context->stats, "move_group_issue_follow_idle_status", error);
        BestEffortStopFollow(context, context->axis_ids);
        return false;
    }

    const int timeout_ms = EstimateGroupMoveTimeoutMs(*context, requests);
    if (!WaitForAxesProfileSettle(context, RequestAxisIds(requests), label, timeout_ms)) {
        std::string error = context->controller->lastError();
        if (error.empty()) {
            error = "follow-idle non-blocking moveGroup did not settle within timeout";
        }
        std::cerr << '[' << label << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "move_group_issue_follow_idle_wait", error);
        return false;
    }
    if (!VerifyAxesReachedRequests(
            context,
            requests,
            label,
            "move_group_issue_follow_idle_verify",
            0.8)) {
        return false;
    }

    SleepMs(250);
    PrintAxisStates(*context->controller, RequestAxisIds(requests), label + "-completed");
    ++context->completed_scenarios;
    return true;
}

bool EnsureAxisReady(DemoContext* context, int axis_id, const std::string& stage) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const erob::AxisState state = context->controller->getAxisState(axis_id);
    if (state.fault) {
        std::cout << '[' << stage << "] axis=" << axis_id << " fault detected, resetting" << '\n';
        if (!context->controller->resetFault(axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << stage << "] axis=" << axis_id << " reset fault failed: " << error << '\n';
            RecordFailure(&context->stats, "reset_fault", error);
            return false;
        }
        SleepMs(300);
        PrintAxisState(*context->controller, axis_id, stage + "-reset");
    }

    const erob::AxisState after_reset_state = context->controller->getAxisState(axis_id);
    if (!after_reset_state.enabled) {
        std::cout << '[' << stage << "] axis=" << axis_id << " enabling" << '\n';
        if (!context->controller->enableAxis(axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << stage << "] axis=" << axis_id << " enable failed: " << error << '\n';
            RecordFailure(&context->stats, "enable_axis", error);
            return false;
        }
        SleepMs(400);
        PrintAxisState(*context->controller, axis_id, stage + "-enabled");
    }

    return true;
}

bool EnsureAxesReady(DemoContext* context, const std::string& stage) {
    if (context == nullptr) {
        return false;
    }
    for (const int axis_id : context->axis_ids) {
        if (!EnsureAxisReady(context, axis_id, stage)) {
            return false;
        }
    }
    return true;
}

bool RunEnableDisableCycle(
    DemoContext* context,
    int round_index,
    const std::vector<int>& axis_ids,
    const std::string& label_suffix) {
    if (context == nullptr || context->controller == nullptr || axis_ids.empty()) {
        return false;
    }

    const std::string label =
        "round-" + std::to_string(round_index) + "-power-cycle-" + label_suffix;
    std::cout << '[' << label << "] disabling axes=" << AxisIdsString(axis_ids) << '\n';
    for (const int axis_id : axis_ids) {
        if (!context->controller->disableAxis(axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << label << "] axis=" << axis_id << " disable failed: " << error << '\n';
            RecordFailure(&context->stats, "disable_axis", error);
            return false;
        }
    }

    SleepMs(300);
    PrintAxisStates(*context->controller, axis_ids, label + "-disabled");

    std::cout << '[' << label << "] re-enabling axes=" << AxisIdsString(axis_ids) << '\n';
    for (const int axis_id : axis_ids) {
        if (!context->controller->enableAxis(axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << label << "] axis=" << axis_id << " re-enable failed: " << error << '\n';
            RecordFailure(&context->stats, "re_enable_axis", error);
            return false;
        }
    }

    SleepMs(400);
    PrintAxisStates(*context->controller, axis_ids, label + "-re-enabled");
    ++context->completed_scenarios;
    return true;
}

std::vector<AxisScenarioTargets> BuildRoundTargets(DemoContext* context, int round_index) {
    std::vector<AxisScenarioTargets> targets;
    if (context == nullptr || context->controller == nullptr) {
        return targets;
    }

    targets.reserve(context->axis_ids.size());
    for (std::size_t axis_index = 0; axis_index < context->axis_ids.size(); ++axis_index) {
        const int axis_id = context->axis_ids[axis_index];
        const erob::AxisConfig* axis_config = FindAxisConfig(*context, axis_id);
        if (axis_config == nullptr) {
            continue;
        }

        const erob::AxisState state = context->controller->getAxisState(axis_id);
        const double direction = ((round_index + static_cast<int>(axis_index)) % 2 == 0) ? 1.0 : -1.0;
        const double position_scale = 1.0 + 0.15 * static_cast<double>(axis_index);
        const double follow_scale = 1.0 + 0.10 * static_cast<double>(axis_index);
        const double position_step_deg = context->options.position_step_deg * position_scale;
        const double follow_step_deg = context->options.follow_step_deg * follow_scale;

        AxisScenarioTargets target;
        target.axis_id = axis_id;
        target.base_target_deg = ClampTarget(
            state.actual_angle_deg + direction * position_step_deg,
            *axis_config);
        target.interrupt_first_target_deg = ClampTarget(
            state.actual_angle_deg + direction * (position_step_deg * 0.55),
            *axis_config);
        target.interrupt_second_target_deg = ClampTarget(
            state.actual_angle_deg + direction * (position_step_deg + follow_step_deg * 0.5),
            *axis_config);
        target.issued_target_deg = ClampTarget(
            target.base_target_deg - direction * (follow_step_deg * 0.70),
            *axis_config);
        target.single_issue_target_deg = ClampTarget(
            ReferenceAngleDeg(*context, axis_id) + direction * (follow_step_deg * 0.60),
            *axis_config);
        target.follow_targets_deg = {
            ClampTarget(target.base_target_deg + direction * follow_step_deg, *axis_config),
            ClampTarget(target.base_target_deg - direction * follow_step_deg, *axis_config),
            target.base_target_deg,
        };
        targets.push_back(target);
    }

    return targets;
}

bool RunRound(DemoContext* context, int round_index) {
    if (context == nullptr || context->controller == nullptr || context->axis_ids.empty()) {
        return false;
    }
    if (!EnsureAxesReady(context, "round-" + std::to_string(round_index) + "-precheck")) {
        return false;
    }

    const std::vector<AxisScenarioTargets> targets = BuildRoundTargets(context, round_index);
    if (targets.size() != context->axis_ids.size()) {
        RecordFailure(&context->stats, "target_build", "failed to build round targets for all selected axes");
        return false;
    }

    std::vector<erob::AxisMoveRequest> base_requests;
    std::vector<erob::AxisMoveRequest> issued_group_requests;
    std::vector<erob::AxisMoveRequest> override_group_requests;
    base_requests.reserve(targets.size());
    issued_group_requests.reserve(targets.size());
    override_group_requests.reserve(targets.size());
    for (const AxisScenarioTargets& target : targets) {
        base_requests.push_back(erob::AxisMoveRequest{
            target.axis_id,
            target.base_target_deg,
            context->options.velocity_deg_s,
        });
        issued_group_requests.push_back(erob::AxisMoveRequest{
            target.axis_id,
            target.issued_target_deg,
            std::max(1.0, context->options.velocity_deg_s * 0.85),
        });
        override_group_requests.push_back(erob::AxisMoveRequest{
            target.axis_id,
            target.interrupt_second_target_deg,
            std::max(1.0, context->options.velocity_deg_s * 0.90),
        });
    }

    if (!RunMoveGroup(
            context,
            "move_group_blocking",
            "round-" + std::to_string(round_index) + "-group-pp-base",
            base_requests,
            true,
            true,
            true)) {
        return false;
    }

    const AxisScenarioTargets& interrupted_target =
        targets[static_cast<std::size_t>(round_index % static_cast<int>(targets.size()))];
    if (!RunInterruptedProfilePositionScenario(
            context,
            interrupted_target.axis_id,
            round_index,
            interrupted_target.interrupt_first_target_deg,
            interrupted_target.interrupt_second_target_deg,
            context->options.velocity_deg_s)) {
        return false;
    }

    if (!RunMoveGroup(
            context,
            "move_group_issue",
            "round-" + std::to_string(round_index) + "-group-pp-issued",
            issued_group_requests,
            false,
            true,
            true)) {
        return false;
    }

    if (!RunOverriddenNonBlockingMoveGroupScenario(
            context,
            round_index,
            base_requests,
            override_group_requests)) {
        return false;
    }

    const AxisScenarioTargets& issue_target =
        targets[static_cast<std::size_t>((round_index + 1) % static_cast<int>(targets.size()))];
    if (!RunIssuedMoveTo(
            context,
            issue_target.axis_id,
            "issue_move_to",
            "round-" + std::to_string(round_index) + "-axis-" + std::to_string(issue_target.axis_id) + "-issue",
            issue_target.single_issue_target_deg,
            std::max(1.0, context->options.velocity_deg_s * 0.75),
            true)) {
        return false;
    }

    if (!RunFollowMovingRejectedMoveGroupScenario(context, round_index, base_requests)) {
        return false;
    }

    if (!RunFollowIdleIssuedMoveGroupScenario(context, round_index, issued_group_requests)) {
        return false;
    }

    if (!RunFollowScenario(context, round_index, targets, 360, 20)) {
        return false;
    }

    if (round_index > 0 && (round_index % 4) == 0) {
        const int disturbed_axis =
            context->axis_ids[static_cast<std::size_t>(round_index % static_cast<int>(context->axis_ids.size()))];
        if (!RunEnableDisableCycle(
                context,
                round_index,
                {disturbed_axis},
                "axis-" + std::to_string(disturbed_axis))) {
            return false;
        }
    }

    if (context->axis_ids.size() > 1 && round_index > 0 && (round_index % 8) == 0) {
        if (!RunEnableDisableCycle(context, round_index, context->axis_ids, "all")) {
            return false;
        }
    }

    if ((round_index % std::max(1, context->options.switch_cycles)) ==
        (std::max(1, context->options.switch_cycles) - 1)) {
        std::vector<erob::AxisMoveRequest> return_home_requests;
        return_home_requests.reserve(context->axis_ids.size());
        for (const int axis_id : context->axis_ids) {
            return_home_requests.push_back(erob::AxisMoveRequest{
                axis_id,
                ReferenceAngleDeg(*context, axis_id),
                context->options.velocity_deg_s,
            });
        }
        if (!RunMoveGroup(
                context,
                "move_group_return",
                "round-" + std::to_string(round_index) + "-group-return-home",
                return_home_requests,
                true,
                true,
                true)) {
            return false;
        }
    }

    ++context->completed_rounds;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    DemoOptions options;
    int arg_index = 1;
    if (argc > 1 && std::string(argv[1]).find(".yaml") != std::string::npos) {
        options.config_path = argv[1];
        arg_index = 2;
    }

    if (argc > arg_index) {
        options.axis_selector = argv[arg_index++];
    }
    if (argc > arg_index) {
        options.position_step_deg = std::atof(argv[arg_index++]);
    }
    if (argc > arg_index) {
        options.velocity_deg_s = std::atof(argv[arg_index++]);
    }
    if (argc > arg_index) {
        options.follow_step_deg = std::atof(argv[arg_index++]);
    }
    if (argc > arg_index) {
        options.switch_cycles = std::atoi(argv[arg_index++]);
    }
    if (argc > arg_index) {
        options.duration_minutes = std::atoi(argv[arg_index++]);
    }
    if (argc > arg_index) {
        options.log_path = argv[arg_index++];
    }

    if (options.velocity_deg_s <= 0.0 || options.switch_cycles <= 0 || options.duration_minutes < 0) {
        PrintUsage();
        return 1;
    }

    erob::ErobArmController controller(options.config_path);
    DemoContext context;
    context.controller = &controller;
    context.options = options;

    const auto motors = controller.scanAndBind();
    if (motors.empty()) {
        const std::string error = controller.lastError();
        std::cerr << "[scan] failed or no motors discovered: " << error << '\n';
        RecordFailure(&context.stats, "scan_and_bind", error);
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        return 1;
    }
    PrintDiscoveredMotors(motors);
    PrintBindingReports(controller);

    const std::vector<int> axis_ids = ResolveAxisIds(controller, options.axis_selector);
    if (axis_ids.empty()) {
        std::cerr << "[select-axis] no bound axis available for selector: " << options.axis_selector << '\n';
        RecordFailure(&context.stats, "select_axis", "no bound axis available for selector");
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        return 1;
    }
    context.axis_ids = axis_ids;

    if (!controller.initialize()) {
        const std::string error = controller.lastError();
        std::cerr << "[initialize] failed: " << error << '\n';
        RecordFailure(&context.stats, "initialize", error);
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        return 1;
    }

    SleepMs(300);
    PrintAxisStates(controller, context.axis_ids, "initialized");

    if (!EnsureAxesReady(&context, "startup")) {
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        controller.shutdown();
        return 1;
    }

    for (const int axis_id : context.axis_ids) {
        context.axis_configs[axis_id] = controller.config().axes.at(static_cast<std::size_t>(axis_id));
        context.reference_angles_deg[axis_id] = controller.getAxisState(axis_id).actual_angle_deg;
    }

    std::cout
        << "[test-plan] axes=" << AxisIdsString(context.axis_ids)
        << ", position_step_deg=" << options.position_step_deg
        << ", follow_step_deg=" << options.follow_step_deg
        << ", velocity_deg_s=" << options.velocity_deg_s
        << ", switch_cycles=" << options.switch_cycles
        << ", duration_minutes=" << options.duration_minutes
        << ", log_path=" << options.log_path
        << '\n';
    for (const int axis_id : context.axis_ids) {
        std::cout << "[test-plan] axis=" << axis_id
                  << ", reference_deg=" << ReferenceAngleDeg(context, axis_id)
                  << '\n';
    }

    const auto start_time = SteadyClock::now();
    auto next_flush_time = start_time + std::chrono::minutes(1);
    const auto deadline = options.duration_minutes == 0
        ? SteadyClock::time_point::max()
        : start_time + std::chrono::minutes(options.duration_minutes);

    bool ok = true;
    int round_index = 0;
    while (!StopRequested() && SteadyClock::now() < deadline) {
        const bool round_ok = RunRound(&context, round_index);
        MaybeFlushMinuteSummary(context, start_time, &context.stats, &next_flush_time);
        if (!round_ok) {
            std::cerr << "[round] round=" << round_index
                      << " failed, attempting recovery on next iteration" << '\n';
            SleepMs(500);
        }
        ok = ok && round_ok;
        ++round_index;
    }

    for (const int axis_id : context.axis_ids) {
        if (!controller.disableAxis(axis_id)) {
            const std::string error = controller.lastError();
            std::cerr << "[disable] axis=" << axis_id << " warning: " << error << '\n';
            RecordFailure(&context.stats, "disable_axis", error);
        }
    }
    SleepMs(250);
    PrintAxisStates(
        controller,
        context.axis_ids,
        StopRequested() ? "stopped" : (ok ? "completed" : "aborted"));

    AppendMinuteSummary(context, start_time, &context.stats, true);
    controller.shutdown();
    return ok ? 0 : 1;
}