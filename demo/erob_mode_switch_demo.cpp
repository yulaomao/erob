#include <atomic>
#include <chrono>
#include <cmath>
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
    int requested_axis_id = -1;
    double position_step_deg = 12.0;
    double velocity_deg_s = 20.0;
    double follow_step_deg = 6.0;
    int switch_cycles = 3;
    int duration_minutes = 10;
    std::string log_path = "erob_mode_switch_long_run.txt";
};

struct DemoContext {
    erob::ErobArmController* controller = nullptr;
    int axis_id = -1;
    erob::AxisConfig axis_config;
    double reference_angle_deg = 0.0;
    DemoOptions options;
    FailureStats stats;
    uint64_t completed_rounds = 0;
    uint64_t completed_scenarios = 0;
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

double ClampTarget(double angle_deg, const erob::AxisConfig& axis_config) {
    return erob::Clamp(angle_deg, axis_config.min_angle_deg, axis_config.max_angle_deg);
}

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  erob_mode_switch_demo [config_path] [axis_id|auto] [position_step_deg] [velocity_deg_s] [follow_step_deg] [switch_cycles] [duration_minutes] [log_path]\n\n"
        << "Defaults:\n"
        << "  config_path=config/erob_arm.yaml\n"
        << "  axis_id=auto (first bound axis)\n"
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

int ResolveAxisId(const erob::ErobArmController& controller, int requested_axis_id) {
    const auto reports = controller.getBindingReports();
    if (requested_axis_id >= 0) {
        for (const erob::AxisBindingReport& report : reports) {
            if (report.logical_axis_id == requested_axis_id && report.bound) {
                return requested_axis_id;
            }
        }
        return -1;
    }

    for (const erob::AxisBindingReport& report : reports) {
        if (report.bound) {
            return report.logical_axis_id;
        }
    }
    return -1;
}

void SleepMs(int delay_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

bool StopRequested() {
    return g_stop_requested.load(std::memory_order_acquire);
}

bool IsSuccessfulMoveStatus(erob::MoveCommandStatus status, bool blocking_call) {
    if (blocking_call) {
        return status == erob::MoveCommandStatus::kCompleted;
    }
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

bool RunMoveTo(
    DemoContext* context,
    const std::string& command_type,
    const std::string& label,
    double target_deg,
    double velocity_deg_s,
    bool count_scenario_completion) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    std::cout << '[' << label << "] moveTo target_deg=" << target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';
    const erob::MoveCommandStatus move_status =
        context->controller->moveTo(context->axis_id, target_deg, velocity_deg_s);
    if (!IsSuccessfulMoveStatus(move_status, true)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] moveTo failed: status="
                  << erob::MoveCommandStatusName(move_status)
                  << ", error=" << error << '\n';
        PrintAxisState(*context->controller, context->axis_id, label + "-failed");
        RecordFailure(&context->stats, command_type, error);
        return false;
    }

    SleepMs(300);
    PrintAxisState(*context->controller, context->axis_id, label + "-reached");
    if (count_scenario_completion) {
        ++context->completed_scenarios;
    }
    return true;
}

bool RunInterruptedProfilePositionScenario(
    DemoContext* context,
    int round_index,
    double first_target_deg,
    double second_target_deg,
    double velocity_deg_s) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const std::string label_prefix = "round-" + std::to_string(round_index) + "-pp-interrupt";
    std::cout << '[' << label_prefix << "] first_target_deg=" << first_target_deg
              << ", second_target_deg=" << second_target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';

    auto first_move = std::async(std::launch::async, [context, first_target_deg, velocity_deg_s]() {
        return context->controller->moveTo(context->axis_id, first_target_deg, velocity_deg_s);
    });

    SleepMs(180);
    const erob::MoveCommandStatus second_status =
        context->controller->moveTo(context->axis_id, second_target_deg, velocity_deg_s);
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
        PrintAxisState(*context->controller, context->axis_id, label_prefix + "-first-failed");
        RecordFailure(&context->stats, "move_to_interrupt_primary", first_error);
    }
    if (second_status != erob::MoveCommandStatus::kCompleted) {
        std::cerr << '[' << label_prefix << "] second move failed: status="
                  << erob::MoveCommandStatusName(second_status)
                  << ", error=" << second_error << '\n';
        PrintAxisState(*context->controller, context->axis_id, label_prefix + "-second-failed");
        RecordFailure(&context->stats, "move_to_interrupt_secondary", second_error);
        return false;
    }

    SleepMs(300);
    PrintAxisState(*context->controller, context->axis_id, label_prefix + "-completed");
    ++context->completed_scenarios;
    return true;
}

bool WaitForProfilePositionSettle(
    DemoContext* context,
    const std::string& label,
    int timeout_ms) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    while (SteadyClock::now() < deadline) {
        const erob::AxisState state = context->controller->getAxisState(context->axis_id);
        const bool stationary = std::fabs(state.actual_velocity_deg_s) <= 0.5;
        const bool settled =
            state.position_mode_state == erob::PositionModeState::kIdle ||
            state.position_mode_state == erob::PositionModeState::kTargetReached ||
            (stationary && std::fabs(state.position_error_deg) <= 0.2);
        if (settled) {
            return true;
        }
        SleepMs(20);
    }

    PrintAxisState(*context->controller, context->axis_id, label + "-pp-not-settled");
    return false;
}

bool RunFollowScenario(
    DemoContext* context,
    int round_index,
    const std::vector<double>& targets_deg,
    int segment_duration_ms,
    int update_period_ms) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const std::string label_prefix = "round-" + std::to_string(round_index) + "-follow";
    if (!WaitForProfilePositionSettle(context, label_prefix, 1500)) {
        const std::string error = "profile position did not settle before follow start";
        std::cerr << '[' << label_prefix << "] settle wait failed: " << error << '\n';
        RecordFailure(&context->stats, "follow_precheck", error);
        return false;
    }

    std::cout << '[' << label_prefix << "] starting follow mode" << '\n';
    constexpr int kFollowStartAttempts = 5;
    for (int attempt = 1; attempt <= kFollowStartAttempts; ++attempt) {
        if (context->controller->startFollowMode(context->axis_id)) {
            SleepMs(std::max(10, update_period_ms));
            PrintAxisState(*context->controller, context->axis_id, label_prefix + "-started");

            double last_target_deg = context->controller->getAxisState(context->axis_id).actual_angle_deg;
            for (std::size_t index = 0; index < targets_deg.size(); ++index) {
                const double target_deg = targets_deg[index];
                std::cout << '[' << label_prefix << "] target[" << index << "]=" << target_deg << '\n';
                const int steps = std::max(1, segment_duration_ms / std::max(1, update_period_ms));
                for (int step = 1; step <= steps; ++step) {
                    if (StopRequested()) {
                        context->controller->stopFollowMode(context->axis_id);
                        return false;
                    }
                    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
                    const double streamed_target_deg =
                        last_target_deg + ratio * (target_deg - last_target_deg);
                    if (!context->controller->updateFollowTarget(context->axis_id, streamed_target_deg)) {
                        const std::string error = context->controller->lastError();
                        std::cerr << '[' << label_prefix << "] update failed: " << error << '\n';
                        PrintAxisState(*context->controller, context->axis_id, label_prefix + "-update-failed");
                        RecordFailure(&context->stats, "follow_update", error);
                        context->controller->stopFollowMode(context->axis_id);
                        return false;
                    }
                    SleepMs(update_period_ms);
                }
                PrintAxisState(
                    *context->controller,
                    context->axis_id,
                    label_prefix + "-running-" + std::to_string(static_cast<unsigned long long>(index)));
                last_target_deg = target_deg;
            }

            if (!context->controller->stopFollowMode(context->axis_id)) {
                const std::string error = context->controller->lastError();
                std::cerr << '[' << label_prefix << "] stop failed: " << error << '\n';
                PrintAxisState(*context->controller, context->axis_id, label_prefix + "-stop-failed");
                RecordFailure(&context->stats, "follow_stop", error);
                return false;
            }

            SleepMs(400);
            PrintAxisState(*context->controller, context->axis_id, label_prefix + "-stopped");
            ++context->completed_scenarios;
            return true;
        }

        const std::string error = context->controller->lastError();
        if (error.find("axis is still moving in profile position mode") == std::string::npos) {
            std::cerr << '[' << label_prefix << "] start failed: " << error << '\n';
            PrintAxisState(*context->controller, context->axis_id, label_prefix + "-start-failed");
            RecordFailure(&context->stats, "follow_start", error);
            return false;
        }

        std::cout << '[' << label_prefix << "] start delayed by PP settle, attempt=" << attempt << '\n';
        SleepMs(120);
    }

    const std::string error = "follow start still blocked after retries";
    std::cerr << '[' << label_prefix << "] start failed: " << error << '\n';
    PrintAxisState(*context->controller, context->axis_id, label_prefix + "-start-failed");
    RecordFailure(&context->stats, "follow_start", error);
    return false;
}

bool EnsureAxisReady(DemoContext* context, const std::string& stage) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const erob::AxisState state = context->controller->getAxisState(context->axis_id);
    if (state.fault) {
        std::cout << '[' << stage << "] fault detected, resetting" << '\n';
        if (!context->controller->resetFault(context->axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << stage << "] reset fault failed: " << error << '\n';
            RecordFailure(&context->stats, "reset_fault", error);
            return false;
        }
        SleepMs(300);
        PrintAxisState(*context->controller, context->axis_id, stage + "-reset");
    }

    const erob::AxisState after_reset_state = context->controller->getAxisState(context->axis_id);
    if (!after_reset_state.enabled) {
        std::cout << '[' << stage << "] enabling axis" << '\n';
        if (!context->controller->enableAxis(context->axis_id)) {
            const std::string error = context->controller->lastError();
            std::cerr << '[' << stage << "] enable failed: " << error << '\n';
            RecordFailure(&context->stats, "enable_axis", error);
            return false;
        }
        SleepMs(400);
        PrintAxisState(*context->controller, context->axis_id, stage + "-enabled");
    }

    return true;
}

bool RunEnableDisableCycle(DemoContext* context, int round_index) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }

    const std::string label = "round-" + std::to_string(round_index) + "-power-cycle";
    std::cout << '[' << label << "] disabling axis for disturbance test" << '\n';
    if (!context->controller->disableAxis(context->axis_id)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] disable failed: " << error << '\n';
        RecordFailure(&context->stats, "disable_axis", error);
        return false;
    }
    SleepMs(300);
    PrintAxisState(*context->controller, context->axis_id, label + "-disabled");

    std::cout << '[' << label << "] re-enabling axis" << '\n';
    if (!context->controller->enableAxis(context->axis_id)) {
        const std::string error = context->controller->lastError();
        std::cerr << '[' << label << "] re-enable failed: " << error << '\n';
        RecordFailure(&context->stats, "re_enable_axis", error);
        return false;
    }

    SleepMs(400);
    PrintAxisState(*context->controller, context->axis_id, label + "-re-enabled");
    ++context->completed_scenarios;
    return true;
}

bool RunRound(DemoContext* context, int round_index) {
    if (context == nullptr || context->controller == nullptr) {
        return false;
    }
    if (!EnsureAxisReady(context, "round-" + std::to_string(round_index) + "-precheck")) {
        return false;
    }

    const erob::AxisState state = context->controller->getAxisState(context->axis_id);
    const double current_angle_deg = state.actual_angle_deg;
    const double direction = (round_index % 2 == 0) ? 1.0 : -1.0;
    const double base_target_deg = ClampTarget(
        current_angle_deg + direction * context->options.position_step_deg,
        context->axis_config);
    const double interrupt_first_target_deg = ClampTarget(
        current_angle_deg + direction * (context->options.position_step_deg * 0.6),
        context->axis_config);
    const double interrupt_second_target_deg = ClampTarget(
        current_angle_deg + direction * (context->options.position_step_deg + context->options.follow_step_deg * 0.5),
        context->axis_config);
    const double follow_positive_target_deg = ClampTarget(
        base_target_deg + direction * context->options.follow_step_deg,
        context->axis_config);
    const double follow_negative_target_deg = ClampTarget(
        base_target_deg - direction * context->options.follow_step_deg,
        context->axis_config);

    if (!RunMoveTo(
            context,
            "move_to",
            "round-" + std::to_string(round_index) + "-pp-base",
            base_target_deg,
            context->options.velocity_deg_s,
            true)) {
        return false;
    }

    if (!RunInterruptedProfilePositionScenario(
            context,
            round_index,
            interrupt_first_target_deg,
            interrupt_second_target_deg,
            context->options.velocity_deg_s)) {
        return false;
    }

    if (!RunFollowScenario(
            context,
            round_index,
            {follow_positive_target_deg, follow_negative_target_deg, base_target_deg},
            360,
            20)) {
        return false;
    }

    if (round_index > 0 && (round_index % 4) == 0) {
        if (!RunEnableDisableCycle(context, round_index)) {
            return false;
        }
    }

    if ((round_index % std::max(1, context->options.switch_cycles)) ==
        (std::max(1, context->options.switch_cycles) - 1)) {
        if (!RunMoveTo(
                context,
                "move_to_return",
                "round-" + std::to_string(round_index) + "-return-home",
                context->reference_angle_deg,
                context->options.velocity_deg_s,
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
        const std::string axis_arg = argv[arg_index];
        if (axis_arg != "auto") {
            options.requested_axis_id = std::atoi(argv[arg_index]);
        }
        ++arg_index;
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

    const int axis_id = ResolveAxisId(controller, options.requested_axis_id);
    if (axis_id < 0) {
        std::cerr << "[select-axis] no bound axis available for test" << '\n';
        RecordFailure(&context.stats, "select_axis", "no bound axis available for test");
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        return 1;
    }
    context.axis_id = axis_id;

    if (!controller.initialize()) {
        const std::string error = controller.lastError();
        std::cerr << "[initialize] failed: " << error << '\n';
        RecordFailure(&context.stats, "initialize", error);
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        return 1;
    }

    SleepMs(300);
    PrintAxisState(controller, axis_id, "initialized");

    const erob::AxisState initialized_state = controller.getAxisState(axis_id);
    if (initialized_state.fault) {
        std::cout << "[reset-fault] axis starts in fault, attempting recovery" << '\n';
        if (!controller.resetFault(axis_id)) {
            const std::string error = controller.lastError();
            std::cerr << "[reset-fault] failed: " << error << '\n';
            RecordFailure(&context.stats, "reset_fault", error);
            AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
            controller.shutdown();
            return 1;
        }
        SleepMs(300);
        PrintAxisState(controller, axis_id, "fault-reset");
    }

    if (!controller.enableAxis(axis_id)) {
        const std::string error = controller.lastError();
        std::cerr << "[enable] failed: " << error << '\n';
        RecordFailure(&context.stats, "enable_axis", error);
        AppendMinuteSummary(context, SteadyClock::now(), &context.stats, true);
        controller.shutdown();
        return 1;
    }

    SleepMs(400);
    PrintAxisState(controller, axis_id, "enabled");

    const erob::AxisState enabled_state = controller.getAxisState(axis_id);
    context.axis_config = controller.config().axes.at(static_cast<std::size_t>(axis_id));
    context.reference_angle_deg = enabled_state.actual_angle_deg;

    std::cout
        << "[test-plan] axis=" << axis_id
        << ", reference_deg=" << context.reference_angle_deg
        << ", position_step_deg=" << options.position_step_deg
        << ", follow_step_deg=" << options.follow_step_deg
        << ", velocity_deg_s=" << options.velocity_deg_s
        << ", switch_cycles=" << options.switch_cycles
        << ", duration_minutes=" << options.duration_minutes
        << ", log_path=" << options.log_path
        << '\n';

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
            std::cerr << "[round] round=" << round_index << " failed, attempting recovery on next iteration" << '\n';
            SleepMs(500);
        }
        ok = ok && round_ok;
        ++round_index;
    }

    if (!controller.disableAxis(axis_id)) {
        const std::string error = controller.lastError();
        std::cerr << "[disable] warning: " << error << '\n';
        RecordFailure(&context.stats, "disable_axis", error);
    }
    SleepMs(250);
    PrintAxisState(controller, axis_id, StopRequested() ? "stopped" : (ok ? "completed" : "aborted"));

    AppendMinuteSummary(context, start_time, &context.stats, true);
    controller.shutdown();
    return ok ? 0 : 1;
}