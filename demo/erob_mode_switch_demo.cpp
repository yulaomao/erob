#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "erob/erob_arm_controller.h"

namespace {

double ClampTarget(double angle_deg, const erob::AxisConfig& axis_config) {
    return erob::Clamp(angle_deg, axis_config.min_angle_deg, axis_config.max_angle_deg);
}

void PrintUsage() {
    std::cout
        << "Usage:\n"
    << "  erob_mode_switch_demo [config_path] [axis_id|auto] [position_step_deg] [velocity_deg_s] [follow_step_deg] [switch_cycles]\n\n"
        << "Defaults:\n"
        << "  config_path=config/erob_arm.yaml\n"
        << "  axis_id=auto (first bound axis)\n"
        << "  position_step_deg=12\n"
        << "  velocity_deg_s=20\n"
    << "  follow_step_deg=6\n"
    << "  switch_cycles=3\n";
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

bool RunProfilePositionStep(
    erob::ErobArmController* controller,
    int axis_id,
    double target_deg,
    double velocity_deg_s,
    const std::string& label) {
    std::cout << '[' << label << "] moveTo target_deg=" << target_deg
              << ", velocity_deg_s=" << velocity_deg_s << '\n';
    if (!controller->moveTo(axis_id, target_deg, velocity_deg_s)) {
        std::cerr << '[' << label << "] moveTo failed: " << controller->lastError() << '\n';
        PrintAxisState(*controller, axis_id, label + "-failed");
        return false;
    }
    SleepMs(300);
    PrintAxisState(*controller, axis_id, label + "-reached");
    return true;
}

bool RunProfilePositionSequence(
    erob::ErobArmController* controller,
    int axis_id,
    const std::vector<double>& targets_deg,
    double velocity_deg_s,
    const std::string& label_prefix) {
    for (std::size_t index = 0; index < targets_deg.size(); ++index) {
        const std::string label =
            label_prefix + "-step-" + std::to_string(static_cast<unsigned long long>(index));
        if (!RunProfilePositionStep(controller, axis_id, targets_deg[index], velocity_deg_s, label)) {
            return false;
        }
    }
    return true;
}

bool RunFollowStep(
    erob::ErobArmController* controller,
    int axis_id,
    const std::vector<double>& targets_deg,
    int segment_duration_ms,
    int update_period_ms,
    const std::string& label_prefix) {
    std::cout << '[' << label_prefix << "] starting follow mode" << '\n';
    if (!controller->startFollowMode(axis_id)) {
        std::cerr << '[' << label_prefix << "] start failed: " << controller->lastError() << '\n';
        PrintAxisState(*controller, axis_id, label_prefix + "-start-failed");
        return false;
    }

    SleepMs(std::max(10, update_period_ms));
    PrintAxisState(*controller, axis_id, label_prefix + "-started");

    double last_target_deg = controller->getAxisState(axis_id).actual_angle_deg;
    for (std::size_t index = 0; index < targets_deg.size(); ++index) {
        const double target_deg = targets_deg[index];
        std::cout << '[' << label_prefix << "] target[" << index << "]=" << target_deg << '\n';
        const int steps = std::max(1, segment_duration_ms / std::max(1, update_period_ms));
        for (int step = 1; step <= steps; ++step) {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            const double streamed_target_deg =
                last_target_deg + ratio * (target_deg - last_target_deg);
            if (!controller->updateFollowTarget(axis_id, streamed_target_deg)) {
                std::cerr << '[' << label_prefix << "] update failed: " << controller->lastError() << '\n';
                PrintAxisState(*controller, axis_id, label_prefix + "-update-failed");
                controller->stopFollowMode(axis_id);
                return false;
            }
            SleepMs(update_period_ms);
        }
        PrintAxisState(
            *controller,
            axis_id,
            label_prefix + "-running-" + std::to_string(static_cast<unsigned long long>(index)));
        last_target_deg = target_deg;
    }

    if (!controller->stopFollowMode(axis_id)) {
        std::cerr << '[' << label_prefix << "] stop failed: " << controller->lastError() << '\n';
        PrintAxisState(*controller, axis_id, label_prefix + "-stop-failed");
        return false;
    }

    SleepMs(400);
    PrintAxisState(*controller, axis_id, label_prefix + "-stopped");
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/erob_arm.yaml";
    int arg_index = 1;
    if (argc > 1 && std::string(argv[1]).find(".yaml") != std::string::npos) {
        config_path = argv[1];
        arg_index = 2;
    }

    int requested_axis_id = -1;
    if (argc > arg_index) {
        const std::string axis_arg = argv[arg_index];
        if (axis_arg != "auto") {
            requested_axis_id = std::atoi(argv[arg_index]);
        }
        ++arg_index;
    }

    double position_step_deg = 12.0;
    if (argc > arg_index) {
        position_step_deg = std::atof(argv[arg_index++]);
    }

    double velocity_deg_s = 20.0;
    if (argc > arg_index) {
        velocity_deg_s = std::atof(argv[arg_index++]);
    }

    double follow_step_deg = 6.0;
    if (argc > arg_index) {
        follow_step_deg = std::atof(argv[arg_index++]);
    }

    int switch_cycles = 3;
    if (argc > arg_index) {
        switch_cycles = std::atoi(argv[arg_index++]);
    }

    if (velocity_deg_s <= 0.0 || switch_cycles <= 0) {
        PrintUsage();
        return 1;
    }

    erob::ErobArmController controller(config_path);

    const auto motors = controller.scanAndBind();
    if (motors.empty()) {
        std::cerr << "[scan] failed or no motors discovered: " << controller.lastError() << '\n';
        return 1;
    }
    PrintDiscoveredMotors(motors);
    PrintBindingReports(controller);

    const int axis_id = ResolveAxisId(controller, requested_axis_id);
    if (axis_id < 0) {
        std::cerr << "[select-axis] no bound axis available for test" << '\n';
        return 1;
    }

    if (!controller.initialize()) {
        std::cerr << "[initialize] failed: " << controller.lastError() << '\n';
        return 1;
    }

    SleepMs(300);
    PrintAxisState(controller, axis_id, "initialized");

    const erob::AxisState initialized_state = controller.getAxisState(axis_id);
    if (initialized_state.fault) {
        std::cout << "[reset-fault] axis starts in fault, attempting recovery" << '\n';
        if (!controller.resetFault(axis_id)) {
            std::cerr << "[reset-fault] failed: " << controller.lastError() << '\n';
            controller.shutdown();
            return 1;
        }
        SleepMs(300);
        PrintAxisState(controller, axis_id, "fault-reset");
    }

    if (!controller.enableAxis(axis_id)) {
        std::cerr << "[enable] failed: " << controller.lastError() << '\n';
        controller.shutdown();
        return 1;
    }

    SleepMs(400);
    PrintAxisState(controller, axis_id, "enabled");

    const erob::AxisState enabled_state = controller.getAxisState(axis_id);
    const erob::AxisConfig& axis_config = controller.config().axes.at(static_cast<std::size_t>(axis_id));
    const double start_angle_deg = enabled_state.actual_angle_deg;

    std::vector<double> profile_targets_deg;
    std::vector<std::vector<double>> follow_target_sequences_deg;
    profile_targets_deg.reserve(static_cast<std::size_t>(switch_cycles + 1));
    follow_target_sequences_deg.reserve(static_cast<std::size_t>(switch_cycles));

    for (int cycle = 0; cycle < switch_cycles; ++cycle) {
        const double direction = (cycle % 2 == 0) ? 1.0 : -1.0;
        const double base_target_deg = ClampTarget(
            start_angle_deg + direction * position_step_deg,
            axis_config);
        const double follow_positive_target_deg = ClampTarget(
            base_target_deg + direction * follow_step_deg,
            axis_config);
        const double follow_negative_target_deg = ClampTarget(
            base_target_deg - direction * follow_step_deg,
            axis_config);
        profile_targets_deg.push_back(base_target_deg);
        follow_target_sequences_deg.push_back(
            {follow_positive_target_deg, follow_negative_target_deg, base_target_deg});
    }
    profile_targets_deg.push_back(ClampTarget(start_angle_deg, axis_config));

    std::cout
        << "[test-plan] axis=" << axis_id
        << ", start_deg=" << start_angle_deg
        << ", position_step_deg=" << position_step_deg
        << ", follow_step_deg=" << follow_step_deg
        << ", velocity_deg_s=" << velocity_deg_s
        << ", switch_cycles=" << switch_cycles
        << '\n';

    for (int cycle = 0; cycle < switch_cycles; ++cycle) {
        std::cout
            << "[test-plan] cycle=" << cycle
            << ", pp_target_deg=" << profile_targets_deg[static_cast<std::size_t>(cycle)]
            << ", follow_targets_deg=["
            << follow_target_sequences_deg[static_cast<std::size_t>(cycle)][0]
            << ", " << follow_target_sequences_deg[static_cast<std::size_t>(cycle)][1]
            << ", " << follow_target_sequences_deg[static_cast<std::size_t>(cycle)][2]
            << "]"
            << '\n';
    }
    std::cout << "[test-plan] final_return_deg=" << profile_targets_deg.back() << '\n';

    bool ok = true;
    for (int cycle = 0; cycle < switch_cycles && ok; ++cycle) {
        const std::string cycle_label = "cycle-" + std::to_string(cycle);
        ok = RunProfilePositionSequence(
            &controller,
            axis_id,
            {profile_targets_deg[static_cast<std::size_t>(cycle)]},
            velocity_deg_s,
            cycle_label + "-pp-enter");
        if (!ok) {
            break;
        }

        ok = RunFollowStep(
            &controller,
            axis_id,
            follow_target_sequences_deg[static_cast<std::size_t>(cycle)],
            360,
            20,
            cycle_label + "-follow");
    }

    if (ok) {
        ok = RunProfilePositionSequence(
            &controller,
            axis_id,
            {profile_targets_deg.back()},
            velocity_deg_s,
            "final-pp-return");
    }

    if (!controller.disableAxis(axis_id)) {
        std::cerr << "[disable] warning: " << controller.lastError() << '\n';
    }
    SleepMs(250);
    PrintAxisState(controller, axis_id, ok ? "completed" : "aborted");
    controller.shutdown();
    return ok ? 0 : 1;
}