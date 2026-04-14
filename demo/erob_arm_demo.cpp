#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

#include "erob/erob_arm_controller.h"

namespace {

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  erob_arm_demo [config_path] scan\n"
        << "  erob_arm_demo [config_path] rescan-current\n"
        << "  erob_arm_demo [config_path] rescan-all\n"
        << "  erob_arm_demo [config_path] state\n"
        << "  erob_arm_demo [config_path] enable-axis <axis_id>\n"
        << "  erob_arm_demo [config_path] disable-axis <axis_id>\n"
        << "  erob_arm_demo [config_path] reset-fault <axis_id>\n"
        << "  erob_arm_demo [config_path] disable-all\n"
        << "  erob_arm_demo [config_path] quick-stop\n"
        << "  erob_arm_demo [config_path] enable-all\n"
        << "  erob_arm_demo [config_path] move <axis_id> <angle_deg> <velocity_deg_s>\n"
        << "  erob_arm_demo [config_path] follow <axis_id> <angle_deg_1> [angle_deg_2 ...]\n";
}

void PrintStates(const erob::ErobArmController& controller) {
    std::cout << "controller: degraded=" << controller.isDegraded() << '\n';
    const auto binding_reports = controller.getBindingReports();
    for (const erob::AxisBindingReport& report : binding_reports) {
        std::cout
            << "binding axis " << report.logical_axis_id
            << " (" << report.joint_name << ")"
            << ": bound=" << report.bound
            << ", configured_identity=" << report.configured_identity
            << ", detail=" << report.detail
            << '\n';
    }

    const auto states = controller.getAllAxisStates();
    for (std::size_t axis_id = 0; axis_id < states.size(); ++axis_id) {
        const erob::AxisState& state = states[axis_id];
        std::cout
            << "axis " << axis_id
            << ": online=" << state.online
            << ", enabled=" << state.enabled
            << ", fault=" << state.fault
            << ", mode=" << erob::MotionModeName(state.motion_mode)
            << ", cia402=" << erob::CiA402StateName(state.cia402_state)
            << ", position_state=" << erob::PositionModeStateName(state.position_mode_state)
            << ", follow_state=" << erob::FollowModeStateName(state.follow_mode_state)
            << ", angle_deg=" << state.actual_angle_deg
            << ", velocity_deg_s=" << state.actual_velocity_deg_s
            << ", target_angle_deg=" << state.target_angle_deg
            << ", position_error_deg=" << state.position_error_deg
            << ", target_reached=" << state.target_reached
            << ", near_negative_limit=" << state.near_negative_limit
            << ", near_positive_limit=" << state.near_positive_limit
            << ", al_status=0x" << std::hex << state.al_status_code
            << ", last_error_code=0x" << state.last_error_code
            << ", statusword=0x" << std::hex << state.statusword << std::dec
            << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "config/erob_arm.yaml";
    int arg_index = 1;
    if (argc > 1 && std::string(argv[1]).find(".yaml") != std::string::npos) {
        config_path = argv[1];
        arg_index = 2;
    }

    if (argc <= arg_index) {
        PrintUsage();
        return 1;
    }

    const std::string command = argv[arg_index];
    erob::ErobArmController controller(config_path);

    if (command == "scan") {
        const auto adapters = controller.scanAdapters();
        std::cout << "Adapters:\n";
        for (const auto& adapter : adapters) {
            std::cout
                << "  - " << adapter.name
                << " (" << adapter.description << ")"
                << ", scan_success=" << adapter.scan_success
                << ", discovered_slave_count=" << adapter.discovered_slave_count
                << "\n";
        }
        const auto motors = controller.scanMotorsOnAllAdapters();
        std::cout << "Motors:\n";
        for (const auto& motor : motors) {
            std::cout
                << "  - adapter=" << motor.adapter_name
                << ", slave=" << motor.slave_index
                << ", name=" << motor.name
                << ", serial=" << motor.serial_number
                << ", erob=" << motor.is_erob_motor << '\n';
        }
        return 0;
    }

    if (command == "rescan-current") {
        const auto motors = controller.rescanCurrentAdapter();
        if (motors.empty() && !controller.lastError().empty()) {
            std::cerr << "rescan-current failed: " << controller.lastError() << '\n';
            return 1;
        }
        for (const auto& motor : motors) {
            std::cout
                << "  - adapter=" << motor.adapter_name
                << ", slave=" << motor.slave_index
                << ", name=" << motor.name
                << ", serial=" << motor.serial_number
                << '\n';
        }
        return 0;
    }

    if (command == "rescan-all") {
        const auto motors = controller.rescanAllAdapters();
        if (motors.empty() && !controller.lastError().empty()) {
            std::cerr << "rescan-all failed: " << controller.lastError() << '\n';
            return 1;
        }
        for (const auto& motor : motors) {
            std::cout
                << "  - adapter=" << motor.adapter_name
                << ", slave=" << motor.slave_index
                << ", name=" << motor.name
                << ", serial=" << motor.serial_number
                << '\n';
        }
        return 0;
    }

    if (!controller.initialize()) {
        std::cerr << "initialize failed: " << controller.lastError() << '\n';
        return 1;
    }

    if (command == "state") {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        PrintStates(controller);
    } else if (command == "enable-axis") {
        if (argc < arg_index + 2) {
            PrintUsage();
            controller.shutdown();
            return 1;
        }
        const int axis_id = std::atoi(argv[arg_index + 1]);
        if (!controller.enableAxis(axis_id)) {
            std::cerr << "enable-axis failed: " << controller.lastError() << '\n';
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        PrintStates(controller);
    } else if (command == "disable-axis") {
        if (argc < arg_index + 2) {
            PrintUsage();
            controller.shutdown();
            return 1;
        }
        const int axis_id = std::atoi(argv[arg_index + 1]);
        if (!controller.disableAxis(axis_id)) {
            std::cerr << "disable-axis failed: " << controller.lastError() << '\n';
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        PrintStates(controller);
    } else if (command == "reset-fault") {
        if (argc < arg_index + 2) {
            PrintUsage();
            controller.shutdown();
            return 1;
        }
        const int axis_id = std::atoi(argv[arg_index + 1]);
        if (!controller.resetFault(axis_id)) {
            std::cerr << "reset-fault failed: " << controller.lastError() << '\n';
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        PrintStates(controller);
    } else if (command == "disable-all") {
        if (!controller.disableAll()) {
            std::cerr << "disable-all failed: " << controller.lastError() << '\n';
        }
    } else if (command == "quick-stop") {
        if (!controller.quickStopAll()) {
            std::cerr << "quick-stop failed: " << controller.lastError() << '\n';
        }
    } else if (command == "enable-all") {
        if (!controller.enableAll()) {
            std::cerr << "enable-all failed: " << controller.lastError() << '\n';
        }
    } else if (command == "move") {
        if (argc < arg_index + 4) {
            PrintUsage();
            controller.shutdown();
            return 1;
        }
        const int axis_id = std::atoi(argv[arg_index + 1]);
        const double angle_deg = std::atof(argv[arg_index + 2]);
        const double velocity_deg_s = std::atof(argv[arg_index + 3]);
        if (!controller.enableAxis(axis_id)) {
            std::cerr << "enable-axis failed: " << controller.lastError() << '\n';
            controller.shutdown();
            return 1;
        }
        if (!controller.moveTo(axis_id, angle_deg, velocity_deg_s)) {
            std::cerr << "move failed: " << controller.lastError() << '\n';
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
        PrintStates(controller);
    } else if (command == "follow") {
        if (argc < arg_index + 3) {
            PrintUsage();
            controller.shutdown();
            return 1;
        }
        const int axis_id = std::atoi(argv[arg_index + 1]);
        if (!controller.enableAxis(axis_id)) {
            std::cerr << "enable-axis failed: " << controller.lastError() << '\n';
            controller.shutdown();
            return 1;
        }
        if (!controller.startFollowMode(axis_id)) {
            std::cerr << "start follow failed: " << controller.lastError() << '\n';
            controller.shutdown();
            return 1;
        }
        for (int index = arg_index + 2; index < argc; ++index) {
            if (!controller.updateFollowTarget(axis_id, std::atof(argv[index]))) {
                std::cerr << "update follow target failed: " << controller.lastError() << '\n';
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        controller.stopFollowMode(axis_id);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        PrintStates(controller);
    } else {
        PrintUsage();
        controller.shutdown();
        return 1;
    }

    controller.shutdown();
    return 0;
}