#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "erob/erob_arm_controller.h"

namespace {

using SteadyClock = std::chrono::steady_clock;

// 这个示例不是压测程序，而是“功能导览 + 二次开发模板”。
// 目标是把常见控制流程按合理顺序串起来，并展示每个阶段该检查什么状态。
struct DemoOptions {
    std::string config_path = "config/erob_arm.yaml";
    std::string axis_selector = "auto";
    double position_step_deg = 8.0;
    double velocity_deg_s = 15.0;
    double follow_step_deg = 5.0;
    bool run_safety_demo = false;
};

void SleepMs(int delay_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
}

void PrintUsage() {
    std::cout
        << "Usage:\n"
        << "  erob_guided_demo [config_path] [axis_selector] [position_step_deg] [velocity_deg_s] [follow_step_deg] [run_safety_demo]\n\n"
        << "参数说明:\n"
        << "  config_path       配置文件路径，默认 config/erob_arm.yaml\n"
        << "  axis_selector     轴选择器，支持 auto/all/0,1,2 这种逗号列表，默认 auto\n"
        << "  position_step_deg PP 示例位移，默认 8 度\n"
        << "  velocity_deg_s    PP 示例速度，默认 15 度每秒\n"
        << "  follow_step_deg   follow 示例摆动幅度，默认 5 度\n"
        << "  run_safety_demo   是否执行急停/上下使能示例，0=跳过，1=执行，默认 0\n\n"
        << "推荐用法:\n"
        << "  erob_guided_demo\n"
        << "  erob_guided_demo config/erob_arm.yaml all 8 15 5 0\n";
}

void PrintStepTitle(const std::string& title) {
    std::cout << "\n========== " << title << " ==========\n";
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

    const std::string normalized = Trim(axis_selector);
    if (normalized.empty() || normalized == "auto" || normalized == "all") {
        return bound_axis_ids;
    }

    const std::vector<int> requested_axis_ids = ParseAxisSelector(normalized);
    std::vector<int> resolved_axis_ids;
    for (const int axis_id : requested_axis_ids) {
        if (std::find(bound_axis_ids.begin(), bound_axis_ids.end(), axis_id) != bound_axis_ids.end()) {
            resolved_axis_ids.push_back(axis_id);
        }
    }
    return resolved_axis_ids;
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

const erob::AxisConfig* FindAxisConfig(const erob::SystemConfig& system_config, int axis_id) {
    for (const erob::AxisConfig& axis_config : system_config.axes) {
        if (axis_config.logical_axis_id == axis_id) {
            return &axis_config;
        }
    }
    return nullptr;
}

double ClampTarget(double angle_deg, const erob::AxisConfig& axis_config) {
    return erob::Clamp(angle_deg, axis_config.min_angle_deg, axis_config.max_angle_deg);
}

bool IsBlockingMoveSuccess(erob::MoveCommandStatus status) {
    return status == erob::MoveCommandStatus::kCompleted;
}

bool IsNonBlockingMoveSuccess(erob::MoveCommandStatus status) {
    return status == erob::MoveCommandStatus::kIssued;
}

void PrintDiscoveredMotors(const std::vector<erob::MotorIdentity>& motors) {
    std::cout << "[scan] 发现电机数量: " << motors.size() << '\n';
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
    std::cout << "[binding] 绑定结果数量: " << reports.size() << '\n';
    for (const erob::AxisBindingReport& report : reports) {
        std::cout
            << "[binding] axis=" << report.logical_axis_id
            << ", joint=" << report.joint_name
            << ", bound=" << report.bound
            << ", configured_identity=" << report.configured_identity
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
        << ", actual_deg=" << std::fixed << std::setprecision(2) << state.actual_angle_deg
        << ", target_deg=" << state.target_angle_deg
        << ", error_deg=" << state.position_error_deg
        << ", velocity_deg_s=" << state.actual_velocity_deg_s
        << ", target_reached=" << state.target_reached
        << ", statusword=0x" << std::hex << state.statusword << std::dec
        << '\n';
}

void PrintAxisStates(const erob::ErobArmController& controller, const std::vector<int>& axis_ids, const std::string& stage) {
    for (const int axis_id : axis_ids) {
        PrintAxisState(controller, axis_id, stage);
    }
}

bool WaitForAxesIdle(erob::ErobArmController* controller, const std::vector<int>& axis_ids, const std::string& stage, int timeout_ms) {
    if (controller == nullptr) {
        return false;
    }

    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    while (SteadyClock::now() < deadline) {
        bool all_idle = true;
        for (const int axis_id : axis_ids) {
            const erob::AxisState state = controller->getAxisState(axis_id);
            if (!state.online || state.fault) {
                PrintAxisState(*controller, axis_id, stage + "-fault-or-offline");
                return false;
            }
            const bool follow_idle = state.follow_mode_state == erob::FollowModeState::kIdle;
            const bool axis_not_busy = !controller->isAxisBusy(axis_id);
            const bool low_speed = std::fabs(state.actual_velocity_deg_s) <= 0.8;
            if (!(follow_idle && axis_not_busy && low_speed)) {
                all_idle = false;
                break;
            }
        }
        if (all_idle) {
            return true;
        }
        SleepMs(20);
    }

    PrintAxisStates(*controller, axis_ids, stage + "-timeout");
    return false;
}

bool EnsureAxisReady(erob::ErobArmController* controller, int axis_id) {
    if (controller == nullptr) {
        return false;
    }

    // 二次开发时的常用模板：每次下发运动前，先做 fault/enable 检查。
    // 这样可以把“现场状态恢复”与“业务动作”解耦，避免后面每条指令都夹杂异常处理。
    erob::AxisState state = controller->getAxisState(axis_id);
    if (state.fault) {
        std::cout << "[recover] axis=" << axis_id << " 存在 fault，先执行 resetFault" << '\n';
        if (!controller->resetFault(axis_id)) {
            std::cerr << "[recover] resetFault failed: " << controller->lastError() << '\n';
            return false;
        }
        SleepMs(300);
        state = controller->getAxisState(axis_id);
    }

    if (!state.enabled) {
        std::cout << "[recover] axis=" << axis_id << " 未使能，执行 enableAxis" << '\n';
        if (!controller->enableAxis(axis_id)) {
            std::cerr << "[recover] enableAxis failed: " << controller->lastError() << '\n';
            return false;
        }
        SleepMs(350);
    }

    PrintAxisState(*controller, axis_id, "ready");
    return true;
}

bool EnsureAxesReady(erob::ErobArmController* controller, const std::vector<int>& axis_ids) {
    for (const int axis_id : axis_ids) {
        if (!EnsureAxisReady(controller, axis_id)) {
            return false;
        }
    }
    return true;
}

bool RunSingleAxisBlockingDemo(
    erob::ErobArmController* controller,
    const erob::AxisConfig& axis_config,
    int axis_id,
    double position_step_deg,
    double velocity_deg_s) {
    if (controller == nullptr) {
        return false;
    }

    PrintStepTitle("阶段 4：单轴阻塞式 PP moveTo");
    const erob::AxisState state = controller->getAxisState(axis_id);
    const double target_deg = ClampTarget(state.actual_angle_deg + position_step_deg, axis_config);
    std::cout << "[single-blocking] axis=" << axis_id
              << " target_deg=" << target_deg
              << " velocity_deg_s=" << velocity_deg_s << '\n';

    // moveTo 是最直观的“下发并等待完成”接口，适合教程、脚本和串行任务。
    const erob::MoveCommandStatus status = controller->moveTo(axis_id, target_deg, velocity_deg_s);
    if (!IsBlockingMoveSuccess(status)) {
        std::cerr << "[single-blocking] moveTo failed: status="
                  << erob::MoveCommandStatusName(status)
                  << ", error=" << controller->lastError() << '\n';
        return false;
    }

    PrintAxisState(*controller, axis_id, "single-blocking-completed");
    return true;
}

bool RunSingleAxisIssuedDemo(
    erob::ErobArmController* controller,
    const erob::AxisConfig& axis_config,
    int axis_id,
    double position_step_deg,
    double velocity_deg_s) {
    if (controller == nullptr) {
        return false;
    }

    PrintStepTitle("阶段 5：单轴非阻塞 issueMoveTo + isAxisBusy");
    const erob::AxisState state = controller->getAxisState(axis_id);
    const double target_deg = ClampTarget(state.actual_angle_deg - position_step_deg, axis_config);
    std::cout << "[single-issued] axis=" << axis_id
              << " target_deg=" << target_deg
              << " velocity_deg_s=" << velocity_deg_s << '\n';

    // issueMoveTo 只负责“尽快下发”，随后由调用方自行决定如何等待、并发或超时控制。
    const erob::MoveCommandStatus status = controller->issueMoveTo(axis_id, target_deg, velocity_deg_s);
    if (!IsNonBlockingMoveSuccess(status)) {
        std::cerr << "[single-issued] issueMoveTo failed: status="
                  << erob::MoveCommandStatusName(status)
                  << ", error=" << controller->lastError() << '\n';
        return false;
    }

    if (!WaitForAxesIdle(controller, {axis_id}, "single-issued", 8000)) {
        std::cerr << "[single-issued] 等待轴空闲超时" << '\n';
        return false;
    }

    PrintAxisState(*controller, axis_id, "single-issued-completed");
    return true;
}

std::vector<erob::AxisMoveRequest> BuildGroupRequests(
    erob::ErobArmController* controller,
    const std::vector<int>& axis_ids,
    double base_step_deg,
    double velocity_deg_s,
    bool positive_direction) {
    std::vector<erob::AxisMoveRequest> requests;
    if (controller == nullptr) {
        return requests;
    }

    requests.reserve(axis_ids.size());
    const erob::SystemConfig& system_config = controller->config();
    for (std::size_t index = 0; index < axis_ids.size(); ++index) {
        const int axis_id = axis_ids[index];
        const erob::AxisConfig* axis_config = FindAxisConfig(system_config, axis_id);
        if (axis_config == nullptr) {
            continue;
        }
        const erob::AxisState state = controller->getAxisState(axis_id);
        const double direction = positive_direction ? 1.0 : -1.0;
        const double step_deg = base_step_deg * (1.0 + 0.15 * static_cast<double>(index));
        requests.push_back(erob::AxisMoveRequest{
            axis_id,
            ClampTarget(state.actual_angle_deg + direction * step_deg, *axis_config),
            velocity_deg_s,
        });
    }
    return requests;
}

bool RunGroupBlockingDemo(erob::ErobArmController* controller, const std::vector<int>& axis_ids, double step_deg, double velocity_deg_s) {
    if (controller == nullptr || axis_ids.size() < 2) {
        return true;
    }

    PrintStepTitle("阶段 6：多轴阻塞式 moveGroup");
    const auto requests = BuildGroupRequests(controller, axis_ids, step_deg, velocity_deg_s, true);
    for (const erob::AxisMoveRequest& request : requests) {
        std::cout << "[group-blocking] axis=" << request.axis_id
                  << ", target_deg=" << request.angle_deg
                  << ", velocity_deg_s=" << request.velocity_deg_s << '\n';
    }

    // moveGroup(wait_all=true) 适合“多轴一起下发，然后整体等完成”的业务。
    const erob::MoveCommandStatus status = controller->moveGroup(requests, true, true);
    if (!IsBlockingMoveSuccess(status)) {
        std::cerr << "[group-blocking] moveGroup failed: status="
                  << erob::MoveCommandStatusName(status)
                  << ", error=" << controller->lastError() << '\n';
        return false;
    }

    PrintAxisStates(*controller, axis_ids, "group-blocking-completed");
    return true;
}

bool RunGroupIssuedDemo(erob::ErobArmController* controller, const std::vector<int>& axis_ids, double step_deg, double velocity_deg_s) {
    if (controller == nullptr || axis_ids.size() < 2) {
        return true;
    }

    PrintStepTitle("阶段 7：多轴非阻塞 moveGroup");
    const auto requests = BuildGroupRequests(controller, axis_ids, step_deg, velocity_deg_s, false);
    for (const erob::AxisMoveRequest& request : requests) {
        std::cout << "[group-issued] axis=" << request.axis_id
                  << ", target_deg=" << request.angle_deg
                  << ", velocity_deg_s=" << request.velocity_deg_s << '\n';
    }

    // wait_all=false 适合上层自己管理等待、并发编排、或结合 GUI/状态机轮询。
    const erob::MoveCommandStatus status = controller->moveGroup(requests, false, true);
    if (!IsNonBlockingMoveSuccess(status)) {
        std::cerr << "[group-issued] moveGroup failed: status="
                  << erob::MoveCommandStatusName(status)
                  << ", error=" << controller->lastError() << '\n';
        return false;
    }

    if (!WaitForAxesIdle(controller, axis_ids, "group-issued", 10000)) {
        std::cerr << "[group-issued] 等待多轴空闲超时" << '\n';
        return false;
    }

    PrintAxisStates(*controller, axis_ids, "group-issued-completed");
    return true;
}

bool RunFollowDemo(
    erob::ErobArmController* controller,
    const erob::AxisConfig& axis_config,
    int axis_id,
    double follow_step_deg) {
    if (controller == nullptr) {
        return false;
    }

    PrintStepTitle("阶段 8：单轴 follow 模式");
    if (!controller->startFollowMode(axis_id)) {
        std::cerr << "[follow] startFollowMode failed: " << controller->lastError() << '\n';
        return false;
    }
    SleepMs(180);

    // follow 模式的推荐写法是“小步高频更新目标”，而不是一次性给很大的跳变。
    // 这里用 3 个点示范：向正方向走一点、向负方向走一点、最后回到当前附近。
    const erob::AxisState state = controller->getAxisState(axis_id);
    const std::vector<double> targets_deg = {
        ClampTarget(state.actual_angle_deg + follow_step_deg, axis_config),
        ClampTarget(state.actual_angle_deg - follow_step_deg, axis_config),
        ClampTarget(state.actual_angle_deg + follow_step_deg * 0.5, axis_config),
    };

    for (std::size_t index = 0; index < targets_deg.size(); ++index) {
        std::cout << "[follow] update " << index << " target_deg=" << targets_deg[index] << '\n';
        if (!controller->updateFollowTarget(axis_id, targets_deg[index])) {
            std::cerr << "[follow] updateFollowTarget failed: " << controller->lastError() << '\n';
            controller->stopFollowMode(axis_id);
            return false;
        }
        SleepMs(280);
        PrintAxisState(*controller, axis_id, "follow-streaming");
    }

    if (!controller->stopFollowMode(axis_id)) {
        std::cerr << "[follow] stopFollowMode failed: " << controller->lastError() << '\n';
        return false;
    }
    if (!WaitForAxesIdle(controller, {axis_id}, "follow-stop", 5000)) {
        std::cerr << "[follow] 停止 follow 后等待空闲超时" << '\n';
        return false;
    }

    PrintAxisState(*controller, axis_id, "follow-completed");
    return true;
}

bool RunSafetyDemo(erob::ErobArmController* controller, const std::vector<int>& axis_ids) {
    if (controller == nullptr) {
        return false;
    }

    PrintStepTitle("阶段 9：安全动作与恢复示例");
    std::cout << "[safety] 先演示 quickStopAll，再恢复到可继续控制的状态" << '\n';
    if (!controller->quickStopAll()) {
        std::cerr << "[safety] quickStopAll failed: " << controller->lastError() << '\n';
        return false;
    }
    SleepMs(300);
    PrintAxisStates(*controller, axis_ids, "safety-after-quick-stop");

    // 急停之后，不同驱动的恢复链路可能略有差异。
    // 这里统一复用 EnsureAxesReady，把“恢复”和“业务动作”继续隔离开。
    if (!EnsureAxesReady(controller, axis_ids)) {
        return false;
    }

    std::cout << "[safety] 再演示 disableAll，说明上层如何做整机下电或退出前清理" << '\n';
    if (!controller->disableAll()) {
        std::cerr << "[safety] disableAll failed: " << controller->lastError() << '\n';
        return false;
    }
    SleepMs(250);
    PrintAxisStates(*controller, axis_ids, "safety-after-disable-all");

    std::cout << "[safety] 为了让 demo 结束前恢复可控状态，再调用 enableAll / EnsureAxesReady" << '\n';
    if (!controller->enableAll()) {
        std::cerr << "[safety] enableAll failed: " << controller->lastError() << '\n';
        return false;
    }
    SleepMs(350);
    if (!EnsureAxesReady(controller, axis_ids)) {
        return false;
    }

    PrintAxisStates(*controller, axis_ids, "safety-completed");
    return true;
}

DemoOptions ParseOptions(int argc, char** argv) {
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
        options.run_safety_demo = std::atoi(argv[arg_index]) != 0;
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::string(argv[1]) == "--help") {
        PrintUsage();
        return 0;
    }

    const DemoOptions options = ParseOptions(argc, argv);
    erob::ErobArmController controller(options.config_path);

    PrintStepTitle("阶段 1：网卡与电机扫描");
    const auto adapters = controller.scanAdapters();
    std::cout << "[scan] 可见适配器数量: " << adapters.size() << '\n';
    for (const erob::AdapterInfo& adapter : adapters) {
        std::cout << "[scan] adapter=" << adapter.name
                  << ", description=" << adapter.description
                  << ", scan_success=" << adapter.scan_success
                  << ", discovered_slave_count=" << adapter.discovered_slave_count
                  << '\n';
    }
    PrintDiscoveredMotors(controller.scanMotorsOnAllAdapters());

    PrintStepTitle("阶段 2：scanAndInitialize 一步完成连接、绑定和线程启动");
    if (!controller.scanAndInitialize()) {
        std::cerr << "[initialize] scanAndInitialize failed: " << controller.lastError() << '\n';
        return 1;
    }
    SleepMs(300);

    PrintStepTitle("阶段 3：读取绑定结果与初始状态");
    PrintBindingReports(controller);
    const std::vector<int> axis_ids = ResolveAxisIds(controller, options.axis_selector);
    if (axis_ids.empty()) {
        std::cerr << "[select-axis] 轴选择结果为空，请检查 axis_selector 或绑定状态" << '\n';
        controller.shutdown();
        return 1;
    }
    std::cout << "[select-axis] 本次示例使用轴: " << AxisIdsString(axis_ids) << '\n';
    PrintAxisStates(controller, axis_ids, "initial");

    if (!EnsureAxesReady(&controller, axis_ids)) {
        controller.shutdown();
        return 1;
    }
    if (!WaitForAxesIdle(&controller, axis_ids, "after-enable", 5000)) {
        std::cerr << "[after-enable] 轴未能进入空闲状态" << '\n';
        controller.shutdown();
        return 1;
    }

    const erob::AxisConfig* first_axis_config = FindAxisConfig(controller.config(), axis_ids.front());
    if (first_axis_config == nullptr) {
        std::cerr << "[config] 未找到首个示例轴的配置" << '\n';
        controller.shutdown();
        return 1;
    }

    if (!RunSingleAxisBlockingDemo(
            &controller,
            *first_axis_config,
            axis_ids.front(),
            options.position_step_deg,
            options.velocity_deg_s)) {
        controller.shutdown();
        return 1;
    }

    if (!RunSingleAxisIssuedDemo(
            &controller,
            *first_axis_config,
            axis_ids.front(),
            options.position_step_deg,
            std::max(1.0, options.velocity_deg_s * 0.8))) {
        controller.shutdown();
        return 1;
    }

    if (!RunGroupBlockingDemo(&controller, axis_ids, options.position_step_deg, options.velocity_deg_s)) {
        controller.shutdown();
        return 1;
    }

    if (!RunGroupIssuedDemo(&controller, axis_ids, options.position_step_deg, options.velocity_deg_s)) {
        controller.shutdown();
        return 1;
    }

    if (!RunFollowDemo(&controller, *first_axis_config, axis_ids.front(), options.follow_step_deg)) {
        controller.shutdown();
        return 1;
    }

    if (options.run_safety_demo) {
        if (!RunSafetyDemo(&controller, axis_ids)) {
            controller.shutdown();
            return 1;
        }
    } else {
        PrintStepTitle("阶段 9：安全动作示例已跳过");
        std::cout << "[safety] 若要演示 quickStopAll / disableAll / enableAll，请把最后一个参数设为 1。\n";
    }

    PrintStepTitle("阶段 10：结束前状态检查与 shutdown");
    PrintAxisStates(controller, axis_ids, "final");
    if (!controller.shutdown()) {
        std::cerr << "[shutdown] shutdown failed: " << controller.lastError() << '\n';
        return 1;
    }

    std::cout << "\n[done] 引导型 demo 已执行完成。你可以直接复制其中某个阶段的 helper，作为后续复杂业务的模板。\n";
    return 0;
}