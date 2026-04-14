#ifndef EROB_ARM_CONTROLLER_H_
#define EROB_ARM_CONTROLLER_H_

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "erob/erob_axis.h"
#include "erob/erob_config.h"
#include "erob/ethercat_master_session.h"

namespace erob {

class ErobArmController {
public:
    explicit ErobArmController(std::string config_path = "config/erob_arm.yaml");
    ~ErobArmController();

    bool initialize();
    bool shutdown();

    bool loadConfig(const std::string& path);
    bool saveConfig(const std::string& path) const;

    std::vector<AdapterInfo> scanAdapters();
    std::vector<MotorIdentity> scanMotorsOnAllAdapters();
    std::vector<MotorIdentity> rescan();
    std::vector<MotorIdentity> rescanCurrentAdapter();
    std::vector<MotorIdentity> rescanAllAdapters();

    bool connect(const std::string& adapter_name);
    bool bindAxis(int axis_id, const MotorIdentity& motor);
    bool connectAndBind();

    bool enableAxis(int axis_id);
    bool disableAxis(int axis_id);
    bool resetFault(int axis_id);
    bool enableAll();
    bool disableAll();

    bool moveTo(int axis_id, double angle_deg, double velocity_deg_s);
    bool startFollowMode(int axis_id);
    bool updateFollowTarget(int axis_id, double angle_deg);
    bool stopFollowMode(int axis_id);
    std::vector<MotorIdentity> scanAndBind();

    AxisState getAxisState(int axis_id) const;
    std::vector<AxisState> getAllAxisStates() const;
    std::vector<AxisBindingReport> getBindingReports() const;
    std::vector<MotorIdentity> discoveredMotors() const;
    bool isDegraded() const;
    const SystemConfig& config() const;
    void setPreferredAdapter(const std::string& adapter_name);

    bool quickStopAxis(int axis_id);
    bool quickStopAll();

    std::string lastError() const;

private:
    ErobAxis* axis(int axis_id);
    const ErobAxis* axis(int axis_id) const;
    bool startThreads();
    void stopThreads();
    void cycleLoop();
    void followLoop();
    void monitorLoop();
    bool loadDiscoveryCache();
    bool canRescan() const;
    bool applyProfilePositionParams(uint16_t slave_index, const ProfilePositionParams& params);
    bool autoBindDiscoveredMotors(const std::vector<MotorIdentity>& motors);
    void rebuildAxesFromDiscoveredMotors(const std::vector<MotorIdentity>& motors);
    void auditBindingState(const std::vector<MotorIdentity>& motors);
    void syncAxisControlRates();
    std::string discoveryCachePath() const;
    void setLastError(const std::string& message);

    ConfigManager config_manager_;
    SystemConfig config_;
    std::string config_path_;
    std::vector<std::unique_ptr<ErobAxis>> axes_;
    std::vector<std::unique_ptr<std::mutex>> axis_command_mutexes_;
    EthercatMasterSession master_;
    std::vector<MotorIdentity> discovered_motors_;
    std::vector<AxisBindingReport> binding_reports_;
    std::atomic<bool> degraded_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
    std::thread cycle_thread_;
    std::thread follow_thread_;
    std::thread monitor_thread_;
    std::atomic<int> wkc_miss_count_{0};
    std::atomic<int> recovery_fail_count_{0};
    std::string last_error_;
};

}  // namespace erob

#endif