#include "erob/ethercat_master_session.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <iostream>
#include <sstream>

#include "ethercatprint.h"
#include "ethercatconfig.h"

namespace erob {
namespace {

constexpr int kMonitorTimeoutUs = 5000;

std::string ToLowerCopy(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return value;
}

bool ProbeAdapter(const std::string& adapter_name, int* discovered_slave_count) {
    EthercatMasterSession probe;
    if (!probe.connect(adapter_name)) {
        return false;
    }

    std::vector<MotorIdentity> motors;
    const bool discovered = probe.discoverMotors(&motors);
    if (discovered_slave_count != nullptr) {
        *discovered_slave_count = discovered ? static_cast<int>(motors.size()) : 0;
    }
    probe.disconnect();
    return discovered;
}

const char* EthercatStateName(uint16_t state) {
    switch (state & 0x0F) {
    case EC_STATE_INIT:
        return "INIT";
    case EC_STATE_PRE_OP:
        return "PRE_OP";
    case EC_STATE_BOOT:
        return "BOOT";
    case EC_STATE_SAFE_OP:
        return "SAFE_OP";
    case EC_STATE_OPERATIONAL:
        return "OPERATIONAL";
    default:
        return "UNKNOWN";
    }
}

std::string FormatSlaveDiagnostics() {
    std::ostringstream stream;
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (slave != 1) {
            stream << " ; ";
        }
        const uint16_t state = ec_slave[slave].state;
        stream << "slave " << slave
               << " (" << ec_slave[slave].name << ")"
               << ": state=0x" << std::hex << state << std::dec
               << " " << EthercatStateName(state);
        if ((state & EC_STATE_ERROR) != 0) {
            stream << "+ERROR";
        }
        stream << ", AL=0x" << std::hex << ec_slave[slave].ALstatuscode << std::dec
               << " " << ec_ALstatuscode2string(ec_slave[slave].ALstatuscode);
    }
    return stream.str();
}

std::string DisplayAdapterName(const std::string& adapter_name) {
    return adapter_name.empty() ? std::string("-") : adapter_name;
}

std::string HexState(uint16_t state) {
    std::ostringstream stream;
    stream << std::hex << state;
    return stream.str();
}

void LogMasterDebug(
    const std::string& adapter_name,
    const std::string& stage,
    const std::string& message) {
    static std::mutex log_mutex;
    std::lock_guard<std::mutex> lock(log_mutex);
    std::cerr << "[erob-master-debug] adapter=" << DisplayAdapterName(adapter_name)
              << " stage=" << stage
              << " | " << message << std::endl;
}

template <typename T>
bool SdoWrite(uint16_t slave, uint16_t index, uint8_t subindex, const T& value) {
    T local = value;
    return ec_SDOwrite(slave, index, subindex, FALSE, sizeof(local), &local, EC_TIMEOUTSAFE) > 0;
}

template <typename T>
bool SdoRead(uint16_t slave, uint16_t index, uint8_t subindex, T* value) {
    int size = sizeof(T);
    return value != nullptr &&
        ec_SDOread(slave, index, subindex, FALSE, &size, value, EC_TIMEOUTSAFE) > 0;
}

}  // namespace

EthercatMasterSession::EthercatMasterSession() {
    std::memset(iomap_, 0, sizeof(iomap_));
}

EthercatMasterSession::~EthercatMasterSession() {
    disconnect();
}

std::vector<AdapterInfo> EthercatMasterSession::scanAdapters() {
    std::vector<AdapterInfo> adapters;
    ec_adaptert* list = ec_find_adapters();
    for (ec_adaptert* adapter = list; adapter != nullptr; adapter = adapter->next) {
        if (std::strcmp(adapter->name, "lo") == 0) {
            continue;
        }
        AdapterInfo info;
        info.name = adapter->name;
        info.description = adapter->desc;
        LogMasterDebug(info.name, "scan/probe", "begin adapter probe");
        info.scan_success = ProbeAdapter(info.name, &info.discovered_slave_count);
        LogMasterDebug(
            info.name,
            "scan/probe",
            std::string(info.scan_success ? "probe succeeded" : "probe failed") +
                " | discovered_slave_count=" + std::to_string(info.discovered_slave_count));
        adapters.push_back(info);
    }
    ec_free_adapters(list);
    return adapters;
}

std::vector<MotorIdentity> EthercatMasterSession::scanMotorsOnAllAdapters() {
    std::vector<MotorIdentity> all_motors;
    const std::vector<AdapterInfo> adapters = scanAdapters();
    for (const AdapterInfo& adapter : adapters) {
        if (!connect(adapter.name)) {
            continue;
        }
        std::vector<MotorIdentity> motors;
        if (discoverMotors(&motors)) {
            all_motors.insert(all_motors.end(), motors.begin(), motors.end());
        }
        disconnect();
    }
    return all_motors;
}

bool EthercatMasterSession::connect(const std::string& adapter_name) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return connectNoLock(adapter_name);
}

void EthercatMasterSession::disconnect() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    disconnectNoLock();
}

bool EthercatMasterSession::discoverMotors(std::vector<MotorIdentity>* motors) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return discoverMotorsNoLock(motors);
}

bool EthercatMasterSession::configurePdos() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return configurePdosNoLock();
}

bool EthercatMasterSession::requestSafeOperational() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return requestSafeOperationalNoLock();
}

bool EthercatMasterSession::configureDistributedClocks(int64_t cycle_ns) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return configureDistributedClocksNoLock(cycle_ns);
}

bool EthercatMasterSession::requestOperational() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return requestOperationalNoLock();
}

bool EthercatMasterSession::sendProcessData() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sendProcessDataNoLock();
}

int EthercatMasterSession::receiveProcessData(int timeout_us) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return receiveProcessDataNoLock(timeout_us);
}

bool EthercatMasterSession::readAxisFeedback(uint16_t slave_index, TxPdoCommon* txpdo) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return readAxisFeedbackNoLock(slave_index, txpdo);
}

bool EthercatMasterSession::writeAxisCommand(uint16_t slave_index, const RxPdoUnified& rxpdo) {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return writeAxisCommandNoLock(slave_index, rxpdo);
}

bool EthercatMasterSession::sdoWriteU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoWriteU8NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoWriteU16NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoWriteU32NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteI32(uint16_t slave, uint16_t index, uint8_t subindex, int32_t value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoWriteI32NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t* value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoReadU16NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t* value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoReadU32NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t* value) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return sdoReadU8NoLock(slave, index, subindex, value);
}

bool EthercatMasterSession::recoverSlaves() {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return recoverSlavesNoLock();
}

std::mutex& EthercatMasterSession::busMutex() {
    return bus_mutex_;
}

int EthercatMasterSession::slaveCount() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return slaveCountNoLock();
}

int EthercatMasterSession::expectedWkc() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return expectedWkcNoLock();
}

int EthercatMasterSession::lastWkc() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return lastWkcNoLock();
}

int EthercatMasterSession::slaveAlStatusCode(uint16_t slave_index) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return slaveAlStatusCodeNoLock(slave_index);
}

bool EthercatMasterSession::isConnected() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return isConnectedNoLock();
}

bool EthercatMasterSession::isOperational() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return isOperationalNoLock();
}

bool EthercatMasterSession::allSlavesOperational() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return allSlavesOperationalNoLock();
}

std::string EthercatMasterSession::adapterName() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return adapter_name_;
}

std::string EthercatMasterSession::lastError() const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return last_error_;
}

MotorIdentity EthercatMasterSession::motorIdentity(uint16_t slave_index) const {
    std::lock_guard<std::mutex> lock(bus_mutex_);
    return motorIdentityNoLock(slave_index);
}

bool EthercatMasterSession::connectNoLock(const std::string& adapter_name) {
    disconnectNoLock();
    if (adapter_name.empty()) {
        setLastErrorNoLock("adapter name is empty");
        return false;
    }
    LogMasterDebug(adapter_name, "connect/raw-socket", "begin ec_init(adapter)");
    const int init_result = ec_init(adapter_name.c_str());
    if (init_result <= 0) {
        LogMasterDebug(adapter_name, "connect/raw-socket", "ec_init failed");
        setLastErrorNoLock("connect(raw-socket) failed to initialize EtherCAT master on adapter " + adapter_name);
        return false;
    }
    adapter_name_ = adapter_name;
    connected_ = true;
    operational_ = false;
    LogMasterDebug(adapter_name_, "connect/raw-socket", "ec_init succeeded; raw socket/session ready");
    return true;
}

void EthercatMasterSession::disconnectNoLock() {
    if (connected_) {
        LogMasterDebug(adapter_name_, "disconnect/clear", "request INIT begin");
        ec_slave[0].state = EC_STATE_INIT;
        ec_writestate(0);
        ec_readstate();
        LogMasterDebug(
            adapter_name_,
            "disconnect/clear",
            "request INIT sent | master_state=0x" + HexState(ec_slave[0].state));
        ec_close();
        LogMasterDebug(adapter_name_, "disconnect/clear", "ec_close completed");
    }
    adapter_name_.clear();
    connected_ = false;
    operational_ = false;
    expected_wkc_ = 0;
    last_wkc_ = 0;
    std::memset(iomap_, 0, sizeof(iomap_));
}

bool EthercatMasterSession::discoverMotorsNoLock(std::vector<MotorIdentity>* motors) {
    if (!connected_) {
        setLastErrorNoLock("EtherCAT master is not connected");
        return false;
    }
    if (motors == nullptr) {
        setLastErrorNoLock("motors output is null");
        return false;
    }

    motors->clear();
    LogMasterDebug(adapter_name_, "discover/ec_config_init", "begin ec_config_init(FALSE)");
    const int slave_count = ec_config_init(FALSE);
    if (slave_count <= 0) {
        LogMasterDebug(adapter_name_, "discover/ec_config_init", "ec_config_init returned no slaves");
        setLastErrorNoLock("discover(ec_config_init(FALSE)) found no EtherCAT slaves on adapter " + adapter_name_);
        return false;
    }

    ec_readstate();
    LogMasterDebug(
        adapter_name_,
        "discover/ec_config_init",
        "ec_config_init succeeded | slave_count=" + std::to_string(slave_count) +
            " | diagnostics=" + FormatSlaveDiagnostics());
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        motors->push_back(motorIdentityNoLock(static_cast<uint16_t>(slave)));
    }

    return true;
}

bool EthercatMasterSession::configurePdosNoLock() {
    LogMasterDebug(adapter_name_, "pdo-map", "begin configure PDO mapping");
    if (!ensurePreOperationalNoLock()) {
        return false;
    }

    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (!mapRxPdoNoLock(static_cast<uint16_t>(slave)) ||
            !mapTxPdoNoLock(static_cast<uint16_t>(slave))) {
            std::ostringstream message;
            message << "failed to configure PDO for slave " << slave;
            LogMasterDebug(adapter_name_, "pdo-map", message.str());
            setLastErrorNoLock(message.str());
            return false;
        }
    }

    ecx_context.manualstatechange = 1;
    ec_config_map(iomap_);
    expected_wkc_ = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    LogMasterDebug(
        adapter_name_,
        "pdo-map",
        "PDO mapping complete | expected_wkc=" + std::to_string(expected_wkc_));
    return true;
}

bool EthercatMasterSession::requestSafeOperationalNoLock() {
    if (!connected_) {
        setLastErrorNoLock("EtherCAT master is not connected");
        return false;
    }

    LogMasterDebug(adapter_name_, "state-switch/SAFE_OP", "begin request SAFE_OP");
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            ec_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ec_writestate(static_cast<uint16_t>(slave));
        }
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    if (ec_statecheck(0, EC_STATE_SAFE_OP, 5 * EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
        ec_readstate();
        LogMasterDebug(
            adapter_name_,
            "state-switch/SAFE_OP",
            "failed | diagnostics=" + FormatSlaveDiagnostics());
        setLastErrorNoLock("failed to switch slaves to SAFE_OP | " + FormatSlaveDiagnostics());
        return false;
    }
    LogMasterDebug(adapter_name_, "state-switch/SAFE_OP", "success");
    return true;
}

bool EthercatMasterSession::configureDistributedClocksNoLock(int64_t cycle_ns) {
    if (!connected_) {
        setLastErrorNoLock("EtherCAT master is not connected");
        return false;
    }

    if (cycle_ns <= 0) {
        LogMasterDebug(adapter_name_, "distributed-clocks", "disable DC sync on all slaves");
        for (int slave = 1; slave <= ec_slavecount; ++slave) {
            ec_dcsync0(slave, FALSE, 0, 0);
        }
        return true;
    }

    LogMasterDebug(
        adapter_name_,
        "distributed-clocks",
        "begin configure DC | cycle_ns=" + std::to_string(cycle_ns));
    ec_configdc();
    int dc_slave_count = 0;
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (ec_slave[slave].hasdc) {
            ec_dcsync0(slave, TRUE, cycle_ns, 0);
            ++dc_slave_count;
        }
    }
    LogMasterDebug(
        adapter_name_,
        "distributed-clocks",
        "DC configured | dc_slave_count=" + std::to_string(dc_slave_count));
    return true;
}

bool EthercatMasterSession::requestOperationalNoLock() {
    if (!connected_) {
        setLastErrorNoLock("EtherCAT master is not connected");
        return false;
    }

    LogMasterDebug(adapter_name_, "state-switch/OPERATIONAL", "begin request OPERATIONAL");
    RxPdoUnified safe_pdo{};
    safe_pdo.controlword = 0x0000;
    safe_pdo.target_position = 0;
    safe_pdo.target_velocity = 0;
    safe_pdo.mode_of_operation = MotionModeToCia402Value(MotionMode::kProfilePosition);
    safe_pdo.padding = 0;

    auto write_safe_outputs = [&]() {
        for (int slave = 1; slave <= ec_slavecount; ++slave) {
            if (ec_slave[slave].outputs == nullptr ||
                ec_slave[slave].Obytes < static_cast<int>(sizeof(RxPdoUnified))) {
                continue;
            }
            std::memcpy(ec_slave[slave].outputs, &safe_pdo, sizeof(safe_pdo));
        }
    };

    auto prime_safe_process_data = [&](int cycles) {
        for (int cycle = 0; cycle < cycles; ++cycle) {
            write_safe_outputs();
            ec_send_processdata();
            last_wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
        }
    };

    constexpr int kPrimeCyclesBeforeOp = 20;
    constexpr int kPrimeCyclesAfterOp = 10;
    constexpr int kOperationalRequestAttempts = 3;

    for (int attempt = 1; attempt <= kOperationalRequestAttempts; ++attempt) {
        ec_readstate();
        for (int slave = 1; slave <= ec_slavecount; ++slave) {
            if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                ec_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
                ec_writestate(static_cast<uint16_t>(slave));
            }
        }

        ec_slave[0].state = EC_STATE_SAFE_OP;
        ec_writestate(0);
        ec_statecheck(0, EC_STATE_SAFE_OP, 5 * EC_TIMEOUTSTATE);

        prime_safe_process_data(kPrimeCyclesBeforeOp);

        ec_slave[0].state = EC_STATE_OPERATIONAL;
        ec_writestate(0);

        int retries = 200;
        while (retries-- > 0) {
            write_safe_outputs();
            ec_send_processdata();
            last_wkc_ = ec_receive_processdata(EC_TIMEOUTRET);
            if (ec_statecheck(0, EC_STATE_OPERATIONAL, 50000) == EC_STATE_OPERATIONAL) {
                operational_ = true;
                prime_safe_process_data(kPrimeCyclesAfterOp);
                LogMasterDebug(
                    adapter_name_,
                    "state-switch/OPERATIONAL",
                    "success | op_attempt=" + std::to_string(attempt) +
                        " | expected_wkc=" + std::to_string(expected_wkc_) +
                        " | last_wkc=" + std::to_string(last_wkc_));
                return true;
            }
        }

        ec_readstate();
        LogMasterDebug(
            adapter_name_,
            "state-switch/OPERATIONAL",
            "retry required | op_attempt=" + std::to_string(attempt) +
                " | diagnostics=" + FormatSlaveDiagnostics());
    }

    if (ec_slave[0].state != EC_STATE_OPERATIONAL) {
        ec_readstate();
        std::ostringstream stream;
        stream << "failed to switch slaves to OPERATIONAL"
               << " | expected_wkc=" << expected_wkc_
               << ", last_wkc=" << last_wkc_
               << " | " << FormatSlaveDiagnostics();
        LogMasterDebug(adapter_name_, "state-switch/OPERATIONAL", stream.str());
        setLastErrorNoLock(stream.str());
        operational_ = false;
        return false;
    }

    operational_ = true;
    LogMasterDebug(adapter_name_, "state-switch/OPERATIONAL", "success without retries exhausted");
    return true;
}

bool EthercatMasterSession::sendProcessDataNoLock() {
    if (!operational_) {
        return false;
    }
    ec_send_processdata();
    return true;
}

int EthercatMasterSession::receiveProcessDataNoLock(int timeout_us) {
    if (!operational_) {
        return 0;
    }
    last_wkc_ = ec_receive_processdata(timeout_us);
    return last_wkc_;
}

bool EthercatMasterSession::readAxisFeedbackNoLock(uint16_t slave_index, TxPdoCommon* txpdo) const {
    if (!operational_ || txpdo == nullptr || slave_index == 0 || slave_index > ec_slavecount) {
        return false;
    }
    if (ec_slave[slave_index].inputs == nullptr ||
        ec_slave[slave_index].Ibytes < static_cast<int>(sizeof(TxPdoCommon))) {
        return false;
    }
    std::memcpy(txpdo, ec_slave[slave_index].inputs, sizeof(TxPdoCommon));
    return true;
}

bool EthercatMasterSession::writeAxisCommandNoLock(uint16_t slave_index, const RxPdoUnified& rxpdo) {
    if (!connected_ || slave_index == 0 || slave_index > ec_slavecount) {
        return false;
    }
    if (ec_slave[slave_index].outputs == nullptr ||
        ec_slave[slave_index].Obytes < static_cast<int>(sizeof(RxPdoUnified))) {
        return false;
    }
    std::memcpy(ec_slave[slave_index].outputs, &rxpdo, sizeof(RxPdoUnified));
    return true;
}

bool EthercatMasterSession::sdoWriteU8NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value) const {
    return SdoWrite(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteU16NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value) const {
    return SdoWrite(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteU32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value) const {
    return SdoWrite(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoWriteI32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, int32_t value) const {
    return SdoWrite(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU16NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t* value) const {
    return SdoRead(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t* value) const {
    return SdoRead(slave, index, subindex, value);
}

bool EthercatMasterSession::sdoReadU8NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t* value) const {
    return SdoRead(slave, index, subindex, value);
}

bool EthercatMasterSession::recoverSlavesNoLock() {
    if (!connected_) {
        return false;
    }

    bool recovered_any = false;
    ec_group[0].docheckstate = FALSE;
    ec_readstate();
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (ec_slave[slave].state == EC_STATE_OPERATIONAL) {
            continue;
        }
        ec_group[0].docheckstate = TRUE;
        if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
            ec_slave[slave].state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ec_writestate(static_cast<uint16_t>(slave));
            recovered_any = true;
        } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(static_cast<uint16_t>(slave));
            recovered_any = true;
        } else if (ec_slave[slave].state > EC_STATE_NONE) {
            if (ec_reconfig_slave(static_cast<uint16_t>(slave), kMonitorTimeoutUs)) {
                ec_slave[slave].islost = FALSE;
                recovered_any = true;
            }
        } else if (!ec_slave[slave].islost) {
            ec_statecheck(static_cast<uint16_t>(slave), EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (ec_slave[slave].state == EC_STATE_NONE) {
                ec_slave[slave].islost = TRUE;
            }
        }

        if (ec_slave[slave].islost) {
            if (ec_slave[slave].state == EC_STATE_NONE) {
                if (ec_recover_slave(static_cast<uint16_t>(slave), kMonitorTimeoutUs)) {
                    ec_slave[slave].islost = FALSE;
                    recovered_any = true;
                }
            } else {
                ec_slave[slave].islost = FALSE;
                recovered_any = true;
            }
        }
    }
    return recovered_any;
}

int EthercatMasterSession::slaveCountNoLock() const {
    return connected_ ? ec_slavecount : 0;
}

int EthercatMasterSession::expectedWkcNoLock() const {
    return expected_wkc_;
}

int EthercatMasterSession::lastWkcNoLock() const {
    return last_wkc_;
}

int EthercatMasterSession::slaveAlStatusCodeNoLock(uint16_t slave_index) const {
    if (!connected_ || slave_index == 0 || slave_index > ec_slavecount) {
        return 0;
    }
    return ec_slave[slave_index].ALstatuscode;
}

bool EthercatMasterSession::isConnectedNoLock() const {
    return connected_;
}

bool EthercatMasterSession::isOperationalNoLock() const {
    return operational_;
}

bool EthercatMasterSession::allSlavesOperationalNoLock() const {
    if (!connected_) {
        return false;
    }
    for (int slave = 1; slave <= ec_slavecount; ++slave) {
        if (ec_slave[slave].state != EC_STATE_OPERATIONAL) {
            return false;
        }
    }
    return true;
}

MotorIdentity EthercatMasterSession::motorIdentityNoLock(uint16_t slave_index) const {
    MotorIdentity identity;
    if (!connected_ || slave_index == 0 || slave_index > ec_slavecount) {
        return identity;
    }

    const ec_slavet& slave = ec_slave[slave_index];
    identity.adapter_name = adapter_name_;
    identity.slave_index = slave_index;
    identity.alias = slave.aliasadr;
    identity.eep_man = slave.eep_man;
    identity.eep_id = slave.eep_id;
    identity.eep_rev = slave.eep_rev;
    identity.name = slave.name;
    identity.is_erob_motor = isLikelyErobMotor(slave);
    readSerialNumberNoLock(slave_index, &identity.serial_number);
    return identity;
}

bool EthercatMasterSession::ensurePreOperationalNoLock() {
    if (!connected_) {
        setLastErrorNoLock("EtherCAT master is not connected");
        return false;
    }
    LogMasterDebug(adapter_name_, "state-switch/PRE_OP", "begin request PRE_OP");
    ec_readstate();
    ec_slave[0].state = EC_STATE_PRE_OP;
    ec_writestate(0);
    if (ec_statecheck(0, EC_STATE_PRE_OP, 3 * EC_TIMEOUTSTATE) != EC_STATE_PRE_OP) {
        ec_readstate();
        LogMasterDebug(
            adapter_name_,
            "state-switch/PRE_OP",
            "failed | diagnostics=" + FormatSlaveDiagnostics());
        setLastErrorNoLock("failed to switch slaves to PRE_OP | " + FormatSlaveDiagnostics());
        return false;
    }
    LogMasterDebug(adapter_name_, "state-switch/PRE_OP", "success");
    return true;
}

bool EthercatMasterSession::mapRxPdoNoLock(uint16_t slave) const {
    const uint8_t zero_map = 0;
    if (!sdoWriteU8NoLock(slave, 0x1600, 0x00, zero_map)) {
        return false;
    }

    const uint32_t controlword = 0x60400010;
    const uint32_t target_position = 0x607A0020;
    const uint32_t target_velocity = 0x60FF0020;
    const uint32_t mode = 0x60600008;
    const uint32_t padding = 0x00000008;
    const uint8_t map_count = 5;
    const uint16_t clear = 0x0000;
    const uint16_t assignment = 0x1600;
    const uint16_t assign_count = 0x0001;

    return SdoWrite(slave, 0x1600, 0x01, controlword) &&
        SdoWrite(slave, 0x1600, 0x02, target_position) &&
        SdoWrite(slave, 0x1600, 0x03, target_velocity) &&
        SdoWrite(slave, 0x1600, 0x04, mode) &&
        SdoWrite(slave, 0x1600, 0x05, padding) &&
        sdoWriteU8NoLock(slave, 0x1600, 0x00, map_count) &&
        sdoWriteU16NoLock(slave, 0x1C12, 0x00, clear) &&
        sdoWriteU16NoLock(slave, 0x1C12, 0x01, assignment) &&
        sdoWriteU16NoLock(slave, 0x1C12, 0x00, assign_count);
}

bool EthercatMasterSession::mapTxPdoNoLock(uint16_t slave) const {
    const uint8_t zero_map = 0;
    if (!sdoWriteU8NoLock(slave, 0x1A00, 0x00, zero_map)) {
        return false;
    }

    const uint32_t statusword = 0x60410010;
    const uint32_t actual_position = 0x60640020;
    const uint32_t actual_velocity = 0x606C0020;
    const uint32_t actual_torque = 0x60770010;
    const uint32_t mode_display = 0x60610008;
    const uint32_t padding = 0x00000008;
    const uint8_t map_count = 6;
    const uint16_t clear = 0x0000;
    const uint16_t assignment = 0x1A00;
    const uint16_t assign_count = 0x0001;

    return SdoWrite(slave, 0x1A00, 0x01, statusword) &&
        SdoWrite(slave, 0x1A00, 0x02, actual_position) &&
        SdoWrite(slave, 0x1A00, 0x03, actual_velocity) &&
        SdoWrite(slave, 0x1A00, 0x04, actual_torque) &&
        SdoWrite(slave, 0x1A00, 0x05, mode_display) &&
        SdoWrite(slave, 0x1A00, 0x06, padding) &&
        sdoWriteU8NoLock(slave, 0x1A00, 0x00, map_count) &&
        sdoWriteU16NoLock(slave, 0x1C13, 0x00, clear) &&
        sdoWriteU16NoLock(slave, 0x1C13, 0x01, assignment) &&
        sdoWriteU16NoLock(slave, 0x1C13, 0x00, assign_count);
}

bool EthercatMasterSession::readSerialNumberNoLock(uint16_t slave, std::string* serial_number) const {
    if (serial_number == nullptr) {
        return false;
    }
    uint32_t serial = 0;
    if (!sdoReadU32NoLock(slave, 0x1018, 0x04, &serial) || serial == 0) {
        serial_number->clear();
        return false;
    }
    std::ostringstream stream;
    stream << "EROB-" << std::hex << serial;
    *serial_number = stream.str();
    return true;
}

bool EthercatMasterSession::isLikelyErobMotor(const ec_slavet& slave) const {
    const std::string lower_name = ToLowerCopy(slave.name);
    return lower_name.find("erob") != std::string::npos || lower_name.find("servo") != std::string::npos;
}

void EthercatMasterSession::setLastErrorNoLock(const std::string& message) {
    last_error_ = message;
}

}  // namespace erob