#ifndef ETHERCAT_MASTER_SESSION_H_
#define ETHERCAT_MASTER_SESSION_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "ethercat.h"

#include "erob/erob_types.h"

namespace erob {

class EthercatMasterSession {
public:
    EthercatMasterSession();
    ~EthercatMasterSession();

    std::vector<AdapterInfo> scanAdapters();
    std::vector<MotorIdentity> scanMotorsOnAllAdapters();

    bool connect(const std::string& adapter_name);
    void disconnect();

    bool discoverMotors(std::vector<MotorIdentity>* motors);
    bool configurePdos();
    bool requestSafeOperational();
    bool configureDistributedClocks(int64_t cycle_ns);
    bool requestOperational();

    bool sendProcessData();
    int receiveProcessData(int timeout_us);
    bool readAxisFeedback(uint16_t slave_index, TxPdoCommon* txpdo) const;
    bool writeAxisCommand(uint16_t slave_index, const RxPdoUnified& rxpdo);

    bool sdoWriteU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value) const;
    bool sdoWriteU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value) const;
    bool sdoWriteU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value) const;
    bool sdoWriteI32(uint16_t slave, uint16_t index, uint8_t subindex, int32_t value) const;
    bool sdoReadU16(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t* value) const;
    bool sdoReadU32(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t* value) const;
    bool sdoReadU8(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t* value) const;

    bool recoverSlaves();
    std::recursive_mutex& busMutex();

    int slaveCount() const;
    int expectedWkc() const;
    int lastWkc() const;
    int slaveAlStatusCode(uint16_t slave_index) const;
    bool isConnected() const;
    bool isOperational() const;
    bool allSlavesOperational() const;
    const std::string& adapterName() const;
    std::string lastError() const;
    MotorIdentity motorIdentity(uint16_t slave_index) const;

private:
    bool ensurePreOperational();
    bool mapRxPdo(uint16_t slave) const;
    bool mapTxPdo(uint16_t slave) const;
    bool readSerialNumber(uint16_t slave, std::string* serial_number) const;
    bool isLikelyErobMotor(const ec_slavet& slave) const;
    void setLastError(const std::string& message);

    char iomap_[8192];
    std::string adapter_name_;
    std::string last_error_;
    int expected_wkc_ = 0;
    int last_wkc_ = 0;
    bool connected_ = false;
    bool operational_ = false;
    mutable std::recursive_mutex bus_mutex_;
};

}  // namespace erob

#endif