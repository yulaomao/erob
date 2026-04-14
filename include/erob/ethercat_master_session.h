#ifndef ETHERCAT_MASTER_SESSION_H_
#define ETHERCAT_MASTER_SESSION_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "ethercat.h"

#include "erob/erob_types.h"

namespace erob {

class ErobArmController;

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
    std::mutex& busMutex();

    int slaveCount() const;
    int expectedWkc() const;
    int lastWkc() const;
    int slaveAlStatusCode(uint16_t slave_index) const;
    bool isConnected() const;
    bool isOperational() const;
    bool allSlavesOperational() const;
    std::string adapterName() const;
    std::string lastError() const;
    MotorIdentity motorIdentity(uint16_t slave_index) const;

private:
    friend class ErobArmController;

    bool connectNoLock(const std::string& adapter_name);
    void disconnectNoLock();
    bool discoverMotorsNoLock(std::vector<MotorIdentity>* motors);
    bool configurePdosNoLock();
    bool requestSafeOperationalNoLock();
    bool configureDistributedClocksNoLock(int64_t cycle_ns);
    bool requestOperationalNoLock();
    bool sendProcessDataNoLock();
    int receiveProcessDataNoLock(int timeout_us);
    bool readAxisFeedbackNoLock(uint16_t slave_index, TxPdoCommon* txpdo) const;
    bool writeAxisCommandNoLock(uint16_t slave_index, const RxPdoUnified& rxpdo);
    bool sdoWriteU8NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t value) const;
    bool sdoWriteU16NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t value) const;
    bool sdoWriteU32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t value) const;
    bool sdoWriteI32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, int32_t value) const;
    bool sdoReadU16NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint16_t* value) const;
    bool sdoReadU32NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint32_t* value) const;
    bool sdoReadU8NoLock(uint16_t slave, uint16_t index, uint8_t subindex, uint8_t* value) const;
    bool recoverSlavesNoLock();
    int slaveCountNoLock() const;
    int expectedWkcNoLock() const;
    int lastWkcNoLock() const;
    int slaveAlStatusCodeNoLock(uint16_t slave_index) const;
    bool isConnectedNoLock() const;
    bool isOperationalNoLock() const;
    bool allSlavesOperationalNoLock() const;
    MotorIdentity motorIdentityNoLock(uint16_t slave_index) const;
    bool ensurePreOperationalNoLock();
    bool mapRxPdoNoLock(uint16_t slave) const;
    bool mapTxPdoNoLock(uint16_t slave) const;
    bool readSerialNumberNoLock(uint16_t slave, std::string* serial_number) const;
    bool isLikelyErobMotor(const ec_slavet& slave) const;
    void setLastErrorNoLock(const std::string& message);

    char iomap_[8192];
    std::string adapter_name_;
    std::string last_error_;
    int expected_wkc_ = 0;
    int last_wkc_ = 0;
    bool connected_ = false;
    bool operational_ = false;
    mutable std::mutex bus_mutex_;
};

}  // namespace erob

#endif