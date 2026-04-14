#ifndef EROB_CONFIG_H_
#define EROB_CONFIG_H_

#include <string>
#include <vector>

#include "erob/erob_types.h"

namespace erob {

class ConfigManager {
public:
    bool loadFromFile(const std::string& path, SystemConfig* config) const;
    bool saveToFile(const std::string& path, const SystemConfig& config) const;
    bool loadDiscoveryCache(
        const std::string& path,
        std::string* last_adapter,
        std::vector<MotorIdentity>* motors) const;
    bool saveDiscoveryCache(
        const std::string& path,
        const std::string& last_adapter,
        const std::vector<MotorIdentity>& motors) const;
};

}  // namespace erob

#endif