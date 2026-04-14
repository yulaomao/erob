#include "erob/erob_config.h"

#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace erob {
namespace {

std::string Trim(const std::string& input) {
    const std::string whitespace = " \t\r\n";
    const std::size_t start = input.find_first_not_of(whitespace);
    if (start == std::string::npos) {
        return "";
    }
    const std::size_t end = input.find_last_not_of(whitespace);
    return input.substr(start, end - start + 1);
}

std::string StripComments(const std::string& input) {
    const std::size_t comment = input.find('#');
    if (comment == std::string::npos) {
        return input;
    }
    return input.substr(0, comment);
}

bool ParseKeyValue(const std::string& line, std::string* key, std::string* value) {
    const std::size_t delimiter = line.find(':');
    if (delimiter == std::string::npos) {
        return false;
    }
    *key = Trim(line.substr(0, delimiter));
    *value = Trim(line.substr(delimiter + 1));
    if (!value->empty() && ((*value)[0] == '"' || (*value)[0] == '\'')) {
        *value = value->substr(1, value->size() > 1 ? value->size() - 2 : 0);
    }
    return !key->empty();
}

bool ParseFaultPolicy(const std::string& value, FaultPolicy* policy) {
    if (value == "all_stop") {
        *policy = FaultPolicy::kAllStop;
        return true;
    }
    if (value == "single_axis_stop") {
        *policy = FaultPolicy::kSingleAxisStop;
        return true;
    }
    return false;
}

std::string FaultPolicyToString(FaultPolicy policy) {
    return policy == FaultPolicy::kSingleAxisStop ? "single_axis_stop" : "all_stop";
}

bool SetAxisField(AxisConfig* axis, const std::string& key, const std::string& value) {
    if (key == "logical_axis_id") {
        axis->logical_axis_id = std::stoi(value);
        return true;
    }
    if (key == "joint_name") {
        axis->joint_name = value;
        return true;
    }
    if (key == "motor_serial") {
        axis->bound_motor.serial_number = value;
        return true;
    }
    if (key == "motor_eep_man") {
        axis->bound_motor.eep_man = static_cast<uint32_t>(std::stoul(value));
        return true;
    }
    if (key == "motor_eep_id") {
        axis->bound_motor.eep_id = static_cast<uint32_t>(std::stoul(value));
        return true;
    }
    if (key == "motor_eep_rev") {
        axis->bound_motor.eep_rev = static_cast<uint32_t>(std::stoul(value));
        return true;
    }
    if (key == "counts_per_degree") {
        axis->counts_per_degree = std::stod(value);
        return true;
    }
    if (key == "home_offset_deg") {
        axis->home_offset_deg = std::stod(value);
        return true;
    }
    if (key == "home_offset_count") {
        axis->home_offset_count = std::stoi(value);
        return true;
    }
    if (key == "profile_position_timeout_ms") {
        axis->profile_position_timeout_ms = std::stoi(value);
        return true;
    }
    if (key == "min_angle_deg") {
        axis->min_angle_deg = std::stod(value);
        return true;
    }
    if (key == "max_angle_deg") {
        axis->max_angle_deg = std::stod(value);
        return true;
    }
    if (key == "max_velocity_deg_s") {
        axis->max_velocity_deg_s = std::stod(value);
        return true;
    }
    if (key == "max_accel_deg_s2") {
        axis->max_accel_deg_s2 = std::stod(value);
        return true;
    }
    if (key == "max_decel_deg_s2") {
        axis->max_decel_deg_s2 = std::stod(value);
        return true;
    }
    return false;
}

bool SetFollowField(FollowParams* follow, const std::string& key, const std::string& value) {
    if (key == "kp") {
        follow->kp = std::stod(value);
        return true;
    }
    if (key == "kd") {
        follow->kd = std::stod(value);
        return true;
    }
    if (key == "deadband_deg") {
        follow->deadband_deg = std::stod(value);
        return true;
    }
    if (key == "max_velocity_deg_s") {
        follow->max_velocity_deg_s = std::stod(value);
        return true;
    }
    if (key == "max_accel_deg_s2") {
        follow->max_accel_deg_s2 = std::stod(value);
        return true;
    }
    if (key == "max_decel_deg_s2") {
        follow->max_decel_deg_s2 = std::stod(value);
        return true;
    }
    if (key == "target_filter_alpha") {
        follow->target_filter_alpha = std::stod(value);
        return true;
    }
    if (key == "watchdog_timeout_ms") {
        follow->watchdog_timeout_ms = std::stod(value);
        return true;
    }
    if (key == "position_limit_margin_deg") {
        follow->position_limit_margin_deg = std::stod(value);
        return true;
    }
    return false;
}

std::string JsonEscape(const std::string& input) {
    std::ostringstream stream;
    for (char ch : input) {
        switch (ch) {
        case '\\':
            stream << "\\\\";
            break;
        case '"':
            stream << "\\\"";
            break;
        case '\n':
            stream << "\\n";
            break;
        default:
            stream << ch;
            break;
        }
    }
    return stream.str();
}

bool ExtractJsonStringField(const std::string& line, const std::string& key, std::string* value) {
    const std::string token = "\"" + key + "\"";
    const std::size_t key_pos = line.find(token);
    if (key_pos == std::string::npos) {
        return false;
    }
    const std::size_t colon_pos = line.find(':', key_pos + token.size());
    const std::size_t first_quote = line.find('"', colon_pos + 1);
    const std::size_t second_quote = line.find('"', first_quote + 1);
    if (colon_pos == std::string::npos || first_quote == std::string::npos || second_quote == std::string::npos) {
        return false;
    }
    if (value != nullptr) {
        *value = line.substr(first_quote + 1, second_quote - first_quote - 1);
    }
    return true;
}

bool ExtractJsonUIntField(const std::string& line, const std::string& key, uint32_t* value) {
    const std::string token = "\"" + key + "\"";
    const std::size_t key_pos = line.find(token);
    if (key_pos == std::string::npos) {
        return false;
    }
    const std::size_t colon_pos = line.find(':', key_pos + token.size());
    if (colon_pos == std::string::npos) {
        return false;
    }

    std::size_t start = colon_pos + 1;
    while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) {
        ++start;
    }
    std::size_t end = start;
    while (end < line.size() && std::isdigit(static_cast<unsigned char>(line[end]))) {
        ++end;
    }
    if (start == end) {
        return false;
    }
    if (value != nullptr) {
        *value = static_cast<uint32_t>(std::stoul(line.substr(start, end - start)));
    }
    return true;
}

bool ExtractJsonBoolField(const std::string& line, const std::string& key, bool* value) {
    const std::string token = "\"" + key + "\"";
    const std::size_t key_pos = line.find(token);
    if (key_pos == std::string::npos) {
        return false;
    }
    const std::size_t colon_pos = line.find(':', key_pos + token.size());
    if (colon_pos == std::string::npos) {
        return false;
    }
    const std::string remainder = Trim(line.substr(colon_pos + 1));
    if (remainder.rfind("true", 0) == 0) {
        if (value != nullptr) {
            *value = true;
        }
        return true;
    }
    if (remainder.rfind("false", 0) == 0) {
        if (value != nullptr) {
            *value = false;
        }
        return true;
    }
    return false;
}

}  // namespace

bool ConfigManager::loadFromFile(const std::string& path, SystemConfig* config) const {
    if (config == nullptr) {
        return false;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        return false;
    }

    *config = DefaultSystemConfig();
    int axis_index = -1;
    bool in_system = false;
    bool in_axes = false;
    bool in_follow = false;
    std::string line;

    while (std::getline(input, line)) {
        const std::string no_comment = StripComments(line);
        const std::string trimmed = Trim(no_comment);
        if (trimmed.empty()) {
            continue;
        }

        const std::size_t indent = line.find_first_not_of(' ');
        if (trimmed == "system:") {
            in_system = true;
            in_axes = false;
            in_follow = false;
            continue;
        }
        if (trimmed == "axes:") {
            in_system = false;
            in_axes = true;
            in_follow = false;
            continue;
        }
        if (in_axes && trimmed.rfind("- ", 0) == 0) {
            ++axis_index;
            if (axis_index >= static_cast<int>(config->axes.size())) {
                config->axes.push_back(DefaultAxisConfig(axis_index));
            }
            config->axes[axis_index].logical_axis_id = axis_index;
            in_follow = false;
            const std::string remainder = Trim(trimmed.substr(2));
            if (!remainder.empty()) {
                std::string key;
                std::string value;
                if (ParseKeyValue(remainder, &key, &value)) {
                    SetAxisField(&config->axes[axis_index], key, value);
                }
            }
            continue;
        }

        std::string key;
        std::string value;
        if (!ParseKeyValue(trimmed, &key, &value)) {
            continue;
        }

        if (in_system && indent != std::string::npos && indent >= 2) {
            if (key == "preferred_adapter") {
                config->preferred_adapter = value;
            } else if (key == "ethercat_cycle_hz") {
                config->ethercat_cycle_hz = std::stoi(value);
            } else if (key == "follow_control_hz") {
                config->follow_control_hz = std::stoi(value);
            } else if (key == "follow_max_velocity_deg_s") {
                config->follow_max_velocity_deg_s = std::stod(value);
            } else if (key == "fault_policy") {
                ParseFaultPolicy(value, &config->fault_policy);
            }
            continue;
        }

        if (!in_axes || axis_index < 0 || axis_index >= static_cast<int>(config->axes.size())) {
            continue;
        }

        if (key == "follow") {
            in_follow = true;
            continue;
        }

        if (indent != std::string::npos && indent >= 6 && in_follow) {
            SetFollowField(&config->axes[axis_index].follow, key, value);
        } else {
            in_follow = false;
            SetAxisField(&config->axes[axis_index], key, value);
        }
    }

    for (AxisConfig& axis : config->axes) {
        axis.follow.control_rate_hz = static_cast<double>(config->follow_control_hz);
        axis.follow.max_velocity_deg_s = config->follow_max_velocity_deg_s;
    }

    return true;
}

bool ConfigManager::saveToFile(const std::string& path, const SystemConfig& config) const {
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << "system:\n";
    output << "  preferred_adapter: " << config.preferred_adapter << "\n";
    output << "  ethercat_cycle_hz: " << config.ethercat_cycle_hz << "\n";
    output << "  follow_control_hz: " << config.follow_control_hz << "\n";
    output << "  follow_max_velocity_deg_s: " << config.follow_max_velocity_deg_s << "\n";
    output << "  fault_policy: " << FaultPolicyToString(config.fault_policy) << "\n\n";
    output << "axes:\n";

    output << std::fixed << std::setprecision(1);
    for (const AxisConfig& axis : config.axes) {
        output << "  - logical_axis_id: " << axis.logical_axis_id << "\n";
        output << "    joint_name: " << axis.joint_name << "\n";
        output << "    motor_serial: " << axis.bound_motor.serial_number << "\n";
        output << "    motor_eep_man: " << axis.bound_motor.eep_man << "\n";
        output << "    motor_eep_id: " << axis.bound_motor.eep_id << "\n";
        output << "    motor_eep_rev: " << axis.bound_motor.eep_rev << "\n";
        output << "    counts_per_degree: " << axis.counts_per_degree << "\n";
        output << "    home_offset_deg: " << axis.home_offset_deg << "\n";
        output << "    home_offset_count: " << axis.home_offset_count << "\n";
        output << "    profile_position_timeout_ms: " << axis.profile_position_timeout_ms << "\n";
        output << "    min_angle_deg: " << axis.min_angle_deg << "\n";
        output << "    max_angle_deg: " << axis.max_angle_deg << "\n";
        output << "    max_velocity_deg_s: " << axis.max_velocity_deg_s << "\n";
        output << "    max_accel_deg_s2: " << axis.max_accel_deg_s2 << "\n";
        output << "    max_decel_deg_s2: " << axis.max_decel_deg_s2 << "\n";
        output << "    follow:\n";
        output << "      kp: " << axis.follow.kp << "\n";
        output << "      kd: " << axis.follow.kd << "\n";
        output << "      deadband_deg: " << axis.follow.deadband_deg << "\n";
        output << "      max_accel_deg_s2: " << axis.follow.max_accel_deg_s2 << "\n";
        output << "      max_decel_deg_s2: " << axis.follow.max_decel_deg_s2 << "\n";
        output << "      target_filter_alpha: " << axis.follow.target_filter_alpha << "\n";
        output << "      watchdog_timeout_ms: " << axis.follow.watchdog_timeout_ms << "\n";
        output << "      position_limit_margin_deg: " << axis.follow.position_limit_margin_deg << "\n\n";
    }

    return true;
}

bool ConfigManager::loadDiscoveryCache(
    const std::string& path,
    std::string* last_adapter,
    std::vector<MotorIdentity>* motors) const {
    if (last_adapter == nullptr || motors == nullptr) {
        return false;
    }

    std::ifstream input(path);
    if (!input.is_open()) {
        return false;
    }

    last_adapter->clear();
    motors->clear();

    bool in_motor = false;
    MotorIdentity current_motor;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = Trim(line);
        if (trimmed.empty()) {
            continue;
        }

        ExtractJsonStringField(trimmed, "last_adapter", last_adapter);

        if (trimmed == "{" && !in_motor) {
            continue;
        }

        std::string string_value;
        uint32_t uint_value = 0;
        bool bool_value = false;
        if (ExtractJsonStringField(trimmed, "serial_number", &string_value)) {
            if (in_motor) {
                motors->push_back(current_motor);
            }
            current_motor = MotorIdentity{};
            current_motor.serial_number = string_value;
            in_motor = true;
            continue;
        }
        if (!in_motor) {
            continue;
        }
        if (ExtractJsonStringField(trimmed, "adapter_name", &string_value)) {
            current_motor.adapter_name = string_value;
        } else if (ExtractJsonStringField(trimmed, "name", &string_value)) {
            current_motor.name = string_value;
        } else if (ExtractJsonUIntField(trimmed, "slave_index", &uint_value)) {
            current_motor.slave_index = static_cast<uint16_t>(uint_value);
        } else if (ExtractJsonUIntField(trimmed, "alias", &uint_value)) {
            current_motor.alias = static_cast<uint16_t>(uint_value);
        } else if (ExtractJsonUIntField(trimmed, "eep_man", &uint_value)) {
            current_motor.eep_man = uint_value;
        } else if (ExtractJsonUIntField(trimmed, "eep_id", &uint_value)) {
            current_motor.eep_id = uint_value;
        } else if (ExtractJsonUIntField(trimmed, "eep_rev", &uint_value)) {
            current_motor.eep_rev = uint_value;
        } else if (ExtractJsonBoolField(trimmed, "is_erob_motor", &bool_value)) {
            current_motor.is_erob_motor = bool_value;
        } else if (trimmed == "}" || trimmed == "},") {
            motors->push_back(current_motor);
            current_motor = MotorIdentity{};
            in_motor = false;
        }
    }

    if (in_motor) {
        motors->push_back(current_motor);
    }

    return !last_adapter->empty() || !motors->empty();
}

bool ConfigManager::saveDiscoveryCache(
    const std::string& path,
    const std::string& last_adapter,
    const std::vector<MotorIdentity>& motors) const {
    std::ofstream output(path);
    if (!output.is_open()) {
        return false;
    }

    output << "{\n";
    output << "  \"last_adapter\": \"" << JsonEscape(last_adapter) << "\",\n";
    output << "  \"motors\": [\n";
    for (std::size_t index = 0; index < motors.size(); ++index) {
        const MotorIdentity& motor = motors[index];
        output << "    {\n";
        output << "      \"serial_number\": \"" << JsonEscape(motor.serial_number) << "\",\n";
        output << "      \"adapter_name\": \"" << JsonEscape(motor.adapter_name) << "\",\n";
        output << "      \"slave_index\": " << motor.slave_index << ",\n";
        output << "      \"alias\": " << motor.alias << ",\n";
        output << "      \"eep_man\": " << motor.eep_man << ",\n";
        output << "      \"eep_id\": " << motor.eep_id << ",\n";
        output << "      \"eep_rev\": " << motor.eep_rev << ",\n";
        output << "      \"name\": \"" << JsonEscape(motor.name) << "\",\n";
        output << "      \"is_erob_motor\": " << (motor.is_erob_motor ? "true" : "false") << "\n";
        output << "    }";
        if (index + 1 != motors.size()) {
            output << ',';
        }
        output << "\n";
    }
    output << "  ]\n";
    output << "}\n";

    return true;
}

}  // namespace erob