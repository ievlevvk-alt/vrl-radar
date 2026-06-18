// src/core/config.cpp
#include "vrl/radar/core/config.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>

namespace vrl {
namespace radar {

// ============================================================================
// CONFIG PARSER IMPLEMENTATION
// ============================================================================

bool ConfigParser::load(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Cannot open config file: " << filename << std::endl;
        return false;
    }
    
    sections_.clear();
    current_section_ = "";
    line_number_ = 0;
    
    std::string line;
    while (std::getline(file, line)) {
        line_number_++;
        
        // Удаляем BOM
        if (line.length() >= 3 && 
            static_cast<unsigned char>(line[0]) == 0xEF &&
            static_cast<unsigned char>(line[1]) == 0xBB &&
            static_cast<unsigned char>(line[2]) == 0xBF) {
            line = line.substr(3);
        }
        
        // Удаляем комментарии
        size_t comment_pos = std::string::npos;
        bool in_quotes = false;
        
        for (size_t i = 0; i < line.length(); ++i) {
            char c = line[i];
            if (c == '"' || c == '\'') {
                in_quotes = !in_quotes;
            } else if (c == '#' && !in_quotes) {
                comment_pos = i;
                break;
            }
        }
        
        if (comment_pos != std::string::npos) {
            line = line.substr(0, comment_pos);
        }
        
        line = trim(line);
        if (line.empty()) continue;
        
        if (line.front() == '[' && line.back() == ']') {
            current_section_ = line.substr(1, line.length() - 2);
            current_section_ = trim(current_section_);
            sections_[current_section_] = {};
            continue;
        }
        
        size_t eq_pos = line.find('=');
        if (eq_pos != std::string::npos) {
            std::string key = trim(line.substr(0, eq_pos));
            std::string value = trim(line.substr(eq_pos + 1));
            
            if (key.empty()) continue;
            
            if (!current_section_.empty()) {
                sections_[current_section_][key] = value;
            } else {
                sections_[""][key] = value;
            }
        }
    }
    
    return true;
}

std::optional<std::string> ConfigParser::get_string(const std::string& key, 
                                                     const std::string& section) const {
    auto it = sections_.find(section);
    if (it == sections_.end()) return std::nullopt;
    
    auto val_it = it->second.find(key);
    if (val_it == it->second.end()) return std::nullopt;
    
    std::string value = val_it->second;
    if (value.length() >= 2) {
        if ((value.front() == '"' && value.back() == '"') ||
            (value.front() == '\'' && value.back() == '\'')) {
            value = value.substr(1, value.length() - 2);
        }
    }
    return trim(value);
}

std::vector<std::string> ConfigParser::get_sections() const {
    std::vector<std::string> result;
    for (const auto& [section, _] : sections_) {
        if (!section.empty()) {
            result.push_back(section);
        }
    }
    return result;
}

void ConfigParser::parse_target_field(GeneratedTarget& target, 
                                      const std::string& key, 
                                      const std::string& value) const {
    if (key == "name") {
        target.name = value;
    } else if (key == "azimuth_deg") {
        auto val = safe_stod<double>(value);
        if (val) target.azimuth_deg = *val;
    } else if (key == "range_km") {
        auto val = safe_stod<double>(value);
        if (val) target.range_km = *val;
    } else if (key == "rbs_code_octal") {
        try {
            target.rbs_code_octal = static_cast<uint16_t>(std::stoul(value, nullptr, 8));
        } catch (...) {
            target.rbs_code_octal = 0;
        }
    } else if (key == "uvd_data_dec") {
        auto val = safe_stod<unsigned int>(value);
        if (val) target.uvd_data_dec = *val;
    } else if (key == "azimuth_speed_deg_per_rev") {
        auto val = safe_stod<double>(value);
        if (val) target.azimuth_speed_deg_per_rev = *val;
    } else if (key == "range_speed_km_per_rev") {
        auto val = safe_stod<double>(value);
        if (val) target.range_speed_km_per_rev = *val;
    } else if (key == "spi") {
        auto val = safe_stod<bool>(value);
        if (val) target.spi = *val;
    } else if (key == "enabled") {
        auto val = safe_stod<bool>(value);
        if (val) target.enabled = *val;
    } else if (key == "update_every_n_revolutions") {
        auto val = safe_stod<int>(value);
        if (val) target.update_every_n_revolutions = *val;
    } else if (key == "revolution_offset") {
        auto val = safe_stod<int>(value);
        if (val) target.revolution_offset = *val;
    } else if (key == "altitude_meters") {
        auto val = safe_stod<int>(value);
        if (val) target.altitude_meters = *val;
    } else if (key == "enable_altitude") {
        auto val = safe_stod<bool>(value);
        if (val) target.enable_altitude = *val;
    } else if (key == "alternate_code_altitude") {
        auto val = safe_stod<bool>(value);
        if (val) target.alternate_code_altitude = *val;
    } else if (key == "alternate_data_altitude") {
        auto val = safe_stod<bool>(value);
        if (val) target.alternate_data_altitude = *val;
    } else if (key == "use_linear_motion") {
        auto val = safe_stod<bool>(value);
        if (val) target.use_linear_motion = *val;
    } else if (key == "speed_m_per_s") {
        auto val = safe_stod<double>(value);
        if (val) target.speed_m_per_s = *val;
    } else if (key == "course_deg") {
        auto val = safe_stod<double>(value);
        if (val) target.course_deg = *val;
    } else if (key == "initial_x_km") {
        auto val = safe_stod<double>(value);
        if (val) target.initial_x_km = *val;
    } else if (key == "initial_y_km") {
        auto val = safe_stod<double>(value);
        if (val) target.initial_y_km = *val;
    }
}

std::vector<GeneratedTarget> ConfigParser::parse_targets(const std::string& section) const {
    std::vector<GeneratedTarget> targets;
    
    auto it = sections_.find(section);
    if (it == sections_.end()) return targets;
    
    GeneratedTarget current;
    bool in_target = false;
    
    for (const auto& [key, value] : it->second) {
        if (key == "end") {
            if (in_target && !current.name.empty()) {
                targets.push_back(std::move(current));
                current = GeneratedTarget{};
                in_target = false;
            }
            continue;
        }
        
        in_target = true;
        parse_target_field(current, key, value);
    }
    
    if (in_target && !current.name.empty()) {
        targets.push_back(std::move(current));
    }
    
    return targets;
}

// ============================================================================
// GENERATED TARGET METHODS
// ============================================================================

uint16_t GeneratedTarget::get_mode_c_code() const {
    int altitude_100ft = static_cast<int>(altitude_meters / 30.48);
    if (altitude_100ft < -12) altitude_100ft = -12;
    if (altitude_100ft > 1267) altitude_100ft = 1267;
    
    int altitude_100ft_offset = altitude_100ft + 12;
    int hundreds = (altitude_100ft_offset / 100) % 10;
    int tens = (altitude_100ft_offset / 10) % 10;
    int units = altitude_100ft_offset % 10;
    
    uint8_t units_gray = units ^ (units >> 1);
    uint8_t tens_gray = tens ^ (tens >> 1);
    uint8_t hundreds_gray = hundreds ^ (hundreds >> 1);
    
    uint16_t mode_c_code = 0;
    if (units_gray & 0x01) mode_c_code |= (1 << 0);
    if (units_gray & 0x02) mode_c_code |= (1 << 1);
    if (units_gray & 0x04) mode_c_code |= (1 << 2);
    if (hundreds_gray & 0x01) mode_c_code |= (1 << 3);
    if (hundreds_gray & 0x02) mode_c_code |= (1 << 4);
    if (hundreds_gray & 0x04) mode_c_code |= (1 << 5);
    if (tens_gray & 0x01) mode_c_code |= (1 << 6);
    if (tens_gray & 0x02) mode_c_code |= (1 << 7);
    if (tens_gray & 0x04) mode_c_code |= (1 << 8);
    
    return mode_c_code;
}

bool GeneratedTarget::get_current_rbs_reply(uint16_t& code, bool& spi_out) const {
    if (enable_altitude && alternate_code_altitude) {
        if (current_mode == 0) {
            code = get_rbs_code();
            spi_out = spi;
            return true;
        } else {
            code = get_mode_c_code();
            spi_out = false;
            return false;
        }
    }
    code = get_rbs_code();
    spi_out = spi;
    return true;
}

uint32_t GeneratedTarget::get_current_uvd_data() const {
    if (!enable_altitude || !alternate_data_altitude) {
        return uvd_data_dec & 0x0FFFFF;
    }
    
    if (uvd_current_mode == 0) {
        return uvd_data_dec & 0x0FFFFF;
    } else {
        int alt = altitude_meters;
        if (alt < -1000) alt = -1000;
        if (alt > 126750) alt = 126750;
        
        int alt_dam = alt / 10;
        if (alt_dam < 0) alt_dam = 0;
        uint16_t altitude_dam = static_cast<uint16_t>(alt_dam & 0x1FFF);
        uint8_t q_code = 1;
        
        uint32_t result = 0;
        result |= (altitude_dam & 0x1FFF);
        result |= (q_code & 0x03) << 13;
        return result & 0x0FFFFF;
    }
}

void GeneratedTarget::update_linear_position(double time_delta_seconds) {
    if (!use_linear_motion) return;
    
    double course_rad = course_deg * M_PI / 180.0;
    double vx = speed_m_per_s * sin(course_rad);
    double vy = speed_m_per_s * cos(course_rad);
    double vx_km_s = vx / 1000.0;
    double vy_km_s = vy / 1000.0;
    
    double current_x_km = initial_x_km + vx_km_s * time_delta_seconds;
    double current_y_km = initial_y_km + vy_km_s * time_delta_seconds;
    range_km = sqrt(current_x_km*current_x_km + current_y_km*current_y_km);
    azimuth_deg = atan2(current_x_km, current_y_km) * 180.0 / M_PI;
    if (azimuth_deg < 0) azimuth_deg += 360.0;
}

// ============================================================================
// CONFIG BUILDER
// ============================================================================

SystemConfig ConfigBuilder::build(const ConfigParser& parser) {
    SystemConfig config;
    
    config.radar.range_bin_rbs = parser.get_or_default("range_bin_rbs", 30.0);
    config.radar.range_bin_uvd = parser.get_or_default("range_bin_uvd", 60.0);
    config.radar.max_azimuth_diff_for_overlap = parser.get_or_default(
        "max_azimuth_diff_for_overlap", 2.0);
    config.radar.max_range_diff_for_overlap = parser.get_or_default<uint16_t>(
        "max_range_diff_for_overlap", 10);
    config.radar.min_amplitude = parser.get_or_default<unsigned char>(
        "min_amplitude", static_cast<unsigned char>(10));
    
    config.beamwidth_deg = parser.get_or_default("beamwidth_deg", 5.0);
    
    config.processing.max_gap_azimuth = parser.get_or_default(
        "max_gap_azimuth", 8);
    config.processing.range_window = parser.get_or_default(
        "range_window", 30);
    config.processing.range_tolerance = parser.get_or_default<uint16_t>(
        "range_tolerance", 5);
    config.processing.min_hits = parser.get_or_default("min_hits", 2);
    config.processing.output_file = parser.get_or_default<std::string>(
        "output_file", "targets.txt");
    
    config.tracker.min_hits_to_confirm = parser.get_or_default(
        "min_hits_to_confirm", 3);
    config.tracker.max_coast_count = parser.get_or_default(
        "max_coast_count", 10);
    config.tracker.max_gate_distance = parser.get_or_default(
        "max_gate_distance", 300.0);
    config.tracker.max_gate_azimuth = parser.get_or_default(
        "max_gate_azimuth", 30.0);
    config.tracker.debug_mode = parser.get_or_default(
        "tracking_debug", false);
    config.tracker.process_noise = parser.get_or_default(
        "process_noise", 0.5);
    config.tracker.measurement_noise = parser.get_or_default(
        "measurement_noise", 0.1);
    
    auto rbs_targets = parser.parse_targets("RBS_TARGETS");
    for (auto& target : rbs_targets) {
        target.type = GeneratedTarget::Type::RBS;
        config.rbs_targets.push_back(std::move(target));
    }
    
    auto uvd_targets = parser.parse_targets("UVD_TARGETS");
    for (auto& target : uvd_targets) {
        target.type = GeneratedTarget::Type::UVD;
        config.uvd_targets.push_back(std::move(target));
    }
    
    return config;
}

} // namespace radar
} // namespace vrl
