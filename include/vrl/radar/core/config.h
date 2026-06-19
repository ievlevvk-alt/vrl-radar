// include/vrl/radar/core/config.h
#pragma once

#include "types.h"
#include "replies.h"
#include <string>
#include <vector>
#include <map>          // <-- ДОБАВЛЕНО
#include <optional>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <cmath>

namespace vrl {
namespace radar {

// ============================================================================
// GENERATED TARGET
// ============================================================================

struct GeneratedTarget {
    enum class Type { RBS, UVD } type;
    
    std::string name;
    double azimuth_deg{0.0};
    double range_km{0.0};
    
    uint16_t rbs_code_octal{0};
    uint32_t uvd_data_dec{0};
    
    double azimuth_speed_deg_per_rev{0.0};
    double range_speed_km_per_rev{0.0};
    
    bool spi{false};
    bool enabled{true};
    int update_every_n_revolutions{1};
    int revolution_offset{0};
    
    int altitude_meters{0};
    bool enable_altitude{false};
    bool alternate_code_altitude{false};
    mutable int current_mode{0};
    bool alternate_data_altitude{false};
    mutable int uvd_current_mode{0};
    
    bool use_linear_motion{false};
    double speed_m_per_s{0.0};
    double course_deg{0.0};
    double initial_x_km{0.0};
    double initial_y_km{0.0};
    
    uint16_t get_rbs_code() const { return rbs_code_octal; }
    uint16_t get_mode_c_code() const;
    bool get_current_rbs_reply(uint16_t& code, bool& spi_out) const;
    void toggle_rbs_mode() { 
        if (alternate_code_altitude) {
            current_mode = (current_mode + 1) % 2;
        }
    }
    
    uint32_t get_current_uvd_data() const;
    void toggle_uvd_mode() {
        if (alternate_data_altitude) {
            uvd_current_mode = (uvd_current_mode + 1) % 2;
        }
    }
    
    void update_linear_position(double time_delta_seconds);
};

// ============================================================================
// SYSTEM CONFIG
// ============================================================================

struct SimulatorConfig {
    RadarConfig radar;
    
    struct {
        double snr_db{20.0};
        double amp_variation{0.1};
        double f1f2_amp_ratio{1.0};
    } rbs;
    
    struct {
        double snr_db{20.0};
        double error_probability{0.01};
    } uvd;
    
    struct {
        bool enabled{false};
        double main_to_sls_ratio{1.0};
        double sls_attenuation_db{20.0};
        double sidelobe_probability{0.1};
    } sls;
};

struct TrackerConfig {
    int min_hits_to_confirm{3};
    int max_coast_count{5};
    double max_gate_distance{150.0};
    double max_gate_azimuth{30.0};
    double process_noise{0.1};
    double measurement_noise{1.0};
    bool enable_uvd_tracking{true};
    bool enable_rbs_tracking{true};
    bool debug_mode{false};
};

struct SystemConfig {
    RadarConfig radar;
    SimulatorConfig simulator;
    TrackerConfig tracker;
    
    struct ProcessingConfig {
        int max_gap_azimuth{8};
        int range_window{30};
        uint16_t range_tolerance{5};
        int min_hits{2};
        std::string output_file{"targets.txt"};
        std::string plots_output_file{""};  // <-- ДОБАВЛЕНО
    } processing;
    
    double beamwidth_deg{5.0};
    double revolution_time{5.0};
    std::vector<GeneratedTarget> rbs_targets;
    std::vector<GeneratedTarget> uvd_targets;
    
    bool has_targets() const {
        return !rbs_targets.empty() || !uvd_targets.empty();
    }
};

// ============================================================================
// CONFIG PARSER (СТАРЫЙ, ОСТАВЛЕН ДЛЯ СОВМЕСТИМОСТИ)
// ============================================================================

class ConfigParser {
public:
    bool load(const std::string& filename);
    
    template<typename T>
    std::optional<T> get(const std::string& key, const std::string& section = "") const {
        auto str = get_string(key, section);
        if (!str) return std::nullopt;
        return safe_stod<T>(*str);
    }
    
    template<typename T>
    T get_or_default(const std::string& key, const T& default_value, 
                     const std::string& section = "") const {
        auto val = get<T>(key, section);
        return val.value_or(default_value);
    }
    
    std::optional<std::string> get_string(const std::string& key, 
                                          const std::string& section = "") const;
    
    std::vector<GeneratedTarget> parse_targets(const std::string& section) const;
    std::vector<std::string> get_sections() const;
    
private:
    void parse_target_field(GeneratedTarget& target, const std::string& key, 
                           const std::string& value) const;
    
    template<typename T>
    static std::optional<T> safe_stod(const std::string& str) {
        if (str.empty()) return std::nullopt;
        
        std::string cleaned = trim(str);
        if (cleaned.empty()) return std::nullopt;
        
        size_t space_pos = cleaned.find_first_of(" \t");
        if (space_pos != std::string::npos) {
            cleaned = cleaned.substr(0, space_pos);
        }
        
        try {
            if constexpr (std::is_integral<T>::value) {
                if constexpr (std::is_unsigned<T>::value) {
                    if (cleaned.length() > 1 && cleaned[0] == '0') {
                        return static_cast<T>(std::stoul(cleaned, nullptr, 8));
                    }
                    return static_cast<T>(std::stoul(cleaned));
                } else {
                    return static_cast<T>(std::stoi(cleaned));
                }
            } else {
                return static_cast<T>(std::stod(cleaned));
            }
        } catch (const std::exception&) {
            return std::nullopt;
        }
    }
    
    static std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\n\r\f\v");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\n\r\f\v");
        return str.substr(first, last - first + 1);
    }
    
    std::map<std::string, std::map<std::string, std::string>> sections_;
    std::string current_section_;
    int line_number_{0};
};

// Специализация для bool
template<>
inline std::optional<bool> ConfigParser::safe_stod<bool>(const std::string& str) {
    if (str.empty()) return std::nullopt;
    std::string lower = str;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    lower = trim(lower);
    
    if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") {
        return true;
    }
    if (lower == "false" || lower == "0" || lower == "no" || lower == "off") {
        return false;
    }
    return std::nullopt;
}

// Специализация для std::string
template<>
inline std::optional<std::string> ConfigParser::safe_stod<std::string>(const std::string& str) {
    return trim(str);
}

// ============================================================================
// CONFIG BUILDER
// ============================================================================

class ConfigBuilder {
public:
    static SystemConfig build(const ConfigParser& parser);
};

} // namespace radar
} // namespace vrl
