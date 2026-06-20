// src/core/config_loader.cpp
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/utils/logger.h"
#include <fstream>
#include <set>
#include <filesystem>
#include <cmath>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ ВАЛИДАЦИИ
// ============================================================================

namespace {
    // Базовые валидаторы
    bool is_positive(double value) { return value > 0.0; }
    bool is_non_negative(double value) { return value >= 0.0; }
    bool is_in_range(double value, double min, double max) {
        return value >= min && value <= max;
    }
    bool is_angle(double value) { return value >= 0.0 && value <= 360.0; }
    bool is_percentage(double value) { return value >= 0.0 && value <= 1.0; }
    bool is_snr(double value) { return value >= -10.0 && value <= 60.0; }
    
    // Валидаторы для целых чисел
    bool is_positive_int(int value) { return value > 0; }
    bool is_non_negative_int(int value) { return value >= 0; }
    
    // Валидаторы для беззнаковых целых
    bool is_positive_uint16(uint16_t value) { return value > 0; }
    bool is_positive_uint32(uint32_t value) { return value > 0; }
    bool is_positive_uint8(uint8_t value) { return value > 0; }
    bool is_non_negative_uint16(uint16_t value) { return value >= 0; }
    bool is_non_negative_uint32(uint32_t value) { return value >= 0; }
    bool is_non_negative_uint8(uint8_t value) { return value >= 0; }
}



// ============================================================================
// ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ДЛЯ БЕЗОПАСНОГО ПОЛУЧЕНИЯ ЗНАЧЕНИЙ
// ============================================================================

// Общий шаблон для всех типов
template<typename T>
static bool safe_get_value(const json& j, const std::string& key, T& out,
                           bool (*validator)(T) = nullptr,
                           const std::string& error_msg = "") {
    if (!j.contains(key)) {
        VRL_LOG_DEBUG(modules::CONFIG, "Key not found: " + key + ", using default");
        return false;
    }
    
    try {
        if constexpr (std::is_same<T, std::string>::value) {
            if (!j[key].is_string()) {
                VRL_LOG_WARN(modules::CONFIG, "Key " + key + " is not a string");
                return false;
            }
            out = j[key].get<T>();
        } else if constexpr (std::is_same<T, bool>::value) {
            if (!j[key].is_boolean()) {
                VRL_LOG_WARN(modules::CONFIG, "Key " + key + " is not a boolean");
                return false;
            }
            out = j[key].get<T>();
        } else if constexpr (std::is_arithmetic<T>::value) {
            if (!j[key].is_number()) {
                VRL_LOG_WARN(modules::CONFIG, "Key " + key + " is not a number");
                return false;
            }
            out = j[key].get<T>();
        } else {
            out = j[key].get<T>();
        }
        
        // Валидация
        if (validator && !validator(out)) {
            VRL_LOG_WARN(modules::CONFIG, "Validation failed for " + key + 
                         (error_msg.empty() ? "" : ": " + error_msg));
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_WARN(modules::CONFIG, "Failed to get value for " + key + 
                     ": " + std::string(e.what()));
        return false;
    }
}



// ============================================================================
// КОНФИГУРАЦИЯ
// ============================================================================

json ConfigLoader::load_with_includes(const std::string& filename, 
                                      const std::filesystem::path& base_path) {
    VRL_LOG_DEBUG(modules::CONFIG, "Loading config with includes: " + filename);
    
    for (const auto& loaded : loaded_files_) {
        if (loaded == filename) {
            VRL_LOG_WARN(modules::CONFIG, "Circular include detected: " + filename);
            return json::object();
        }
    }
    loaded_files_.push_back(filename);
    
    std::filesystem::path file_path = filename;
    if (!file_path.is_absolute()) {
        file_path = base_path / file_path;
    }
    
    try {
        file_path = std::filesystem::weakly_canonical(file_path);
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to normalize path: " + std::string(e.what()));
        return json::object();
    }
    
    VRL_LOG_DEBUG(modules::CONFIG, "Full path: " + file_path.string());
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        VRL_LOG_ERROR(modules::CONFIG, "Cannot open file: " + file_path.string());
        return json::object();
    }
    
    json result;
    try {
        file >> result;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse JSON: " + std::string(e.what()));
        return json::object();
    }
    
    if (!result.is_object()) {
        VRL_LOG_ERROR(modules::CONFIG, "JSON root is not an object");
        return json::object();
    }
    
    json merged_result = json::object();
    
    if (result.contains("_includes") && result["_includes"].is_array()) {
        for (const auto& include_file : result["_includes"]) {
            if (!include_file.is_string()) continue;
            
            std::string include_path = include_file.get<std::string>();
            std::filesystem::path include_full_path = file_path.parent_path() / include_path;
            try {
                include_full_path = std::filesystem::weakly_canonical(include_full_path);
            } catch (const std::exception& e) {
                VRL_LOG_ERROR(modules::CONFIG, "Failed to normalize include path: " + std::string(e.what()));
                continue;
            }
            
            VRL_LOG_DEBUG(modules::CONFIG, "Including: " + include_full_path.string());
            
            json included = load_with_includes(include_full_path.string(), 
                                               file_path.parent_path());
            
            if (!included.is_object()) {
                VRL_LOG_WARN(modules::CONFIG, "Included file is not an object: " + include_full_path.string());
                continue;
            }
            
            if (included.empty()) {
                VRL_LOG_WARN(modules::CONFIG, "Included file is empty: " + include_full_path.string());
                continue;
            }
            
            merged_result = merge_json(merged_result, included);
        }
    }
    
    if (merged_result.empty() && !result.empty()) {
        bool has_data = false;
        for (auto& [key, _] : result.items()) {
            if (key != "_includes" && key != "_comment" && key != "$schema") {
                has_data = true;
                break;
            }
        }
        if (has_data) {
            merged_result = result;
            if (merged_result.contains("_includes")) merged_result.erase("_includes");
            if (merged_result.contains("_comment")) merged_result.erase("_comment");
            if (merged_result.contains("$schema")) merged_result.erase("$schema");
        }
    }
    
    if (merged_result.contains("_includes")) merged_result.erase("_includes");
    if (merged_result.contains("_comment")) merged_result.erase("_comment");
    if (merged_result.contains("$schema")) merged_result.erase("$schema");
    
    VRL_LOG_DEBUG(modules::CONFIG, "Final merged config has " + 
                  std::to_string(merged_result.size()) + " top-level keys");
    
    return merged_result;
}

json ConfigLoader::merge_json(const json& base, const json& overlay) {
    if (!overlay.is_object()) {
        return base;
    }
    
    if (overlay.empty()) {
        return base;
    }
    
    if (!base.is_object()) {
        return overlay;
    }
    
    if (base.empty()) {
        return overlay;
    }
    
    json result = base;
    
    for (auto& [key, value] : overlay.items()) {
        if (key == "_includes" || key == "_comment" || key == "$schema") {
            continue;
        }
        
        if (result.contains(key)) {
            if (result[key].is_object() && value.is_object()) {
                result[key] = merge_json(result[key], value);
            } else {
                result[key] = value;
            }
        } else {
            result[key] = value;
        }
    }
    
    return result;
}

bool ConfigLoader::load(const std::string& filename, SystemConfig& config) {
    VRL_LOG_INFO(modules::CONFIG, "Loading config from: " + filename);
    
    loaded_files_.clear();
    
    std::filesystem::path file_path = filename;
    std::filesystem::path base_path = file_path.parent_path();
    if (base_path.empty()) {
        base_path = ".";
    }
    
    json merged = load_with_includes(filename, base_path);
    
    if (merged.empty() || !merged.is_object()) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to load config (empty or invalid result)");
        return false;
    }
    
    VRL_LOG_DEBUG(modules::CONFIG, "Merged config has keys: " + 
                  std::to_string(merged.size()));
    for (auto& [key, _] : merged.items()) {
        VRL_LOG_DEBUG(modules::CONFIG, "  Key: " + key);
    }
    
    if (!parse_config(merged, config)) {
        return false;
    }
    
    std::string error;
    if (!validate(config, error)) {
        VRL_LOG_ERROR(modules::CONFIG, "Config validation failed: " + error);
        return false;
    }
    
    VRL_LOG_INFO(modules::CONFIG, "Config loaded successfully from " + 
                 std::to_string(loaded_files_.size()) + " files");
    VRL_LOG_DEBUG(modules::CONFIG, "RBS targets: " + std::to_string(config.rbs_targets.size()));
    VRL_LOG_DEBUG(modules::CONFIG, "UVD targets: " + std::to_string(config.uvd_targets.size()));
    
    return true;
}

bool ConfigLoader::load_from_string(const std::string& content, SystemConfig& config) {
    try {
        json j = json::parse(content);
        return parse_config(j, config);
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse config: " + std::string(e.what()));
        return false;
    }
}

bool ConfigLoader::save(const SystemConfig& config, const std::string& filename) {
    VRL_LOG_INFO(modules::CONFIG, "Saving config to: " + filename);
    
    try {
        json j = to_json(config);
        std::ofstream file(filename);
        if (!file.is_open()) {
            VRL_LOG_ERROR(modules::CONFIG, "Cannot open file for writing: " + filename);
            return false;
        }
        
        file << j.dump(4);
        file.close();
        
        VRL_LOG_INFO(modules::CONFIG, "Config saved successfully");
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to save config: " + std::string(e.what()));
        return false;
    }
}

// ============================================================================
// PARSE TARGET
// ============================================================================

bool ConfigLoader::parse_target(const json& j, GeneratedTarget& target, bool is_rbs) {
    try {
        // Проверяем обязательные поля
        if (!j.contains("name") || !j["name"].is_string()) {
            VRL_LOG_ERROR(modules::CONFIG, "Target missing 'name' field");
            return false;
        }
        target.name = j["name"].get<std::string>();
        
        // Поля с валидацией
        safe_get_value(j, "azimuth_deg", target.azimuth_deg, is_angle, "must be 0-360");
        safe_get_value(j, "range_km", target.range_km, is_positive, "must be > 0");
        
        if (is_rbs) {
            if (!safe_get_value(j, "rbs_code_octal", target.rbs_code_octal, 
                               is_positive_uint16, "must be > 0")) {
                VRL_LOG_WARN(modules::CONFIG, "RBS target " + target.name + 
                             " missing or invalid rbs_code_octal");
            }
        } else {
            if (!safe_get_value(j, "uvd_data_dec", target.uvd_data_dec,
                               is_positive_uint32, "must be > 0")) {
                VRL_LOG_WARN(modules::CONFIG, "UVD target " + target.name + 
                             " missing or invalid uvd_data_dec");
            }
        }
        
        safe_get_value(j, "azimuth_speed_deg_per_rev", target.azimuth_speed_deg_per_rev);
        safe_get_value(j, "range_speed_km_per_rev", target.range_speed_km_per_rev);
        safe_get_value(j, "spi", target.spi);
        safe_get_value(j, "enabled", target.enabled);
        safe_get_value(j, "update_every_n_revolutions", target.update_every_n_revolutions,
                      is_positive_int, "must be >= 1");
        safe_get_value(j, "revolution_offset", target.revolution_offset,
                      is_non_negative_int, "must be >= 0");
        
        safe_get_value(j, "altitude_meters", target.altitude_meters);
        safe_get_value(j, "enable_altitude", target.enable_altitude);
        safe_get_value(j, "alternate_code_altitude", target.alternate_code_altitude);
        safe_get_value(j, "alternate_data_altitude", target.alternate_data_altitude);
        safe_get_value(j, "use_linear_motion", target.use_linear_motion);
        
        safe_get_value(j, "speed_m_per_s", target.speed_m_per_s, is_non_negative, "must be >= 0");
        safe_get_value(j, "course_deg", target.course_deg, is_angle, "must be 0-360");
        safe_get_value(j, "initial_x_km", target.initial_x_km);
        safe_get_value(j, "initial_y_km", target.initial_y_km);
        
        target.type = is_rbs ? GeneratedTarget::Type::RBS : GeneratedTarget::Type::UVD;
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse target: " + std::string(e.what()));
        return false;
    }
}


// ============================================================================
// PARSE CONFIG
// ============================================================================

bool ConfigLoader::parse_config(const json& j, SystemConfig& config) {
    try {
        if (!j.is_object()) {
            VRL_LOG_ERROR(modules::CONFIG, "Config root is not an object");
            return false;
        }
        
        if (j.empty()) {
            VRL_LOG_ERROR(modules::CONFIG, "Config is empty");
            return false;
        }
        
        // ===== RADAR =====
        if (j.contains("radar") && j["radar"].is_object()) {
            const auto& r = j["radar"];
            
            safe_get_value(r, "range_bin_rbs", config.radar.range_bin_rbs,
                          is_positive, "must be > 0");
            safe_get_value(r, "range_bin_uvd", config.radar.range_bin_uvd,
                          is_positive, "must be > 0");
            safe_get_value(r, "max_azimuth_diff_for_overlap", 
                          config.radar.max_azimuth_diff_for_overlap,
                          is_non_negative, "must be >= 0");
            safe_get_value(r, "max_range_diff_for_overlap", 
                          config.radar.max_range_diff_for_overlap,
                          is_positive_uint16, "must be > 0");
            safe_get_value(r, "min_amplitude", config.radar.min_amplitude,
                          is_positive_uint8, "must be > 0");
        }
        
        // ===== RBS SIMULATOR =====
        if (j.contains("rbs") && j["rbs"].is_object()) {
            const auto& r = j["rbs"];
            safe_get_value(r, "snr_db", config.simulator.rbs.snr_db, is_snr, "must be -10 to 60");
            safe_get_value(r, "amp_variation", config.simulator.rbs.amp_variation,
                          is_non_negative, "must be >= 0");
            safe_get_value(r, "f1f2_amp_ratio", config.simulator.rbs.f1f2_amp_ratio,
                          is_positive, "must be > 0");
        }
        
        // ===== UVD SIMULATOR =====
        if (j.contains("uvd") && j["uvd"].is_object()) {
            const auto& u = j["uvd"];
            safe_get_value(u, "snr_db", config.simulator.uvd.snr_db, is_snr, "must be -10 to 60");
            safe_get_value(u, "error_probability", config.simulator.uvd.error_probability,
                          is_percentage, "must be 0-1");
        }
        
        // ===== SLS =====
        if (j.contains("sls") && j["sls"].is_object()) {
            const auto& s = j["sls"];
            safe_get_value(s, "enabled", config.simulator.sls.enabled);
            safe_get_value(s, "main_to_sls_ratio", config.simulator.sls.main_to_sls_ratio,
                          is_positive, "must be > 0");
            safe_get_value(s, "sls_attenuation_db", config.simulator.sls.sls_attenuation_db,
                          is_non_negative, "must be >= 0");
            safe_get_value(s, "sidelobe_probability", config.simulator.sls.sidelobe_probability,
                          is_percentage, "must be 0-1");
        }
        
        // ===== TRACKER =====
        if (j.contains("tracker") && j["tracker"].is_object()) {
            const auto& t = j["tracker"];
            safe_get_value(t, "min_hits_to_confirm", config.tracker.min_hits_to_confirm,
                          is_positive_int, "must be >= 1");
            safe_get_value(t, "max_coast_count", config.tracker.max_coast_count,
                          is_positive_int, "must be >= 1");
            safe_get_value(t, "max_gate_distance", config.tracker.max_gate_distance,
                          is_positive, "must be > 0");
            safe_get_value(t, "max_gate_azimuth", config.tracker.max_gate_azimuth,
                          is_positive, "must be > 0");
            safe_get_value(t, "process_noise", config.tracker.process_noise,
                          is_non_negative, "must be >= 0");
            safe_get_value(t, "measurement_noise", config.tracker.measurement_noise,
                          is_non_negative, "must be >= 0");
            safe_get_value(t, "enable_uvd_tracking", config.tracker.enable_uvd_tracking);
            safe_get_value(t, "enable_rbs_tracking", config.tracker.enable_rbs_tracking);
            safe_get_value(t, "debug_mode", config.tracker.debug_mode);
        }
        
        // ===== PROCESSING =====
        if (j.contains("processing") && j["processing"].is_object()) {
            const auto& p = j["processing"];
            safe_get_value(p, "max_gap_azimuth", config.processing.max_gap_azimuth,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "range_window", config.processing.range_window,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "range_tolerance", config.processing.range_tolerance,
                          is_positive_uint16, "must be >= 1");
            safe_get_value(p, "min_hits", config.processing.min_hits,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "output_file", config.processing.output_file);
            safe_get_value(p, "plots_output_file", config.processing.plots_output_file);
            safe_get_value(p, "min_cluster_hits", config.processing.min_cluster_hits,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "range_threshold_bins", config.processing.range_threshold_bins,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "azimuth_threshold_bins", config.processing.azimuth_threshold_bins,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "completion_gap_bins", config.processing.completion_gap_bins,
                          is_positive_int, "must be >= 1");
            safe_get_value(p, "min_confidence", config.processing.min_confidence,
                          is_percentage, "must be 0-1");
            safe_get_value(p, "garbled_confidence_threshold", config.processing.garbled_confidence_threshold,
                          is_percentage, "must be 0-1");
            safe_get_value(p, "min_uvd_confidence", config.processing.min_uvd_confidence,
                          is_percentage, "must be 0-1");
            safe_get_value(p, "uvd_garbled_threshold", config.processing.uvd_garbled_threshold,
                          is_percentage, "must be 0-1");
        }
        
        // ===== CONFIDENCE =====
        if (j.contains("confidence") && j["confidence"].is_object()) {
            const auto& c = j["confidence"];
            safe_get_value(c, "initial_track_confidence", config.confidence.initial_track_confidence,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "coast_confidence_decay", config.confidence.coast_confidence_decay,
                          is_non_negative, "must be >= 0");
            safe_get_value(c, "min_track_confidence", config.confidence.min_track_confidence,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "max_track_confidence", config.confidence.max_track_confidence,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_rbs_weight_framing", config.confidence.confidence_rbs_weight_framing,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_rbs_weight_snr", config.confidence.confidence_rbs_weight_snr,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_rbs_weight_stability", config.confidence.confidence_rbs_weight_stability,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_rbs_weight_errors", config.confidence.confidence_rbs_weight_errors,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_uvd_weight_snr", config.confidence.confidence_uvd_weight_snr,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_uvd_weight_errors", config.confidence.confidence_uvd_weight_errors,
                          is_percentage, "must be 0-1");
            safe_get_value(c, "confidence_uvd_weight_stability", config.confidence.confidence_uvd_weight_stability,
                          is_percentage, "must be 0-1");
        }
        
        // ===== SIMULATOR CONSTANTS =====
        if (j.contains("simulator_constants") && j["simulator_constants"].is_object()) {
            const auto& s = j["simulator_constants"];
            safe_get_value(s, "base_signal_power", config.simulator_constants.base_signal_power,
                          is_positive, "must be > 0");
            safe_get_value(s, "amp_variation_min", config.simulator_constants.amp_variation_min,
                          is_positive, "must be > 0");
            safe_get_value(s, "amp_variation_max", config.simulator_constants.amp_variation_max,
                          is_positive, "must be > 0");
            safe_get_value(s, "uvd_error_threshold", config.simulator_constants.uvd_error_threshold,
                          is_positive, "must be > 0");
            safe_get_value(s, "max_snr_db", config.simulator_constants.max_snr_db,
                          is_positive, "must be > 0");
            safe_get_value(s, "min_speed_ms", config.simulator_constants.min_speed_ms,
                          is_non_negative, "must be >= 0");
            safe_get_value(s, "min_time_delta", config.simulator_constants.min_time_delta,
                          is_positive, "must be > 0");
            safe_get_value(s, "max_mode_c_code", config.simulator_constants.max_mode_c_code,
                          is_positive_int, "must be > 0");
            safe_get_value(s, "max_mode_c_attempts", config.simulator_constants.max_mode_c_attempts,
                          is_positive_int, "must be > 0");
            safe_get_value(s, "display_beamwidth_deg", config.simulator_constants.display_beamwidth_deg,
                          is_positive, "must be > 0");
            safe_get_value(s, "min_amplitude_ratio_for_separation", 
                          config.simulator_constants.min_amplitude_ratio_for_separation,
                          is_percentage, "must be 0-1");
        }
        
        // ===== AZIMUTH =====
        if (j.contains("azimuth") && j["azimuth"].is_object()) {
            const auto& a = j["azimuth"];
            int bins;
            if (safe_get_value(a, "azimuth_bins", bins, is_positive_int, "must be > 0")) {
                config.azimuth.azimuth_bins = bins;
                config.azimuth.azimuth_half = config.azimuth.azimuth_bins / 2;
                config.azimuth.azimuth_per_bin_deg = 360.0 / config.azimuth.azimuth_bins;
                config.azimuth.azimuth_per_bin_rad = M_PI / (config.azimuth.azimuth_bins / 2);
            }
        }
        
        // ===== ОБЩИЕ ПАРАМЕТРЫ =====
        safe_get_value(j, "beamwidth_deg", config.beamwidth_deg, is_positive, "must be > 0");
        safe_get_value(j, "revolution_time", config.revolution_time, is_positive, "must be > 0");
        
        // ===== LOGGING =====
        if (j.contains("logging") && j["logging"].is_object()) {
            const auto& l = j["logging"];
            safe_get_value(l, "console_enabled", config.logging.console_enabled);
            safe_get_value(l, "file_enabled", config.logging.file_enabled);
            safe_get_value(l, "log_file", config.logging.log_file);
            safe_get_value(l, "timestamp_format", config.logging.timestamp_format);
            
            if (l.contains("modules") && l["modules"].is_object()) {
                for (auto& [module, level] : l["modules"].items()) {
                    if (level.is_string()) {
                        config.logging.set_module_level(module, level.get<std::string>());
                    }
                }
            }
        }
        
        // ===== RBS TARGETS =====
        if (j.contains("rbs_targets") && j["rbs_targets"].is_array()) {
            for (const auto& target_json : j["rbs_targets"]) {
                GeneratedTarget target;
                if (parse_target(target_json, target, true)) {
                    config.rbs_targets.push_back(target);
                } else {
                    VRL_LOG_WARN(modules::CONFIG, "Skipping invalid RBS target");
                }
            }
        }
        
        // ===== UVD TARGETS =====
        if (j.contains("uvd_targets") && j["uvd_targets"].is_array()) {
            for (const auto& target_json : j["uvd_targets"]) {
                GeneratedTarget target;
                if (parse_target(target_json, target, false)) {
                    config.uvd_targets.push_back(target);
                } else {
                    VRL_LOG_WARN(modules::CONFIG, "Skipping invalid UVD target");
                }
            }
        }

        // ===== CLUSTERER =====
        if (j.contains("clusterer") && j["clusterer"].is_object()) {
            const auto& c = j["clusterer"];
            
            // Тип кластеризатора
            std::string type_str;
            if (safe_get_value(c, "type", type_str)) {
                if (type_str == "dbscan") {
                    config.clusterer.type = ClustererConfig::Type::DBSCAN;
                } else if (type_str == "legacy") {
                    config.clusterer.type = ClustererConfig::Type::LEGACY;
                } else {
                    VRL_LOG_WARN(modules::CONFIG, "Unknown clusterer type: " + type_str + 
                                 ", using dbscan");
                    config.clusterer.type = ClustererConfig::Type::DBSCAN;
                }
            }
            
            // Параметры DBSCAN
            safe_get_value(c, "max_range_gap", config.clusterer.max_range_gap,
                          is_positive_int, "must be >= 1");
            // min_points УДАЛЕН
            safe_get_value(c, "azimuth_gap_coefficient", config.clusterer.azimuth_gap_coefficient,
                          is_positive, "must be > 0");
            
            // Параметры Legacy
            safe_get_value(c, "max_gap_azimuth", config.clusterer.max_gap_azimuth,
                          is_positive_int, "must be >= 1");
            safe_get_value(c, "range_window", config.clusterer.range_window,
                          is_positive_int, "must be >= 1");
            
            // Общие параметры
            safe_get_value(c, "max_revolutions_no_update", config.clusterer.max_revolutions_no_update,
                          is_positive_int, "must be >= 1");
            safe_get_value(c, "max_active_clusters", config.clusterer.max_active_clusters,
                          is_positive_int, "must be >= 1");
        }

        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse config: " + std::string(e.what()));
        return false;
    }
}



// ============================================================================
// VALIDATE - РАСШИРЕННАЯ ВАЛИДАЦИЯ
// ============================================================================

bool ConfigLoader::validate(const SystemConfig& config, std::string& error) const {
    // Проверка наличия целей
    if (!config.has_targets()) {
        error = "No targets defined in configuration";
        return false;
    }
    
    // Проверка параметров радара
    if (config.radar.range_bin_rbs <= 0) {
        error = "range_bin_rbs must be > 0";
        return false;
    }
    
    if (config.radar.range_bin_uvd <= 0) {
        error = "range_bin_uvd must be > 0";
        return false;
    }
    
    if (config.beamwidth_deg <= 0 || config.beamwidth_deg > 360) {
        error = "beamwidth_deg must be between 0 and 360";
        return false;
    }
    
    if (config.revolution_time <= 0) {
        error = "revolution_time must be > 0";
        return false;
    }
    
    // Проверка дубликатов RBS кодов
    std::set<uint16_t> rbs_codes;
    for (const auto& target : config.rbs_targets) {
        if (!target.enabled) continue;
        if (rbs_codes.count(target.rbs_code_octal)) {
            error = "Duplicate RBS code: 0" + std::to_string(target.rbs_code_octal) + 
                    " (target: " + target.name + ")";
            return false;
        }
        rbs_codes.insert(target.rbs_code_octal);
    }
    
    // Проверка дубликатов UVD данных
    std::set<uint32_t> uvd_data;
    for (const auto& target : config.uvd_targets) {
        if (!target.enabled) continue;
        if (uvd_data.count(target.uvd_data_dec)) {
            error = "Duplicate UVD data: " + std::to_string(target.uvd_data_dec) + 
                    " (target: " + target.name + ")";
            return false;
        }
        uvd_data.insert(target.uvd_data_dec);
    }
    
    // Проверка параметров трекера
    if (config.tracker.min_hits_to_confirm < 1) {
        error = "min_hits_to_confirm must be >= 1";
        return false;
    }
    
    if (config.tracker.max_coast_count < 1) {
        error = "max_coast_count must be >= 1";
        return false;
    }
    
    if (config.tracker.max_gate_distance <= 0) {
        error = "max_gate_distance must be > 0";
        return false;
    }
    
    if (config.tracker.max_gate_azimuth <= 0) {
        error = "max_gate_azimuth must be > 0";
        return false;
    }
    
    if (config.tracker.process_noise < 0) {
        error = "process_noise must be >= 0";
        return false;
    }
    
    if (config.tracker.measurement_noise < 0) {
        error = "measurement_noise must be >= 0";
        return false;
    }
    
    // Проверка параметров обработки
    if (config.processing.max_gap_azimuth < 1) {
        error = "max_gap_azimuth must be >= 1";
        return false;
    }
    
    if (config.processing.range_window < 1) {
        error = "range_window must be >= 1";
        return false;
    }
    
    if (config.processing.min_hits < 1) {
        error = "min_hits must be >= 1";
        return false;
    }
    
    // Проверка весов уверенности
    double sum_rbs = config.confidence.confidence_rbs_weight_framing +
                     config.confidence.confidence_rbs_weight_snr +
                     config.confidence.confidence_rbs_weight_stability +
                     config.confidence.confidence_rbs_weight_errors;
    
    if (std::abs(sum_rbs - 1.0) > 0.01) {
        error = "RBS confidence weights must sum to 1.0 (current: " + 
                std::to_string(sum_rbs) + ")";
        return false;
    }
    
    double sum_uvd = config.confidence.confidence_uvd_weight_snr +
                     config.confidence.confidence_uvd_weight_errors +
                     config.confidence.confidence_uvd_weight_stability;
    
    if (std::abs(sum_uvd - 1.0) > 0.01) {
        error = "UVD confidence weights must sum to 1.0 (current: " + 
                std::to_string(sum_uvd) + ")";
        return false;
    }
    
    // Проверка порогов уверенности
    if (config.processing.min_confidence < 0 || config.processing.min_confidence > 1) {
        error = "min_confidence must be between 0 and 1";
        return false;
    }
    
    if (config.processing.garbled_confidence_threshold < 0 || 
        config.processing.garbled_confidence_threshold > 1) {
        error = "garbled_confidence_threshold must be between 0 and 1";
        return false;
    }
    
    // Проверка параметров симуляции
    if (config.simulator.rbs.snr_db < -10 || config.simulator.rbs.snr_db > 60) {
        error = "RBS SNR must be between -10 and 60 dB";
        return false;
    }
    
    if (config.simulator.uvd.snr_db < -10 || config.simulator.uvd.snr_db > 60) {
        error = "UVD SNR must be between -10 and 60 dB";
        return false;
    }
    
    if (config.simulator.uvd.error_probability < 0 || 
        config.simulator.uvd.error_probability > 1) {
        error = "error_probability must be between 0 and 1";
        return false;
    }
    
    // Проверка параметров кластеризации
    if (config.clusterer.max_active_clusters < 1) {
        error = "max_active_clusters must be >= 1";
        return false;
    }
    
    if (config.clusterer.max_revolutions_no_update < 1) {
        error = "max_revolutions_no_update must be >= 1";
        return false;
    }
    
    if (config.clusterer.max_range_gap < 1) {
        error = "max_range_gap must be >= 1";
        return false;
    }
    
    if (config.clusterer.azimuth_gap_coefficient <= 0) {
        error = "azimuth_gap_coefficient must be > 0";
        return false;
    }
    
    return true;
}

// ============================================================================
// TO JSON
// ============================================================================

json ConfigLoader::to_json(const SystemConfig& config) {
    json j;
    
    j["radar"] = {
        {"range_bin_rbs", config.radar.range_bin_rbs},
        {"range_bin_uvd", config.radar.range_bin_uvd},
        {"max_azimuth_diff_for_overlap", config.radar.max_azimuth_diff_for_overlap},
        {"max_range_diff_for_overlap", config.radar.max_range_diff_for_overlap},
        {"min_amplitude", config.radar.min_amplitude}
    };
    
    j["rbs"] = {
        {"snr_db", config.simulator.rbs.snr_db},
        {"amp_variation", config.simulator.rbs.amp_variation},
        {"f1f2_amp_ratio", config.simulator.rbs.f1f2_amp_ratio}
    };
    
    j["uvd"] = {
        {"snr_db", config.simulator.uvd.snr_db},
        {"error_probability", config.simulator.uvd.error_probability}
    };
    
    j["sls"] = {
        {"enabled", config.simulator.sls.enabled},
        {"main_to_sls_ratio", config.simulator.sls.main_to_sls_ratio},
        {"sls_attenuation_db", config.simulator.sls.sls_attenuation_db},
        {"sidelobe_probability", config.simulator.sls.sidelobe_probability}
    };
    
    j["tracker"] = {
        {"min_hits_to_confirm", config.tracker.min_hits_to_confirm},
        {"max_coast_count", config.tracker.max_coast_count},
        {"max_gate_distance", config.tracker.max_gate_distance},
        {"max_gate_azimuth", config.tracker.max_gate_azimuth},
        {"process_noise", config.tracker.process_noise},
        {"measurement_noise", config.tracker.measurement_noise},
        {"enable_uvd_tracking", config.tracker.enable_uvd_tracking},
        {"enable_rbs_tracking", config.tracker.enable_rbs_tracking},
        {"debug_mode", config.tracker.debug_mode}
    };
    
    j["processing"] = {
        {"max_gap_azimuth", config.processing.max_gap_azimuth},
        {"range_window", config.processing.range_window},
        {"range_tolerance", config.processing.range_tolerance},
        {"min_hits", config.processing.min_hits},
        {"output_file", config.processing.output_file},
        {"plots_output_file", config.processing.plots_output_file},
        {"min_cluster_hits", config.processing.min_cluster_hits},
        {"range_threshold_bins", config.processing.range_threshold_bins},
        {"azimuth_threshold_bins", config.processing.azimuth_threshold_bins},
        {"completion_gap_bins", config.processing.completion_gap_bins},
        {"min_confidence", config.processing.min_confidence},
        {"garbled_confidence_threshold", config.processing.garbled_confidence_threshold},
        {"min_uvd_confidence", config.processing.min_uvd_confidence},
        {"uvd_garbled_threshold", config.processing.uvd_garbled_threshold}
    };
    
    j["confidence"] = {
        {"initial_track_confidence", config.confidence.initial_track_confidence},
        {"coast_confidence_decay", config.confidence.coast_confidence_decay},
        {"min_track_confidence", config.confidence.min_track_confidence},
        {"max_track_confidence", config.confidence.max_track_confidence},
        {"confidence_rbs_weight_framing", config.confidence.confidence_rbs_weight_framing},
        {"confidence_rbs_weight_snr", config.confidence.confidence_rbs_weight_snr},
        {"confidence_rbs_weight_stability", config.confidence.confidence_rbs_weight_stability},
        {"confidence_rbs_weight_errors", config.confidence.confidence_rbs_weight_errors},
        {"confidence_uvd_weight_snr", config.confidence.confidence_uvd_weight_snr},
        {"confidence_uvd_weight_errors", config.confidence.confidence_uvd_weight_errors},
        {"confidence_uvd_weight_stability", config.confidence.confidence_uvd_weight_stability}
    };
    
    j["simulator_constants"] = {
        {"base_signal_power", config.simulator_constants.base_signal_power},
        {"amp_variation_min", config.simulator_constants.amp_variation_min},
        {"amp_variation_max", config.simulator_constants.amp_variation_max},
        {"uvd_error_threshold", config.simulator_constants.uvd_error_threshold},
        {"max_snr_db", config.simulator_constants.max_snr_db},
        {"min_speed_ms", config.simulator_constants.min_speed_ms},
        {"min_time_delta", config.simulator_constants.min_time_delta},
        {"max_mode_c_code", config.simulator_constants.max_mode_c_code},
        {"max_mode_c_attempts", config.simulator_constants.max_mode_c_attempts},
        {"display_beamwidth_deg", config.simulator_constants.display_beamwidth_deg},
        {"min_amplitude_ratio_for_separation", config.simulator_constants.min_amplitude_ratio_for_separation}
    };
    
    j["azimuth"] = {
        {"azimuth_bins", config.azimuth.azimuth_bins}
    };
    
    j["beamwidth_deg"] = config.beamwidth_deg;
    j["revolution_time"] = config.revolution_time;
    
    // Logging
    json modules_json = json::object();
    for (const auto& [module, level] : config.logging.module_levels) {
        modules_json[module] = level;
    }
    
    j["logging"] = {
        {"console_enabled", config.logging.console_enabled},
        {"file_enabled", config.logging.file_enabled},
        {"log_file", config.logging.log_file},
        {"timestamp_format", config.logging.timestamp_format},
        {"modules", modules_json}
    };
    
    // RBS Targets
    j["rbs_targets"] = json::array();
    for (const auto& target : config.rbs_targets) {
        json t;
        t["name"] = target.name;
        t["azimuth_deg"] = target.azimuth_deg;
        t["range_km"] = target.range_km;
        t["rbs_code_octal"] = target.rbs_code_octal;
        t["spi"] = target.spi;
        t["enabled"] = target.enabled;
        t["update_every_n_revolutions"] = target.update_every_n_revolutions;
        t["revolution_offset"] = target.revolution_offset;
        t["altitude_meters"] = target.altitude_meters;
        t["enable_altitude"] = target.enable_altitude;
        t["alternate_code_altitude"] = target.alternate_code_altitude;
        t["alternate_data_altitude"] = target.alternate_data_altitude;
        t["use_linear_motion"] = target.use_linear_motion;
        t["speed_m_per_s"] = target.speed_m_per_s;
        t["course_deg"] = target.course_deg;
        t["initial_x_km"] = target.initial_x_km;
        t["initial_y_km"] = target.initial_y_km;
        j["rbs_targets"].push_back(t);
    }
    
    // UVD Targets
    j["uvd_targets"] = json::array();
    for (const auto& target : config.uvd_targets) {
        json t;
        t["name"] = target.name;
        t["azimuth_deg"] = target.azimuth_deg;
        t["range_km"] = target.range_km;
        t["uvd_data_dec"] = target.uvd_data_dec;
        t["enabled"] = target.enabled;
        t["update_every_n_revolutions"] = target.update_every_n_revolutions;
        t["revolution_offset"] = target.revolution_offset;
        t["altitude_meters"] = target.altitude_meters;
        t["enable_altitude"] = target.enable_altitude;
        t["alternate_code_altitude"] = target.alternate_code_altitude;
        t["alternate_data_altitude"] = target.alternate_data_altitude;
        t["use_linear_motion"] = target.use_linear_motion;
        t["speed_m_per_s"] = target.speed_m_per_s;
        t["course_deg"] = target.course_deg;
        t["initial_x_km"] = target.initial_x_km;
        t["initial_y_km"] = target.initial_y_km;
        j["uvd_targets"].push_back(t);
    }

    // ===== CLUSTERER =====
    std::string type_str = (config.clusterer.type == ClustererConfig::Type::DBSCAN) ? 
                           "dbscan" : "legacy";
    
    j["clusterer"] = {
        {"type", type_str},
        {"max_range_gap", config.clusterer.max_range_gap},
        // min_points УДАЛЕН
        {"azimuth_gap_coefficient", config.clusterer.azimuth_gap_coefficient},
        {"max_gap_azimuth", config.clusterer.max_gap_azimuth},
        {"range_window", config.clusterer.range_window},
        {"max_revolutions_no_update", config.clusterer.max_revolutions_no_update},
        {"max_active_clusters", config.clusterer.max_active_clusters}
    };

    return j;
}

} // namespace radar
} // namespace vrl
