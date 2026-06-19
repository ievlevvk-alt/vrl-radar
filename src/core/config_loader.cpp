// src/core/config_loader.cpp
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/utils/logger.h"
#include <fstream>
#include <set>
#include <filesystem>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

json ConfigLoader::load_with_includes(const std::string& filename, 
                                      const std::filesystem::path& base_path) {
    VRL_LOG_DEBUG(modules::CONFIG, "Loading config with includes: " + filename);
    
    // Проверка на циклические включения
    for (const auto& loaded : loaded_files_) {
        if (loaded == filename) {
            VRL_LOG_WARN(modules::CONFIG, "Circular include detected: " + filename);
            return json::object();
        }
    }
    loaded_files_.push_back(filename);
    
    // Определяем полный путь
    std::filesystem::path file_path = filename;
    if (!file_path.is_absolute()) {
        file_path = base_path / file_path;
    }
    
    // Нормализуем путь
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
    
    // Проверяем, что результат - объект
    if (!result.is_object()) {
        VRL_LOG_ERROR(modules::CONFIG, "JSON root is not an object");
        return json::object();
    }
    
    // Начинаем с пустого объекта для сбора данных
    json merged_result = json::object();
    
    // Обработка include директив
    if (result.contains("_includes") && result["_includes"].is_array()) {
        for (const auto& include_file : result["_includes"]) {
            if (!include_file.is_string()) continue;
            
            std::string include_path = include_file.get<std::string>();
            
            // Путь относительно текущего файла
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
            
            // Проверяем, что включенный файл - объект и не пустой
            if (!included.is_object()) {
                VRL_LOG_WARN(modules::CONFIG, "Included file is not an object: " + include_full_path.string());
                continue;
            }
            
            if (included.empty()) {
                VRL_LOG_WARN(modules::CONFIG, "Included file is empty: " + include_full_path.string());
                continue;
            }
            
            // Рекурсивное слияние
            merged_result = merge_json(merged_result, included);
        }
    }
    
    // Если нет include, но есть данные в самом файле - используем их
    if (merged_result.empty() && !result.empty()) {
        // Проверяем, есть ли в файле полезные данные (не только мета-поля)
        bool has_data = false;
        for (auto& [key, _] : result.items()) {
            if (key != "_includes" && key != "_comment" && key != "$schema") {
                has_data = true;
                break;
            }
        }
        if (has_data) {
            merged_result = result;
            // Удаляем мета-поля
            if (merged_result.contains("_includes")) merged_result.erase("_includes");
            if (merged_result.contains("_comment")) merged_result.erase("_comment");
            if (merged_result.contains("$schema")) merged_result.erase("$schema");
        }
    }
    
    // Удаляем мета-поля из результата (на всякий случай)
    if (merged_result.contains("_includes")) merged_result.erase("_includes");
    if (merged_result.contains("_comment")) merged_result.erase("_comment");
    if (merged_result.contains("$schema")) merged_result.erase("$schema");
    
    VRL_LOG_DEBUG(modules::CONFIG, "Final merged config has " + 
                  std::to_string(merged_result.size()) + " top-level keys");
    
    return merged_result;
}

json ConfigLoader::merge_json(const json& base, const json& overlay) {
    // Если overlay пустой или не объект, возвращаем base
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
        // Пропускаем мета-поля
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
    
    // Определяем базовый путь (директория файла)
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
    
    // Выводим содержимое для отладки
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
// parse_target - БЕЗ ИЗМЕНЕНИЙ
// ============================================================================

bool ConfigLoader::parse_target(const json& j, GeneratedTarget& target, bool is_rbs) {
    try {
        if (j.contains("name")) target.name = j["name"].get<std::string>();
        if (j.contains("azimuth_deg")) target.azimuth_deg = j["azimuth_deg"].get<double>();
        if (j.contains("range_km")) target.range_km = j["range_km"].get<double>();
        if (j.contains("rbs_code_octal") && is_rbs) {
            target.rbs_code_octal = j["rbs_code_octal"].get<uint16_t>();
        }
        if (j.contains("uvd_data_dec") && !is_rbs) {
            target.uvd_data_dec = j["uvd_data_dec"].get<uint32_t>();
        }
        if (j.contains("azimuth_speed_deg_per_rev")) {
            target.azimuth_speed_deg_per_rev = j["azimuth_speed_deg_per_rev"].get<double>();
        }
        if (j.contains("range_speed_km_per_rev")) {
            target.range_speed_km_per_rev = j["range_speed_km_per_rev"].get<double>();
        }
        if (j.contains("spi")) target.spi = j["spi"].get<bool>();
        if (j.contains("enabled")) target.enabled = j["enabled"].get<bool>();
        if (j.contains("update_every_n_revolutions")) {
            target.update_every_n_revolutions = j["update_every_n_revolutions"].get<int>();
        }
        if (j.contains("revolution_offset")) target.revolution_offset = j["revolution_offset"].get<int>();
        if (j.contains("altitude_meters")) target.altitude_meters = j["altitude_meters"].get<int>();
        if (j.contains("enable_altitude")) target.enable_altitude = j["enable_altitude"].get<bool>();
        if (j.contains("alternate_code_altitude")) {
            target.alternate_code_altitude = j["alternate_code_altitude"].get<bool>();
        }
        if (j.contains("alternate_data_altitude")) {
            target.alternate_data_altitude = j["alternate_data_altitude"].get<bool>();
        }
        if (j.contains("use_linear_motion")) target.use_linear_motion = j["use_linear_motion"].get<bool>();
        if (j.contains("speed_m_per_s")) target.speed_m_per_s = j["speed_m_per_s"].get<double>();
        if (j.contains("course_deg")) target.course_deg = j["course_deg"].get<double>();
        if (j.contains("initial_x_km")) target.initial_x_km = j["initial_x_km"].get<double>();
        if (j.contains("initial_y_km")) target.initial_y_km = j["initial_y_km"].get<double>();
        
        target.type = is_rbs ? GeneratedTarget::Type::RBS : GeneratedTarget::Type::UVD;
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse target: " + std::string(e.what()));
        return false;
    }
}

// ============================================================================
// parse_config - ОБНОВЛЕН С НОВЫМИ ПОЛЯМИ
// ============================================================================

bool ConfigLoader::parse_config(const json& j, SystemConfig& config) {
    try {
        // Проверяем, что j - объект
        if (!j.is_object()) {
            VRL_LOG_ERROR(modules::CONFIG, "Config root is not an object");
            return false;
        }
        
        if (j.empty()) {
            VRL_LOG_ERROR(modules::CONFIG, "Config is empty");
            return false;
        }
        
        // Radar
        if (j.contains("radar") && j["radar"].is_object()) {
            const auto& r = j["radar"];
            if (r.contains("range_bin_rbs") && r["range_bin_rbs"].is_number()) {
                config.radar.range_bin_rbs = r["range_bin_rbs"].get<double>();
            }
            if (r.contains("range_bin_uvd") && r["range_bin_uvd"].is_number()) {
                config.radar.range_bin_uvd = r["range_bin_uvd"].get<double>();
            }
            if (r.contains("max_azimuth_diff_for_overlap") && r["max_azimuth_diff_for_overlap"].is_number()) {
                config.radar.max_azimuth_diff_for_overlap = r["max_azimuth_diff_for_overlap"].get<double>();
            }
            if (r.contains("max_range_diff_for_overlap") && r["max_range_diff_for_overlap"].is_number()) {
                config.radar.max_range_diff_for_overlap = r["max_range_diff_for_overlap"].get<uint16_t>();
            }
            if (r.contains("min_amplitude") && r["min_amplitude"].is_number()) {
                config.radar.min_amplitude = r["min_amplitude"].get<uint8_t>();
            }
        }
        
        // RBS Simulator
        if (j.contains("rbs") && j["rbs"].is_object()) {
            const auto& r = j["rbs"];
            if (r.contains("snr_db") && r["snr_db"].is_number()) {
                config.simulator.rbs.snr_db = r["snr_db"].get<double>();
            }
            if (r.contains("amp_variation") && r["amp_variation"].is_number()) {
                config.simulator.rbs.amp_variation = r["amp_variation"].get<double>();
            }
            if (r.contains("f1f2_amp_ratio") && r["f1f2_amp_ratio"].is_number()) {
                config.simulator.rbs.f1f2_amp_ratio = r["f1f2_amp_ratio"].get<double>();
            }
        }
        
        // UVD Simulator
        if (j.contains("uvd") && j["uvd"].is_object()) {
            const auto& u = j["uvd"];
            if (u.contains("snr_db") && u["snr_db"].is_number()) {
                config.simulator.uvd.snr_db = u["snr_db"].get<double>();
            }
            if (u.contains("error_probability") && u["error_probability"].is_number()) {
                config.simulator.uvd.error_probability = u["error_probability"].get<double>();
            }
        }
        
        // SLS
        if (j.contains("sls") && j["sls"].is_object()) {
            const auto& s = j["sls"];
            if (s.contains("enabled") && s["enabled"].is_boolean()) {
                config.simulator.sls.enabled = s["enabled"].get<bool>();
            }
            if (s.contains("main_to_sls_ratio") && s["main_to_sls_ratio"].is_number()) {
                config.simulator.sls.main_to_sls_ratio = s["main_to_sls_ratio"].get<double>();
            }
            if (s.contains("sls_attenuation_db") && s["sls_attenuation_db"].is_number()) {
                config.simulator.sls.sls_attenuation_db = s["sls_attenuation_db"].get<double>();
            }
            if (s.contains("sidelobe_probability") && s["sidelobe_probability"].is_number()) {
                config.simulator.sls.sidelobe_probability = s["sidelobe_probability"].get<double>();
            }
        }
        
        // Tracker
        if (j.contains("tracker") && j["tracker"].is_object()) {
            const auto& t = j["tracker"];
            if (t.contains("min_hits_to_confirm") && t["min_hits_to_confirm"].is_number()) {
                config.tracker.min_hits_to_confirm = t["min_hits_to_confirm"].get<int>();
            }
            if (t.contains("max_coast_count") && t["max_coast_count"].is_number()) {
                config.tracker.max_coast_count = t["max_coast_count"].get<int>();
            }
            if (t.contains("max_gate_distance") && t["max_gate_distance"].is_number()) {
                config.tracker.max_gate_distance = t["max_gate_distance"].get<double>();
            }
            if (t.contains("max_gate_azimuth") && t["max_gate_azimuth"].is_number()) {
                config.tracker.max_gate_azimuth = t["max_gate_azimuth"].get<double>();
            }
            if (t.contains("process_noise") && t["process_noise"].is_number()) {
                config.tracker.process_noise = t["process_noise"].get<double>();
            }
            if (t.contains("measurement_noise") && t["measurement_noise"].is_number()) {
                config.tracker.measurement_noise = t["measurement_noise"].get<double>();
            }
            if (t.contains("enable_uvd_tracking") && t["enable_uvd_tracking"].is_boolean()) {
                config.tracker.enable_uvd_tracking = t["enable_uvd_tracking"].get<bool>();
            }
            if (t.contains("enable_rbs_tracking") && t["enable_rbs_tracking"].is_boolean()) {
                config.tracker.enable_rbs_tracking = t["enable_rbs_tracking"].get<bool>();
            }
            if (t.contains("debug_mode") && t["debug_mode"].is_boolean()) {
                config.tracker.debug_mode = t["debug_mode"].get<bool>();
            }
        }
        
        // Processing
        if (j.contains("processing") && j["processing"].is_object()) {
            const auto& p = j["processing"];
            if (p.contains("max_gap_azimuth") && p["max_gap_azimuth"].is_number()) {
                config.processing.max_gap_azimuth = p["max_gap_azimuth"].get<int>();
            }
            if (p.contains("range_window") && p["range_window"].is_number()) {
                config.processing.range_window = p["range_window"].get<int>();
            }
            if (p.contains("range_tolerance") && p["range_tolerance"].is_number()) {
                config.processing.range_tolerance = p["range_tolerance"].get<uint16_t>();
            }
            if (p.contains("min_hits") && p["min_hits"].is_number()) {
                config.processing.min_hits = p["min_hits"].get<int>();
            }
            if (p.contains("output_file") && p["output_file"].is_string()) {
                config.processing.output_file = p["output_file"].get<std::string>();
            }
            if (p.contains("plots_output_file") && p["plots_output_file"].is_string()) {
                config.processing.plots_output_file = p["plots_output_file"].get<std::string>();
            }
            if (p.contains("min_cluster_hits") && p["min_cluster_hits"].is_number()) {
                config.processing.min_cluster_hits = p["min_cluster_hits"].get<int>();
            }
            if (p.contains("range_threshold_bins") && p["range_threshold_bins"].is_number()) {
                config.processing.range_threshold_bins = p["range_threshold_bins"].get<int>();
            }
            if (p.contains("azimuth_threshold_bins") && p["azimuth_threshold_bins"].is_number()) {
                config.processing.azimuth_threshold_bins = p["azimuth_threshold_bins"].get<int>();
            }
            if (p.contains("completion_gap_bins") && p["completion_gap_bins"].is_number()) {
                config.processing.completion_gap_bins = p["completion_gap_bins"].get<int>();
            }
            if (p.contains("min_confidence") && p["min_confidence"].is_number()) {
                config.processing.min_confidence = p["min_confidence"].get<double>();
            }
            if (p.contains("garbled_confidence_threshold") && p["garbled_confidence_threshold"].is_number()) {
                config.processing.garbled_confidence_threshold = p["garbled_confidence_threshold"].get<double>();
            }
            if (p.contains("min_uvd_confidence") && p["min_uvd_confidence"].is_number()) {
                config.processing.min_uvd_confidence = p["min_uvd_confidence"].get<double>();
            }
            if (p.contains("uvd_garbled_threshold") && p["uvd_garbled_threshold"].is_number()) {
                config.processing.uvd_garbled_threshold = p["uvd_garbled_threshold"].get<double>();
            }
        }
        
        // Confidence
        if (j.contains("confidence") && j["confidence"].is_object()) {
            const auto& c = j["confidence"];
            if (c.contains("initial_track_confidence") && c["initial_track_confidence"].is_number()) {
                config.confidence.initial_track_confidence = c["initial_track_confidence"].get<double>();
            }
            if (c.contains("coast_confidence_decay") && c["coast_confidence_decay"].is_number()) {
                config.confidence.coast_confidence_decay = c["coast_confidence_decay"].get<double>();
            }
            if (c.contains("min_track_confidence") && c["min_track_confidence"].is_number()) {
                config.confidence.min_track_confidence = c["min_track_confidence"].get<double>();
            }
            if (c.contains("max_track_confidence") && c["max_track_confidence"].is_number()) {
                config.confidence.max_track_confidence = c["max_track_confidence"].get<double>();
            }
            if (c.contains("confidence_rbs_weight_framing") && c["confidence_rbs_weight_framing"].is_number()) {
                config.confidence.confidence_rbs_weight_framing = c["confidence_rbs_weight_framing"].get<double>();
            }
            if (c.contains("confidence_rbs_weight_snr") && c["confidence_rbs_weight_snr"].is_number()) {
                config.confidence.confidence_rbs_weight_snr = c["confidence_rbs_weight_snr"].get<double>();
            }
            if (c.contains("confidence_rbs_weight_stability") && c["confidence_rbs_weight_stability"].is_number()) {
                config.confidence.confidence_rbs_weight_stability = c["confidence_rbs_weight_stability"].get<double>();
            }
            if (c.contains("confidence_rbs_weight_errors") && c["confidence_rbs_weight_errors"].is_number()) {
                config.confidence.confidence_rbs_weight_errors = c["confidence_rbs_weight_errors"].get<double>();
            }
            if (c.contains("confidence_uvd_weight_snr") && c["confidence_uvd_weight_snr"].is_number()) {
                config.confidence.confidence_uvd_weight_snr = c["confidence_uvd_weight_snr"].get<double>();
            }
            if (c.contains("confidence_uvd_weight_errors") && c["confidence_uvd_weight_errors"].is_number()) {
                config.confidence.confidence_uvd_weight_errors = c["confidence_uvd_weight_errors"].get<double>();
            }
            if (c.contains("confidence_uvd_weight_stability") && c["confidence_uvd_weight_stability"].is_number()) {
                config.confidence.confidence_uvd_weight_stability = c["confidence_uvd_weight_stability"].get<double>();
            }
        }
        
        // Simulator Constants
        if (j.contains("simulator_constants") && j["simulator_constants"].is_object()) {
            const auto& s = j["simulator_constants"];
            if (s.contains("base_signal_power") && s["base_signal_power"].is_number()) {
                config.simulator_constants.base_signal_power = s["base_signal_power"].get<double>();
            }
            if (s.contains("amp_variation_min") && s["amp_variation_min"].is_number()) {
                config.simulator_constants.amp_variation_min = s["amp_variation_min"].get<double>();
            }
            if (s.contains("amp_variation_max") && s["amp_variation_max"].is_number()) {
                config.simulator_constants.amp_variation_max = s["amp_variation_max"].get<double>();
            }
            if (s.contains("uvd_error_threshold") && s["uvd_error_threshold"].is_number()) {
                config.simulator_constants.uvd_error_threshold = s["uvd_error_threshold"].get<double>();
            }
            if (s.contains("max_snr_db") && s["max_snr_db"].is_number()) {
                config.simulator_constants.max_snr_db = s["max_snr_db"].get<double>();
            }
            if (s.contains("min_speed_ms") && s["min_speed_ms"].is_number()) {
                config.simulator_constants.min_speed_ms = s["min_speed_ms"].get<double>();
            }
            if (s.contains("min_time_delta") && s["min_time_delta"].is_number()) {
                config.simulator_constants.min_time_delta = s["min_time_delta"].get<double>();
            }
            if (s.contains("max_mode_c_code") && s["max_mode_c_code"].is_number()) {
                config.simulator_constants.max_mode_c_code = s["max_mode_c_code"].get<int>();
            }
            if (s.contains("max_mode_c_attempts") && s["max_mode_c_attempts"].is_number()) {
                config.simulator_constants.max_mode_c_attempts = s["max_mode_c_attempts"].get<int>();
            }
            if (s.contains("display_beamwidth_deg") && s["display_beamwidth_deg"].is_number()) {
                config.simulator_constants.display_beamwidth_deg = s["display_beamwidth_deg"].get<double>();
            }
            if (s.contains("min_amplitude_ratio_for_separation") && s["min_amplitude_ratio_for_separation"].is_number()) {
                config.simulator_constants.min_amplitude_ratio_for_separation = s["min_amplitude_ratio_for_separation"].get<double>();
            }
        }
        
        // Azimuth
        if (j.contains("azimuth") && j["azimuth"].is_object()) {
            const auto& a = j["azimuth"];
            if (a.contains("azimuth_bins") && a["azimuth_bins"].is_number()) {
                config.azimuth.azimuth_bins = a["azimuth_bins"].get<int>();
                config.azimuth.azimuth_half = config.azimuth.azimuth_bins / 2;
                config.azimuth.azimuth_per_bin_deg = 360.0 / config.azimuth.azimuth_bins;
                config.azimuth.azimuth_per_bin_rad = M_PI / (config.azimuth.azimuth_bins / 2);
            }
        }
        
        // Общие параметры
        if (j.contains("beamwidth_deg") && j["beamwidth_deg"].is_number()) {
            config.beamwidth_deg = j["beamwidth_deg"].get<double>();
        }
        if (j.contains("revolution_time") && j["revolution_time"].is_number()) {
            config.revolution_time = j["revolution_time"].get<double>();
        }
        
        // RBS Targets
        if (j.contains("rbs_targets") && j["rbs_targets"].is_array()) {
            for (const auto& target_json : j["rbs_targets"]) {
                GeneratedTarget target;
                if (parse_target(target_json, target, true)) {
                    config.rbs_targets.push_back(target);
                }
            }
        }
        
        // UVD Targets
        if (j.contains("uvd_targets") && j["uvd_targets"].is_array()) {
            for (const auto& target_json : j["uvd_targets"]) {
                GeneratedTarget target;
                if (parse_target(target_json, target, false)) {
                    config.uvd_targets.push_back(target);
                }
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to parse config: " + std::string(e.what()));
        return false;
    }
}

// ============================================================================
// validate - БЕЗ ИЗМЕНЕНИЙ
// ============================================================================

bool ConfigLoader::validate(const SystemConfig& config, std::string& error) const {
    if (!config.has_targets()) {
        error = "No targets defined in configuration";
        return false;
    }
    
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
    
    return true;
}

// ============================================================================
// to_json - ОБНОВЛЕН С НОВЫМИ ПОЛЯМИ
// ============================================================================

json ConfigLoader::to_json(const SystemConfig& config) {
    json j;
    
    // Radar
    j["radar"] = {
        {"range_bin_rbs", config.radar.range_bin_rbs},
        {"range_bin_uvd", config.radar.range_bin_uvd},
        {"max_azimuth_diff_for_overlap", config.radar.max_azimuth_diff_for_overlap},
        {"max_range_diff_for_overlap", config.radar.max_range_diff_for_overlap},
        {"min_amplitude", config.radar.min_amplitude}
    };
    
    // RBS Simulator
    j["rbs"] = {
        {"snr_db", config.simulator.rbs.snr_db},
        {"amp_variation", config.simulator.rbs.amp_variation},
        {"f1f2_amp_ratio", config.simulator.rbs.f1f2_amp_ratio}
    };
    
    // UVD Simulator
    j["uvd"] = {
        {"snr_db", config.simulator.uvd.snr_db},
        {"error_probability", config.simulator.uvd.error_probability}
    };
    
    // SLS
    j["sls"] = {
        {"enabled", config.simulator.sls.enabled},
        {"main_to_sls_ratio", config.simulator.sls.main_to_sls_ratio},
        {"sls_attenuation_db", config.simulator.sls.sls_attenuation_db},
        {"sidelobe_probability", config.simulator.sls.sidelobe_probability}
    };
    
    // Tracker
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
    
    // Processing
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
    
    // Confidence
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
    
    // Simulator Constants
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
    
    // Azimuth
    j["azimuth"] = {
        {"azimuth_bins", config.azimuth.azimuth_bins}
    };
    
    // Общие параметры
    j["beamwidth_deg"] = config.beamwidth_deg;
    j["revolution_time"] = config.revolution_time;
    
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
    
    return j;
}

} // namespace radar
} // namespace vrl
