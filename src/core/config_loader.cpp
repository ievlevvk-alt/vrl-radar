// src/core/config_loader.cpp
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/utils/logger.h"
#include <fstream>
#include <set>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

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

bool ConfigLoader::parse_config(const json& j, SystemConfig& config) {
    try {
        // Radar
        if (j.contains("radar")) {
            const auto& r = j["radar"];
            if (r.contains("range_bin_rbs")) config.radar.range_bin_rbs = r["range_bin_rbs"].get<double>();
            if (r.contains("range_bin_uvd")) config.radar.range_bin_uvd = r["range_bin_uvd"].get<double>();
            if (r.contains("max_azimuth_diff_for_overlap")) {
                config.radar.max_azimuth_diff_for_overlap = r["max_azimuth_diff_for_overlap"].get<double>();
            }
            if (r.contains("max_range_diff_for_overlap")) {
                config.radar.max_range_diff_for_overlap = r["max_range_diff_for_overlap"].get<uint16_t>();
            }
            if (r.contains("min_amplitude")) config.radar.min_amplitude = r["min_amplitude"].get<uint8_t>();
        }
        
        // RBS Simulator
        if (j.contains("rbs")) {
            const auto& r = j["rbs"];
            if (r.contains("snr_db")) config.simulator.rbs.snr_db = r["snr_db"].get<double>();
            if (r.contains("amp_variation")) config.simulator.rbs.amp_variation = r["amp_variation"].get<double>();
            if (r.contains("f1f2_amp_ratio")) config.simulator.rbs.f1f2_amp_ratio = r["f1f2_amp_ratio"].get<double>();
        }
        
        // UVD Simulator
        if (j.contains("uvd")) {
            const auto& u = j["uvd"];
            if (u.contains("snr_db")) config.simulator.uvd.snr_db = u["snr_db"].get<double>();
            if (u.contains("error_probability")) config.simulator.uvd.error_probability = u["error_probability"].get<double>();
        }
        
        // SLS
        if (j.contains("sls")) {
            const auto& s = j["sls"];
            if (s.contains("enabled")) config.simulator.sls.enabled = s["enabled"].get<bool>();
            if (s.contains("main_to_sls_ratio")) config.simulator.sls.main_to_sls_ratio = s["main_to_sls_ratio"].get<double>();
            if (s.contains("sls_attenuation_db")) config.simulator.sls.sls_attenuation_db = s["sls_attenuation_db"].get<double>();
            if (s.contains("sidelobe_probability")) {
                config.simulator.sls.sidelobe_probability = s["sidelobe_probability"].get<double>();
            }
        }
        
        // Tracker
        if (j.contains("tracker")) {
            const auto& t = j["tracker"];
            if (t.contains("min_hits_to_confirm")) config.tracker.min_hits_to_confirm = t["min_hits_to_confirm"].get<int>();
            if (t.contains("max_coast_count")) config.tracker.max_coast_count = t["max_coast_count"].get<int>();
            if (t.contains("max_gate_distance")) config.tracker.max_gate_distance = t["max_gate_distance"].get<double>();
            if (t.contains("max_gate_azimuth")) config.tracker.max_gate_azimuth = t["max_gate_azimuth"].get<double>();
            if (t.contains("process_noise")) config.tracker.process_noise = t["process_noise"].get<double>();
            if (t.contains("measurement_noise")) config.tracker.measurement_noise = t["measurement_noise"].get<double>();
            if (t.contains("enable_uvd_tracking")) config.tracker.enable_uvd_tracking = t["enable_uvd_tracking"].get<bool>();
            if (t.contains("enable_rbs_tracking")) config.tracker.enable_rbs_tracking = t["enable_rbs_tracking"].get<bool>();
            if (t.contains("debug_mode")) config.tracker.debug_mode = t["debug_mode"].get<bool>();
        }
        
        // Processing
        if (j.contains("processing")) {
            const auto& p = j["processing"];
            if (p.contains("max_gap_azimuth")) config.processing.max_gap_azimuth = p["max_gap_azimuth"].get<int>();
            if (p.contains("range_window")) config.processing.range_window = p["range_window"].get<int>();
            if (p.contains("range_tolerance")) config.processing.range_tolerance = p["range_tolerance"].get<uint16_t>();
            if (p.contains("min_hits")) config.processing.min_hits = p["min_hits"].get<int>();
            if (p.contains("output_file")) config.processing.output_file = p["output_file"].get<std::string>();
            // Дополнительные поля для processing
            if (p.contains("range_threshold_bins")) {
                // Эти поля могут быть в старом config.h, но мы их игнорируем если их нет
                // или можно добавить если они есть
            }
        }
        
        // Общие параметры
        if (j.contains("beamwidth_deg")) config.beamwidth_deg = j["beamwidth_deg"].get<double>();
        // revolution_time нет в SystemConfig, пропускаем
        
        // RBS Targets
        if (j.contains("rbs_targets")) {
            for (const auto& target_json : j["rbs_targets"]) {
                GeneratedTarget target;
                if (parse_target(target_json, target, true)) {
                    config.rbs_targets.push_back(target);
                }
            }
        }
        
        // UVD Targets
        if (j.contains("uvd_targets")) {
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
        {"output_file", config.processing.output_file}
    };
    
    // Общие параметры
    j["beamwidth_deg"] = config.beamwidth_deg;
    // revolution_time нет в SystemConfig, пропускаем
    
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

bool ConfigLoader::load(const std::string& filename, SystemConfig& config) {
    VRL_LOG_INFO(modules::CONFIG, "Loading config from: " + filename);
    
    try {
        std::ifstream file(filename);
        if (!file.is_open()) {
            VRL_LOG_ERROR(modules::CONFIG, "Cannot open file: " + filename);
            return false;
        }
        
        json j;
        file >> j;
        
        if (!parse_config(j, config)) {
            return false;
        }
        
        std::string error;
        if (!validate(config, error)) {
            VRL_LOG_ERROR(modules::CONFIG, "Config validation failed: " + error);
            return false;
        }
        
        VRL_LOG_INFO(modules::CONFIG, "Config loaded successfully");
        VRL_LOG_DEBUG(modules::CONFIG, "RBS targets: " + std::to_string(config.rbs_targets.size()));
        VRL_LOG_DEBUG(modules::CONFIG, "UVD targets: " + std::to_string(config.uvd_targets.size()));
        
        return true;
    } catch (const std::exception& e) {
        VRL_LOG_ERROR(modules::CONFIG, "Failed to load config: " + std::string(e.what()));
        return false;
    }
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

} // namespace radar
} // namespace vrl
