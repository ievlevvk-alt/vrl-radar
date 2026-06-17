// src/config_parser.cpp
#include "radar/config_parser.h"
#include "radar/radar_system.h"
#include <iostream>

namespace radar {

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
        
        // Удаляем BOM если есть
        line = remove_bom(line);
        
        // Удаляем комментарии (только после символа #)
        // Но делаем это аккуратно, чтобы не затронуть # в строковых значениях
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
        
        // Удаляем пробелы и невидимые символы
        line = trim(line);
        if (line.empty()) continue;
        
        // Проверяем секцию
        if (line.front() == '[' && line.back() == ']') {
            current_section_ = line.substr(1, line.length() - 2);
            current_section_ = trim(current_section_);
            sections_[current_section_] = {};
            continue;
        }
        
        // Проверяем ключ=значение
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
    // Удаляем кавычки если есть
    if (value.length() >= 2) {
        if ((value.front() == '"' && value.back() == '"') ||
            (value.front() == '\'' && value.back() == '\'')) {
            value = value.substr(1, value.length() - 2);
        }
    }
    return trim(value);
}

std::vector<std::string> ConfigParser::get_keys(const std::string& section) const {
    auto it = sections_.find(section);
    if (it == sections_.end()) return {};
    
    std::vector<std::string> keys;
    for (const auto& [key, _] : it->second) {
        keys.push_back(key);
    }
    return keys;
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
    // Используем безопасное преобразование с значением по умолчанию
    auto get_or = [&](const std::string& k, auto default_val) {
        auto val = safe_stod<decltype(default_val)>(value);
        return val.value_or(default_val);
    };
    
    if (key == "name") {
        target.name = value;
    } else if (key == "azimuth_deg") {
        target.azimuth_deg = get_or(key, 0.0);
    } else if (key == "range_km") {
        target.range_km = get_or(key, 0.0);
    } else if (key == "rbs_code_octal") {
        // Восьмеричное число
        try {
            target.rbs_code_octal = static_cast<uint16_t>(
                std::stoul(value, nullptr, 8));
        } catch (...) {
            target.rbs_code_octal = 0;
        }
    } else if (key == "uvd_data_dec") {
        target.uvd_data_dec = get_or(key, 0u);
    } else if (key == "azimuth_speed_deg_per_rev") {
        target.azimuth_speed_deg_per_rev = get_or(key, 0.0);
    } else if (key == "range_speed_km_per_rev") {
        target.range_speed_km_per_rev = get_or(key, 0.0);
    } else if (key == "spi") {
        target.spi = get_or(key, false);
    } else if (key == "enabled") {
        target.enabled = get_or(key, false);
    } else if (key == "update_every_n_revolutions") {
        target.update_every_n_revolutions = get_or(key, 1);
    } else if (key == "revolution_offset") {
        target.revolution_offset = get_or(key, 0);
    } else if (key == "altitude_meters") {
        target.altitude_meters = get_or(key, 0);
    } else if (key == "enable_altitude") {
        target.enable_altitude = get_or(key, false);
    } else if (key == "alternate_code_altitude") {
        target.alternate_code_altitude = get_or(key, false);
    } else if (key == "alternate_data_altitude") {
        target.alternate_data_altitude = get_or(key, false);
    } else if (key == "use_linear_motion") {
        target.use_linear_motion = get_or(key, false);
    } else if (key == "speed_m_per_s") {
        target.speed_m_per_s = get_or(key, 0.0);
    } else if (key == "course_deg") {
        target.course_deg = get_or(key, 0.0);
    } else if (key == "initial_x_km") {
        target.initial_x_km = get_or(key, 0.0);
    } else if (key == "initial_y_km") {
        target.initial_y_km = get_or(key, 0.0);
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

// Реализация ConfigBuilder::build
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

// Реализация SystemConfig::load_from_file
SystemConfig SystemConfig::load_from_file(const std::string& filename) {
    ConfigParser parser;
    if (!parser.load(filename)) {
        std::cerr << "Warning: Cannot load config from " << filename << "\n";
        return SystemConfig{};
    }
    return ConfigBuilder::build(parser);
}

// Реализация SystemConfig::load_from_parser
SystemConfig SystemConfig::load_from_parser(const ConfigParser& parser) {
    return ConfigBuilder::build(parser);
}

} // namespace radar
