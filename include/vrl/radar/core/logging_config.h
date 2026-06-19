// include/vrl/radar/core/logging_config.h
#pragma once

#include <string>
#include <map>

namespace vrl {
namespace radar {

// ============================================================================
// LOGGING CONFIG
// ============================================================================

struct LoggingConfig {
    bool console_enabled{true};
    bool file_enabled{true};
    std::string log_file{"radar.log"};
    std::string timestamp_format{"%Y-%m-%d %H:%M:%S"};
    
    // Уровни логирования для каждого модуля
    std::map<std::string, std::string> module_levels;
    
    // Получить уровень для модуля с fallback к INFO
    std::string get_module_level(const std::string& module) const {
        auto it = module_levels.find(module);
        if (it != module_levels.end()) {
            return it->second;
        }
        return "INFO";
    }
    
    // Установить уровень для модуля
    void set_module_level(const std::string& module, const std::string& level) {
        module_levels[module] = level;
    }
};

} // namespace radar
} // namespace vrl
