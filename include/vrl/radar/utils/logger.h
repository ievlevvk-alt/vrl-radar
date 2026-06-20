// include/vrl/radar/utils/logger.h
#pragma once

#include "../core/logging_config.h"
#include <string>
#include <sstream>
#include <fstream>
#include <mutex>
#include <memory>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>
#include <functional>
#include <map>

// ============================================================================
// КОНТРОЛЬ УРОВНЯ ЛОГИРОВАНИЯ НА ЭТАПЕ КОМПИЛЯЦИИ
// ============================================================================

// Используем другие имена для макросов, чтобы не конфликтовать с enum
#ifdef DEBUG
    #undef DEBUG
#endif

#ifdef TRACE
    #undef TRACE
#endif

// По умолчанию TRACE и DEBUG включены только в Debug сборке
#ifdef CMAKE_BUILD_TYPE_DEBUG
    #define VRL_ENABLE_TRACE_LOGGING 1
    #define VRL_ENABLE_DEBUG_LOGGING 1
#else
    // В Release сборке TRACE отключен, DEBUG может быть включен через флаг
    #ifdef ENABLE_VERBOSE_LOGGING
        #define VRL_ENABLE_DEBUG_LOGGING 1
    #else
        #define VRL_ENABLE_DEBUG_LOGGING 0
    #endif
    #define VRL_ENABLE_TRACE_LOGGING 0
#endif

// Возможность переопределить через CMake
#ifndef VRL_ENABLE_TRACE_LOGGING
    #define VRL_ENABLE_TRACE_LOGGING 0
#endif

#ifndef VRL_ENABLE_DEBUG_LOGGING
    #define VRL_ENABLE_DEBUG_LOGGING 1
#endif

namespace vrl {
namespace radar {
namespace utils {

// ============================================================================
// УРОВНИ ЛОГИРОВАНИЯ
// ============================================================================

enum class LogLevel : uint8_t {
    TRACE = 0,
    DEBUG = 1,
    INFO  = 2,
    WARN  = 3,
    ERROR = 4,
    FATAL = 5,
    OFF   = 6
};

inline const char* log_level_to_string(LogLevel level) {
    switch (level) {
        case LogLevel::TRACE: return "TRACE";
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO ";
        case LogLevel::WARN:  return "WARN ";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        case LogLevel::OFF:   return "OFF  ";
        default: return "???? ";
    }
}

inline LogLevel string_to_log_level(const std::string& level) {
    std::string upper = level;
    std::transform(upper.begin(), upper.end(), upper.begin(), ::toupper);
    
    if (upper == "TRACE") return LogLevel::TRACE;
    if (upper == "DEBUG") return LogLevel::DEBUG;
    if (upper == "INFO") return LogLevel::INFO;
    if (upper == "WARN" || upper == "WARNING") return LogLevel::WARN;
    if (upper == "ERROR") return LogLevel::ERROR;
    if (upper == "FATAL") return LogLevel::FATAL;
    if (upper == "OFF") return LogLevel::OFF;
    
    return LogLevel::INFO;  // default
}

// ============================================================================
// МОДУЛИ ДЛЯ ЛОГИРОВАНИЯ
// ============================================================================

namespace modules {
    const std::string MAIN      = "Main";
    const std::string CONFIG    = "Config";
    const std::string SIMULATOR = "Simulator";
    const std::string TRACKER   = "Tracker";
    const std::string CLUSTER   = "Cluster";
    const std::string GARBLING  = "Garbling";
    const std::string KALMAN    = "Kalman";
    const std::string PROCESSING = "Processing";
    const std::string UTILS     = "Utils";
    const std::string PLAYER    = "Player";
    const std::string CORE      = "Core";     // <-- НОВЫЙ
    const std::string DISPLAY   = "Display";  // <-- НОВЫЙ
}

// ============================================================================
// ЛОГГЕР
// ============================================================================

class Logger {
public:
    static Logger& instance();
    
    void set_default_level(LogLevel level) { default_level_ = level; }
    LogLevel get_default_level() const { return default_level_; }
    
    void set_module_level(const std::string& module, LogLevel level);
    LogLevel get_module_level(const std::string& module) const;
    
    void set_console_output(bool enable) { console_output_ = enable; }
    void set_file_output(const std::string& filename);
    void set_timestamp_format(const std::string& format) { timestamp_format_ = format; }
    
    void configure(const LoggingConfig& config);
    
    using LogHandler = std::function<void(LogLevel, const std::string&, const std::string&)>;
    void add_handler(LogHandler handler) { handlers_.push_back(handler); }
    
    void log(LogLevel level, const std::string& module, const std::string& message);
    
    class LogStream {
    public:
        LogStream(Logger& logger, LogLevel level, const std::string& module)
            : logger_(logger), level_(level), module_(module) {}
        
        ~LogStream() {
            if (!buffer_.str().empty()) {
                logger_.log(level_, module_, buffer_.str());
            }
        }
        
        template<typename T>
        LogStream& operator<<(const T& value) {
            buffer_ << value;
            return *this;
        }
        
        LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
            buffer_ << manip;
            return *this;
        }
        
    private:
        Logger& logger_;
        LogLevel level_;
        std::string module_;
        std::ostringstream buffer_;
    };

private:
    Logger();
    ~Logger();
    
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    std::string get_timestamp() const;
    std::string format_message(LogLevel level, const std::string& module, 
                               const std::string& message) const;
    void write_to_console(LogLevel level, const std::string& formatted);
    void write_to_file(LogLevel level, const std::string& formatted);
    void notify_handlers(LogLevel level, const std::string& module, 
                         const std::string& message);
    
    LogLevel default_level_{LogLevel::INFO};
    std::map<std::string, LogLevel> module_levels_;
    bool console_output_{true};
    std::string timestamp_format_{"%Y-%m-%d %H:%M:%S"};
    
    std::ofstream file_;
    mutable std::mutex mutex_;
    std::vector<LogHandler> handlers_;
};

// ============================================================================
// МАКРОСЫ ДЛЯ УДОБНОГО ЛОГИРОВАНИЯ
// ============================================================================

// TRACE - полностью отключается на этапе компиляции в Release сборке
#if VRL_ENABLE_TRACE_LOGGING
    #define VRL_LOG_TRACE(module, msg) \
        vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::TRACE, module, msg)
    #define VRL_LOG_TRACE_IF(module, condition, msg) \
        if (condition) { \
            vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::TRACE, module, msg); \
        }
#else
    #define VRL_LOG_TRACE(module, msg) ((void)0)
    #define VRL_LOG_TRACE_IF(module, condition, msg) ((void)0)
#endif

// DEBUG - отключается только если явно выключен
#if VRL_ENABLE_DEBUG_LOGGING
    #define VRL_LOG_DEBUG(module, msg) \
        vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::DEBUG, module, msg)
    #define VRL_LOG_DEBUG_IF(module, condition, msg) \
        if (condition) { \
            vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::DEBUG, module, msg); \
        }
#else
    #define VRL_LOG_DEBUG(module, msg) ((void)0)
    #define VRL_LOG_DEBUG_IF(module, condition, msg) ((void)0)
#endif

// INFO и выше - всегда включены
#define VRL_LOG_INFO(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::INFO, module, msg)

#define VRL_LOG_WARN(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::WARN, module, msg)

#define VRL_LOG_ERROR(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::ERROR, module, msg)

#define VRL_LOG_FATAL(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::FATAL, module, msg)

// Вербозное логирование
#ifdef ENABLE_VERBOSE_LOGGING
    #define VRL_LOG_VERBOSE(module, msg) \
        VRL_LOG_DEBUG(module, msg)
    #define VRL_LOG_VERBOSE_STREAM(module) \
        vrl::radar::utils::Logger::LogStream(vrl::radar::utils::Logger::instance(), \
                                             vrl::radar::utils::LogLevel::DEBUG, module)
#else
    #define VRL_LOG_VERBOSE(module, msg) ((void)0)
    #define VRL_LOG_VERBOSE_STREAM(module) \
        std::ostringstream().flush(), vrl::radar::utils::Logger::LogStream( \
            vrl::radar::utils::Logger::instance(), \
            vrl::radar::utils::LogLevel::OFF, module)
#endif

#define VRL_LOG_IF(level, module, condition, msg) \
    if (condition) { \
        vrl::radar::utils::Logger::instance().log(level, module, msg); \
    }

#define VRL_LOG_STREAM(level, module) \
    vrl::radar::utils::Logger::LogStream(vrl::radar::utils::Logger::instance(), level, module)

} // namespace utils
} // namespace radar
} // namespace vrl
