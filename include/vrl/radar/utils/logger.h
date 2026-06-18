// include/vrl/radar/utils/logger.h
#pragma once

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

// ============================================================================
// МОДУЛИ ДЛЯ ЛОГИРОВАНИЯ
// ============================================================================

namespace modules {
    constexpr const char* CONFIG     = "Config";
    constexpr const char* TRACKER    = "Tracker";
    constexpr const char* CLUSTER    = "Cluster";
    constexpr const char* SIMULATOR  = "Simulator";
    constexpr const char* PLAYER     = "Player";
    constexpr const char* GARBLING   = "Garbling";
    constexpr const char* KALMAN     = "Kalman";
    constexpr const char* UTILS      = "Utils";
    constexpr const char* MAIN       = "Main";
    constexpr const char* PROCESSING = "Processing";
}

// ============================================================================
// ЛОГГЕР
// ============================================================================

class Logger {
public:
    static Logger& instance();
    
    void set_level(LogLevel level) { level_ = level; }
    LogLevel get_level() const { return level_; }
    
    void set_console_output(bool enable) { console_output_ = enable; }
    void set_file_output(const std::string& filename);
    void set_timestamp_format(const std::string& format) { timestamp_format_ = format; }
    
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
    
    LogLevel level_{LogLevel::INFO};
    bool console_output_{true};
    std::string timestamp_format_{"%Y-%m-%d %H:%M:%S"};
    
    std::ofstream file_;
    std::mutex mutex_;
    std::vector<LogHandler> handlers_;
};

// ============================================================================
// МАКРОСЫ ДЛЯ УДОБНОГО ЛОГИРОВАНИЯ
// ============================================================================

// Основные макросы логирования
#define VRL_LOG_TRACE(module, msg) \
    do { \
        if (true) { \
            vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::TRACE, module, msg); \
        } \
    } while(0)

#define VRL_LOG_DEBUG(module, msg) \
    do { \
        if (true) { \
            vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::DEBUG, module, msg); \
        } \
    } while(0)

#define VRL_LOG_INFO(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::INFO, module, msg)

#define VRL_LOG_WARN(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::WARN, module, msg)

#define VRL_LOG_ERROR(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::ERROR, module, msg)

#define VRL_LOG_FATAL(module, msg) \
    vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::FATAL, module, msg)

// Вербозное логирование - включается только если определена ENABLE_VERBOSE_LOGGING
#ifdef ENABLE_VERBOSE_LOGGING
    #define VRL_LOG_VERBOSE(module, msg) \
        vrl::radar::utils::Logger::instance().log(vrl::radar::utils::LogLevel::DEBUG, module, msg)
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
