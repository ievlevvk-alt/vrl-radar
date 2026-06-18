// src/utils/logger.cpp
#include "vrl/radar/utils/logger.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace vrl {
namespace radar {
namespace utils {

Logger& Logger::instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() {
    level_ = LogLevel::INFO;
    console_output_ = true;
}

Logger::~Logger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void Logger::set_file_output(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (file_.is_open()) {
        file_.close();
    }
    
    file_.open(filename, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
        std::cerr << "[Logger] Failed to open log file: " << filename << std::endl;
    } else {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        file_ << "\n========================================\n";
        file_ << "Log started at: " << std::ctime(&time_t);
        file_ << "========================================\n\n";
        file_.flush();
    }
}

void Logger::log(LogLevel level, const std::string& module, const std::string& message) {
    if (level < level_) return;
    if (level == LogLevel::OFF) return;
    
    auto formatted = format_message(level, module, message);
    
    write_to_console(level, formatted);
    write_to_file(level, formatted);
    notify_handlers(level, module, message);
}

std::string Logger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::tm tm_buf;
    localtime_r(&time_t, &tm_buf);
    
    std::ostringstream ss;
    ss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

std::string Logger::format_message(LogLevel level, const std::string& module, 
                                   const std::string& message) const {
    std::ostringstream ss;
    ss << "[" << get_timestamp() << "] "
       << "[" << log_level_to_string(level) << "] "
       << "[" << module << "] "
       << message;
    return ss.str();
}

void Logger::write_to_console(LogLevel level, const std::string& formatted) {
    if (!console_output_) return;
    
    #ifdef __unix__
    const char* color = "";
    switch (level) {
        case LogLevel::TRACE: color = "\033[37m"; break;
        case LogLevel::DEBUG: color = "\033[36m"; break;
        case LogLevel::INFO:  color = "\033[32m"; break;
        case LogLevel::WARN:  color = "\033[33m"; break;
        case LogLevel::ERROR: color = "\033[31m"; break;
        case LogLevel::FATAL: color = "\033[41;37m"; break;
        default: color = "\033[0m"; break;
    }
    const char* reset = "\033[0m";
    std::cerr << color << formatted << reset << std::endl;
    #else
    std::cerr << formatted << std::endl;
    #endif
}

void Logger::write_to_file(LogLevel level, const std::string& formatted) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_.is_open()) {
        file_ << formatted << std::endl;
        file_.flush();
    }
}

void Logger::notify_handlers(LogLevel level, const std::string& module, 
                             const std::string& message) {
    for (const auto& handler : handlers_) {
        try {
            handler(level, module, message);
        } catch (const std::exception& e) {
            std::cerr << "[Logger] Handler exception: " << e.what() << std::endl;
        }
    }
}

} // namespace utils
} // namespace radar
} // namespace vrl
