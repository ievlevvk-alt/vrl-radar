// include/radar/config_parser.h
#pragma once

#include <string>
#include <map>
#include <vector>
#include <optional>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <iostream>
#include <cmath>

// Forward declaration
namespace radar {
    struct GeneratedTarget;
    struct SystemConfig;
}

namespace radar {

// Вспомогательная функция для обрезки строк
inline std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\n\r\f\v");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\n\r\f\v");
    return str.substr(first, last - first + 1);
}

// Удаление BOM (Byte Order Mark)
inline std::string remove_bom(const std::string& str) {
    if (str.length() >= 3 && 
        static_cast<unsigned char>(str[0]) == 0xEF &&
        static_cast<unsigned char>(str[1]) == 0xBB &&
        static_cast<unsigned char>(str[2]) == 0xBF) {
        return str.substr(3);
    }
    return str;
}

// Безопасное преобразование строки в число
template<typename T>
std::optional<T> safe_stod(const std::string& str) {
    if (str.empty()) return std::nullopt;
    
    std::string cleaned = trim(str);
    if (cleaned.empty()) return std::nullopt;
    
    // Удаляем всё после пробела (может быть комментарий)
    size_t space_pos = cleaned.find_first_of(" \t");
    if (space_pos != std::string::npos) {
        cleaned = cleaned.substr(0, space_pos);
    }
    
    // Проверяем, что строка содержит только допустимые символы
    bool has_digit = false;
    bool has_decimal = false;
    bool has_sign = false;
    bool valid = true;
    
    for (size_t i = 0; i < cleaned.length(); ++i) {
        char c = cleaned[i];
        if (c >= '0' && c <= '9') {
            has_digit = true;
        } else if (c == '.') {
            if (has_decimal) { valid = false; break; }
            has_decimal = true;
        } else if (c == '-' || c == '+') {
            if (has_sign || i != 0) { valid = false; break; }
            has_sign = true;
        } else if (c == 'e' || c == 'E') {
            // Научная нотация - разрешаем
            continue;
        } else {
            valid = false;
            break;
        }
    }
    
    if (!valid || !has_digit) {
        return std::nullopt;
    }
    
    try {
        // Для целых чисел используем stoi/stoul
        if constexpr (std::is_integral<T>::value) {
            if constexpr (std::is_unsigned<T>::value) {
                // Восьмеричные числа с ведущим нулём
                if (cleaned.length() > 1 && cleaned[0] == '0') {
                    return static_cast<T>(std::stoul(cleaned, nullptr, 8));
                }
                return static_cast<T>(std::stoul(cleaned));
            } else {
                return static_cast<T>(std::stoi(cleaned));
            }
        } else {
            // Для чисел с плавающей точкой
            return static_cast<T>(std::stod(cleaned));
        }
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

// Специализации для безопасного парсинга
template<>
inline std::optional<bool> safe_stod<bool>(const std::string& str) {
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

template<>
inline std::optional<std::string> safe_stod<std::string>(const std::string& str) {
    return trim(str);
}

// Базовый парсер конфигурации
class ConfigParser {
public:
    bool load(const std::string& filename);
    
    std::optional<std::string> get_string(const std::string& key, 
                                          const std::string& section = "") const;
    
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
    
    std::vector<std::string> get_keys(const std::string& section = "") const;
    std::vector<std::string> get_sections() const;
    std::vector<GeneratedTarget> parse_targets(const std::string& section) const;
    
private:
    std::map<std::string, std::map<std::string, std::string>> sections_;
    std::string current_section_;
    int line_number_{0};
    
    void parse_target_field(GeneratedTarget& target, const std::string& key, 
                           const std::string& value) const;
};

// Фасад для создания SystemConfig из парсера
class ConfigBuilder {
public:
    static SystemConfig build(const ConfigParser& parser);
};

} // namespace radar
