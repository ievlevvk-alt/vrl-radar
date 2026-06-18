// include/vrl/radar/core/config_loader.hpp
#pragma once

#include "config.h"
#include <nlohmann/json.hpp>
#include <string>

namespace vrl {
namespace radar {

using json = nlohmann::json;

/**
 * @brief Загрузчик конфигурации из JSON файлов
 * 
 * Использует nlohmann/json для парсинга. Не изменяет существующую
 * структуру SystemConfig и GeneratedTarget.
 */
class ConfigLoader {
public:
    /**
     * @brief Загрузить конфигурацию из JSON файла
     * @param filename путь к файлу
     * @param config выходная структура
     * @return true в случае успеха
     */
    bool load(const std::string& filename, SystemConfig& config);
    
    /**
     * @brief Загрузить конфигурацию из JSON строки
     * @param content JSON строка
     * @param config выходная структура
     * @return true в случае успеха
     */
    bool load_from_string(const std::string& content, SystemConfig& config);
    
    /**
     * @brief Сохранить конфигурацию в JSON файл
     * @param config структура конфигурации
     * @param filename путь к файлу
     * @return true в случае успеха
     */
    bool save(const SystemConfig& config, const std::string& filename);
    
    /**
     * @brief Валидация конфигурации
     * @param config структура конфигурации
     * @param error сообщение об ошибке
     * @return true если конфигурация валидна
     */
    bool validate(const SystemConfig& config, std::string& error) const;

private:
    bool parse_target(const json& j, GeneratedTarget& target, bool is_rbs);
    bool parse_config(const json& j, SystemConfig& config);
    json to_json(const SystemConfig& config);
};

} // namespace radar
} // namespace vrl
