// include/vrl/radar/core/config_loader.hpp
#pragma once

#include "config.h"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
#include <filesystem>

namespace vrl {
namespace radar {

using json = nlohmann::json;

/**
 * @brief Загрузчик конфигурации из JSON файлов с поддержкой include
 * 
 * Использует nlohmann/json для парсинга. Поддерживает директиву _includes
 * для включения других JSON файлов.
 */
class ConfigLoader {
public:
    /**
     * @brief Загрузить конфигурацию из JSON файла с поддержкой include
     * @param filename путь к основному файлу
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
    /**
     * @brief Загрузить JSON с обработкой include директив
     * @param filename путь к файлу
     * @param base_path базовый путь для разрешения относительных путей
     * @return объединенный JSON объект
     */
    json load_with_includes(const std::string& filename, 
                            const std::filesystem::path& base_path = "");
    
    /**
     * @brief Объединить два JSON объекта (рекурсивное слияние)
     */
    json merge_json(const json& base, const json& overlay);
    
    bool parse_target(const json& j, GeneratedTarget& target, bool is_rbs);
    bool parse_config(const json& j, SystemConfig& config);
    json to_json(const SystemConfig& config);
    
    std::vector<std::string> loaded_files_;  // Для предотвращения циклических включений
};

} // namespace radar
} // namespace vrl
