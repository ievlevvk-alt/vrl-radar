// include/vrl/radar/processing/uvd_processor.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "reply_processor.h"
#include "range_grouper.h"  // <-- ДОБАВЛЯЕМ
#include <optional>
#include <vector>

namespace vrl {
namespace radar {

/**
 * @brief Обработчик UVD ответов
 * 
 * Отвечает за анализ UVD ответов, декодирование данных и формирование отчетов
 */
class UVDProcessor {
public:
    /**
     * @brief Конструктор
     * @param config конфигурация радара
     */
    explicit UVDProcessor(const RadarConfig& config);
    
    /**
     * @brief Обработать группу UVD ответов
     * @param group группа ответов
     * @return отчет о цели (если найден)
     */
    std::optional<TargetReport> process_group(const RangeGrouper::RangeGroup& group);
    
    /**
     * @brief Проверить, является ли ответ боковым лепестком
     * @param reply UVD ответ
     * @return true если боковой лепесток
     */
    bool is_sidelobe(const UVDReply& reply) const;
    
    /**
     * @brief Установить минимальную уверенность
     * @param threshold порог уверенности (0.0 - 1.0)
     */
    void set_min_confidence(double threshold) { min_confidence_ = threshold; }
    
    /**
     * @brief Установить порог для определения перекрытия
     * @param threshold порог (0.0 - 1.0)
     */
    void set_garbled_threshold(double threshold) { garbled_threshold_ = threshold; }
    
    /**
     * @brief Установить минимальное количество попаданий
     * @param hits минимальное количество
     */
    void set_min_hits(int hits) { min_hits_ = hits; }
    
private:
    /**
     * @brief Вычислить средний азимут
     * @param azimuths список азимутов
     * @return средний азимут в бинах
     */
    double average_azimuth(const std::vector<uint16_t>& azimuths);
    
    /**
     * @brief Выбрать лучшие данные из группы
     * @param group группа ответов
     * @param best_confidence выходная уверенность
     * @return лучшие данные
     */
    uint32_t select_best_data(const RangeGrouper::RangeGroup& group, double& best_confidence);
    
    /**
     * @brief Декодировать информацию UVD
     * @param data20 сырые данные
     * @param report отчет для заполнения
     */
    void decode_uvd_info(uint32_t data20, TargetReport& report);
    
    RadarConfig config_;
    ReplyProcessor reply_processor_;
    double min_confidence_{0.3};
    double garbled_threshold_{0.5};
    int min_hits_{2};
};

} // namespace radar
} // namespace vrl
