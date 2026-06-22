// include/vrl/radar/core/track.hpp
#pragma once

#include "types.h"
#include "../core/replies.h"
#include "../core/filter_types.hpp"
#include <vector>
#include <array>
#include <cstdint>

namespace vrl {
namespace radar {


/**
 * @brief Структура трека
 * 
 * Содержит всю информацию о сопровождаемой цели.
 */
struct Track {
    static constexpr size_t DEFAULT_MAX_HISTORY = 20;
    
    uint64_t id{0};
    TrackState state{TrackState::NEW};
    
    double x{0.0}, y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    double vx{0.0}, vy{0.0};
    double ground_speed{0.0};
    double course_deg{0.0};
    
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    uint32_t first_revolution{0};
    uint32_t last_revolution{0};
    uint32_t last_update_revolution{0};
    uint32_t coast_count{0};
    uint32_t hit_count{0};
    
    double confidence{0.0};
    double position_error{0.0};
    bool code_reliable{true};
    bool altitude_reliable{true};
    
    // Тип фильтра, используемый для этого трека
    FilterType filter_type;  // <-- ДОБАВЛЯЕМ
    
    /**
     * @brief Проверить, подтверждён ли трек
     */
    bool is_confirmed() const {
        return hit_count >= 3 && state == TrackState::ACTIVE;
    }
    
    /**
     * @brief Добавить отчёт в историю
     */
    void add_history(const TargetReport& report);
    
    /**
     * @brief Получить историю
     */
    std::vector<TargetReport> get_history() const;
    
    /**
     * @brief Получить последний отчёт
     */
    const TargetReport* get_last_report() const;
    
    /**
     * @brief Очистить историю
     */
    void clear_history();
    
private:
    std::vector<TargetReport> history_;
    static constexpr size_t MAX_HISTORY = 20;
};

} // namespace radar
} // namespace vrl
