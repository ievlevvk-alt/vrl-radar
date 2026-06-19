// include/vrl/radar/core/types.h
#pragma once

#include <cstdint>
#include <cmath>

namespace vrl {
namespace radar {

// Конфигурация радара
struct RadarConfig {
    double range_bin_rbs{30.0};        // метры на дискрет для RBS
    double range_bin_uvd{60.0};        // метры на дискрет для УВД
    double beamwidth_deg{5.0};         // Ширина диаграммы направленности в градусах    
    
    static constexpr uint16_t azimuth_max = 4096;
    static constexpr double azimuth_per_bin = 360.0 / azimuth_max; // ≈ 0.08789°
    
    double max_azimuth_diff_for_overlap{2.0};   // в дискретах азимута
    uint16_t max_range_diff_for_overlap{10};    // в дискретах дальности
    uint8_t min_amplitude{10};                  // минимальная амплитуда для импульса
};

// Вспомогательная функция для преобразования координат
inline void polar_to_xy(double range_m, double azimuth_deg, double& x, double& y) {
    double az_rad = azimuth_deg * M_PI / 180.0;
    x = range_m * std::sin(az_rad);
    y = range_m * std::cos(az_rad);
}

// Состояние трека
enum class TrackState {
    NEW,        // Новый трек (требует подтверждения)
    ACTIVE,     // Активный трек
    COASTING,   // Потерян, но еще не сброшен
    DROPPED     // Сброшен
};

} // namespace radar
} // namespace vrl
