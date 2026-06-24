// include/vrl/radar/v2/grid_config.hpp
#pragma once

namespace vrl {
namespace radar {
namespace v2 {

struct GridConfig {
    // === Размеры ячеек ===
    double cell_size_km{5.0};
    double max_range_km{400.0};
    
    // === Кольца соседей ===
    int rings_near{1};
    int rings_far{2};
    double far_threshold_km{150.0};
    
    // === Параметры движения ===
    double min_speed_mps{0.0};
    double max_speed_mps{300.0};
    double revolution_time_s{5.0};
    
    // === Поиск кандидатов ===
    double max_candidate_distance_km{10.0};
    
    // === Параметры эллиптического строба ===
    double range_gate_bins_near{5.0};
    double range_gate_bins_mid{10.0};
    double range_gate_bins_far{20.0};
    
    double azimuth_gate_maia_near{10.0};
    double azimuth_gate_maia_mid{20.0};
    double azimuth_gate_maia_far{40.0};
    
    double coast_gate_expansion{1.5};

    // === COASTING ===
    int max_coast_revolutions{3};   // максимальное число оборотов без обновления
};

} // namespace v2
} // namespace radar
} // namespace vrl
