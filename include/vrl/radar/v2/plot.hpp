// include/vrl/radar/v2/plot.hpp
#pragma once

#include <cstdint>

namespace vrl {
namespace radar {
namespace v2 {

/**
 * @brief Плот — результат обработки кластера
 * 
 * Содержит всю информацию для обновления или создания трека.
 */
struct Plot {
    // Позиция в метрах
    double x{0.0};
    double y{0.0};
    
    // Позиция в единицах ответов
    uint16_t azimuth_maia{0};
    uint16_t range_bins{0};
    
    // Информация о цели
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    // Тип источника
    enum class SourceType : uint8_t {
        RBS,
        UVD,
        MIXED
    };
    SourceType source_type{SourceType::RBS};
    
    // Уверенность
    double confidence{1.0};
    
    // Связь с исходным кластером (для отладки)
    uint64_t source_cluster_id{0};
    
    /**
     * @brief Проверить, валиден ли плот
     */
    bool is_valid() const {
        return x != 0.0 || y != 0.0;
    }
};

} // namespace v2
} // namespace radar
} // namespace vrl
