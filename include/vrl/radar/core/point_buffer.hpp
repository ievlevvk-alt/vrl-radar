// include/vrl/radar/core/point_buffer.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>  // <-- ДОБАВЛЯЕМ для size_t
#include "replies.h" // <-- ДОБАВЛЯЕМ для RBSReply и UVDReply

namespace vrl {
namespace radar {

/**
 * @brief Точка (единичный ответ) для хранения в буфере
 */
struct StoredPoint {
    uint16_t azimuth{0};
    uint16_t range{0};
    uint16_t amplitude{0};
    bool is_rbs{true};
    
    bool spi{false};
    uint16_t code12{0};
    uint32_t data20{0};
    
    const RBSReply* rbs_reply{nullptr};
    const UVDReply* uvd_reply{nullptr};
};

/**
 * @brief Кольцевой буфер для хранения точек
 * 
 * Синглтон. Размер буфера задается через конфигурацию.
 * Индексы циклически перезаписываются.
 */
class PointBuffer {
public:
    static PointBuffer& instance();
    
    void init(size_t size = 65536);
    size_t add_point(const StoredPoint& point);
    const StoredPoint& get_point(size_t index) const;
    size_t size() const { return buffer_.size(); }
    bool is_initialized() const { return initialized_; }

private:
    PointBuffer() = default;
    PointBuffer(const PointBuffer&) = delete;
    PointBuffer& operator=(const PointBuffer&) = delete;
    
    std::vector<StoredPoint> buffer_;
    size_t head_{0};
    bool initialized_{false};
};

} // namespace radar
} // namespace vrl
