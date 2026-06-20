// src/core/point_buffer.cpp
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/utils/logger.h"

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

PointBuffer& PointBuffer::instance() {
    static PointBuffer instance;
    return instance;
}

void PointBuffer::init(size_t size) {
    if (size == 0) {
        VRL_LOG_ERROR(modules::CORE, "PointBuffer size must be > 0");
        return;
    }
    
    buffer_.resize(size);
    head_ = 0;
    initialized_ = true;
    
    VRL_LOG_INFO(modules::CORE, "PointBuffer initialized with size: " + std::to_string(size));
}

size_t PointBuffer::add_point(const StoredPoint& point) {
    if (!initialized_) {
        VRL_LOG_ERROR(modules::CORE, "PointBuffer not initialized");
        return 0;
    }
    
    size_t index = head_;
    buffer_[head_] = point;
    head_ = (head_ + 1) % buffer_.size();
    return index;
}

const StoredPoint& PointBuffer::get_point(size_t index) const {
    if (!initialized_) {
        static StoredPoint empty;
        VRL_LOG_ERROR(modules::CORE, "PointBuffer not initialized");
        return empty;
    }
    
    if (index >= buffer_.size()) {
        static StoredPoint empty;
        VRL_LOG_ERROR(modules::CORE, "PointBuffer index out of range: " + std::to_string(index));
        return empty;
    }
    
    return buffer_[index];
}

} // namespace radar
} // namespace vrl
