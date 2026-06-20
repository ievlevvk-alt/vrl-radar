// src/core/cluster.cpp
#include "vrl/radar/core/cluster.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/utils/logger.h"

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

void Cluster::add_point(size_t point_index) {
    if (closed_) {
        VRL_LOG_WARN(modules::CORE, "Attempt to add point to closed cluster");
        return;
    }
    
    const StoredPoint& point = PointBuffer::instance().get_point(point_index);
    
    indices_.push_back(point_index);
    last_azimuth_ = point.azimuth;
    
    if (point.range < min_range_) min_range_ = point.range;
    if (point.range > max_range_) max_range_ = point.range;
    
    if (point.is_rbs) has_rbs_ = true;
    else has_uvd_ = true;
    
    recalculate_statistics();
}

void Cluster::remove_points(const std::vector<size_t>& positions) {
    if (positions.empty()) return;
    
    std::vector<size_t> sorted = positions;
    std::sort(sorted.begin(), sorted.end(), std::greater<size_t>());
    
    for (size_t pos : sorted) {
        if (pos < indices_.size()) {
            indices_.erase(indices_.begin() + pos);
        }
    }
    
    recalculate_statistics();
}

void Cluster::clear() {
    indices_.clear();
    recalculate_statistics();
    closed_ = false;
}

void Cluster::recalculate_statistics() {
    // Сброс
    min_range_ = 65535;
    max_range_ = 0;
    has_rbs_ = false;
    has_uvd_ = false;
    
    if (indices_.empty()) {
        azimuth_span_ = 0;
        return;
    }
    
    uint16_t min_az = 4096;
    uint16_t max_az = 0;
    
    for (size_t idx : indices_) {
        const StoredPoint& point = PointBuffer::instance().get_point(idx);
        
        if (point.range < min_range_) min_range_ = point.range;
        if (point.range > max_range_) max_range_ = point.range;
        
        if (point.azimuth < min_az) min_az = point.azimuth;
        if (point.azimuth > max_az) max_az = point.azimuth;
        
        if (point.is_rbs) has_rbs_ = true;
        else has_uvd_ = true;
    }
    
    if (indices_.size() == 1) {
        azimuth_span_ = 0;
    } else {
        // Если разброс больше половины оборота (2048), значит кластер пересекает Север
        if (max_az - min_az > 2048) {
            // Правильная формула для перехода через Север:
            // (4096 - max_az) + min_az
            azimuth_span_ = (4096 - max_az) + min_az;
        } else {
            azimuth_span_ = max_az - min_az;
        }
    }
}


} // namespace radar
} // namespace vrl
