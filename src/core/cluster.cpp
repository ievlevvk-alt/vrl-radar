// src/core/cluster.cpp
#include "vrl/radar/core/cluster.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/utils/logger.h"
#include "vrl/radar/core/cluster_pool.hpp"  // <-- ДОБАВИТЬ

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

void Cluster::add_point(size_t point_index) {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] add_point: point_index=" << point_index 
                  << ", closed=" << closed_ << std::endl;
    }
#endif
    
    if (closed_) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[Cluster id=" << id_ << "] add_point: WARNING - cluster is closed!" << std::endl;
        }
#endif
        VRL_LOG_WARN(modules::CORE, "Attempt to add point to closed cluster");
        return;
    }
    
    bool was_empty = indices_.empty();
    
    const StoredPoint& point = PointBuffer::instance().get_point(point_index);
    
    indices_.push_back(point_index);
    last_azimuth_ = point.azimuth;
    
    if (point.range < min_range_) min_range_ = point.range;
    if (point.range > max_range_) max_range_ = point.range;
    
    if (point.is_rbs) has_rbs_ = true;
    else has_uvd_ = true;
    
    recalculate_statistics();
    
    // Если кластер был пустым - добавляем в активные
    if (was_empty) {
        ClusterPool::instance().add_to_active(id_);
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[Cluster id=" << id_ << "] add_point: added to active list (first point)" << std::endl;
        }
#endif
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] add_point: done, size=" << indices_.size() 
                  << ", azimuth_span=" << azimuth_span_ << std::endl;
    }
#endif
}

void Cluster::remove_points(const std::vector<size_t>& positions) {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] remove_points: count=" << positions.size() 
                  << ", current_size=" << indices_.size() << std::endl;
    }
#endif
    
    if (positions.empty()) return;
    
    std::vector<size_t> sorted = positions;
    std::sort(sorted.begin(), sorted.end(), std::greater<size_t>());
    
    for (size_t pos : sorted) {
        if (pos < indices_.size()) {
            indices_.erase(indices_.begin() + pos);
        }
    }
    
    recalculate_statistics();
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] remove_points: done, new_size=" << indices_.size() << std::endl;
    }
#endif
}

void Cluster::clear() {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] clear: size=" << indices_.size() << std::endl;
    }
#endif
    
    indices_.clear();
    recalculate_statistics();
    closed_ = false;
}

void Cluster::close() {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] close: size=" << indices_.size() 
                  << ", azimuth_span=" << azimuth_span_ << std::endl;
    }
#endif
    
    closed_ = true;
}

void Cluster::recalculate_statistics() {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] recalculate_statistics: size=" << indices_.size() << std::endl;
    }
#endif
    
    min_range_ = 65535;
    max_range_ = 0;
    has_rbs_ = false;
    has_uvd_ = false;
    
    if (indices_.empty()) {
        azimuth_span_ = 0;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[Cluster id=" << id_ << "] recalculate_statistics: empty, azimuth_span=0" << std::endl;
        }
#endif
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
        if (max_az - min_az > 2048) {
            azimuth_span_ = (4096 - max_az) + min_az;
        } else {
            azimuth_span_ = max_az - min_az;
        }
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[Cluster id=" << id_ << "] recalculate_statistics: min_range=" << min_range_ 
                  << ", max_range=" << max_range_ << ", azimuth_span=" << azimuth_span_ 
                  << ", min_az=" << min_az << ", max_az=" << max_az << std::endl;
    }
#endif
}

} // namespace radar
} // namespace vrl
