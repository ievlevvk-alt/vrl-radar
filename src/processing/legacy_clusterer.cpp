// src/processing/legacy_clusterer.cpp
#include "vrl/radar/processing/legacy_clusterer.h"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>  // <-- ДОБАВЛЯЕМ для отладки

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

LegacyClusterer::LegacyClusterer(int max_gap_azimuth, int range_window)
    : max_gap_azimuth_(max_gap_azimuth), range_window_(range_window) {
    VRL_LOG_DEBUG(modules::CLUSTER, "LegacyClusterer initialized: gap=" + 
                  std::to_string(max_gap_azimuth) + ", window=" + std::to_string(range_window));
}

void LegacyClusterer::process_scan(const ScanReplies& scan) {
    total_scans_processed_++;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing scan at azimuth " + std::to_string(scan.azimuth) +
                  " with " + std::to_string(scan.reply_count()) + " replies");
    
    uint32_t revolution = static_cast<uint32_t>(total_scans_processed_);
    
    update_existing_clusters(scan, revolution);
    
    if (scan.has_replies()) {
        try_create_new_clusters(scan, revolution);
    }
    
    complete_expired_clusters(scan.azimuth);
}


void LegacyClusterer::update_existing_clusters(const ScanReplies& scan, uint32_t revolution) {
    int updated = 0;
    
    for (auto& cluster : active_clusters_) {
        bool active = cluster.is_active(scan.azimuth, max_gap_azimuth_);
        
        if (!active) {
            continue;
        }
        
        if (scan.has_replies()) {
            bool range_match = false;
            
            for (const auto& reply : scan.rbs_replies) {
                if (reply.range >= cluster.min_range - range_window_ &&
                    reply.range <= cluster.max_range + range_window_) {
                    range_match = true;
                    break;
                }
            }
            
            if (!range_match) {
                for (const auto& reply : scan.uvd_replies) {
                    if (reply.range >= cluster.min_range - range_window_ &&
                        reply.range <= cluster.max_range + range_window_) {
                        range_match = true;
                        break;
                    }
                }
            }
            
            if (range_match) {
                cluster.add_scan(scan, revolution);
                updated++;
            }
        } else {
            // ПУСТЫЕ СКАНЫ НЕ ОБНОВЛЯЮТ last_reply_azimuth!
            // Только обновляем last_processed_azimuth для отслеживания
            cluster.last_processed_azimuth = scan.azimuth;
            // НЕ обновляем last_reply_azimuth!
        }
    }
    
    if (updated > 0) {
        VRL_LOG_TRACE(modules::CLUSTER, "Updated " + std::to_string(updated) + " clusters");
    }
}

void LegacyClusterer::complete_expired_clusters(uint16_t current_azimuth) {
    std::cout << "=== complete_expired_clusters: current_azimuth=" << current_azimuth 
              << ", active_clusters=" << active_clusters_.size() 
              << ", max_gap=" << max_gap_azimuth_ << std::endl;
    
    auto it = active_clusters_.begin();
    int completed = 0;
    
    while (it != active_clusters_.end()) {
        bool active = it->is_active(current_azimuth, max_gap_azimuth_);
        
        int16_t gap = current_azimuth - it->last_reply_azimuth;
        if (gap < 0) gap += 4096;
        
        std::cout << "  Cluster: last_reply_az=" << it->last_reply_azimuth 
                  << ", gap=" << gap
                  << ", active=" << active << std::endl;
        
        if (!active) {
            std::cout << "    COMPLETING cluster!" << std::endl;
            completed_clusters_.push_back(std::move(*it));
            it = active_clusters_.erase(it);
            completed++;
            total_clusters_completed_++;
        } else {
            std::cout << "    Keeping cluster" << std::endl;
            ++it;
        }
    }
    
    std::cout << "=== complete_expired_clusters: completed=" << completed 
              << ", remaining=" << active_clusters_.size() << std::endl;
    
    if (completed > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Completed " + std::to_string(completed) + " clusters");
    }
}



void LegacyClusterer::try_create_new_clusters(const ScanReplies& scan, uint32_t revolution) {
    for (auto& cluster : active_clusters_) {
        if (!cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
            continue;
        }
        
        for (const auto& reply : scan.rbs_replies) {
            if (reply.range >= cluster.min_range - range_window_ &&
                reply.range <= cluster.max_range + range_window_) {
                return;
            }
        }
        
        for (const auto& reply : scan.uvd_replies) {
            if (reply.range >= cluster.min_range - range_window_ &&
                reply.range <= cluster.max_range + range_window_) {
                return;
            }
        }
    }
    
    TargetCluster new_cluster;
    new_cluster.add_scan(scan, revolution);
    active_clusters_.push_back(std::move(new_cluster));
    total_clusters_formed_++;
    
    VRL_LOG_DEBUG(modules::CLUSTER, "Created new cluster at azimuth " + std::to_string(scan.azimuth) +
                  " with " + std::to_string(scan.reply_count()) + " replies");
}


std::vector<TargetCluster> LegacyClusterer::get_completed_clusters() {
    auto result = std::move(completed_clusters_);
    completed_clusters_.clear();
    return result;
}

const std::vector<TargetCluster>& LegacyClusterer::get_active_clusters() const {
    return active_clusters_;
}

std::vector<TargetCluster> LegacyClusterer::finish_all() {
    VRL_LOG_DEBUG(modules::CLUSTER, "Finishing all clusters (" + 
                  std::to_string(active_clusters_.size()) + " active)");
    
    std::vector<TargetCluster> result;
    
    for (auto& cluster : active_clusters_) {
        result.push_back(std::move(cluster));
        total_clusters_completed_++;
    }
    active_clusters_.clear();
    
    for (auto& cluster : completed_clusters_) {
        result.push_back(std::move(cluster));
    }
    completed_clusters_.clear();
    
    return result;
}

void LegacyClusterer::reset() {
    size_t old_active = active_clusters_.size();
    size_t old_completed = completed_clusters_.size();
    active_clusters_.clear();
    completed_clusters_.clear();
    total_scans_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    
    VRL_LOG_DEBUG(modules::CLUSTER, "Reset: cleared " + std::to_string(old_active) + 
                  " active and " + std::to_string(old_completed) + " completed clusters");
}

void LegacyClusterer::get_stats(size_t& active, size_t& completed) const {
    active = active_clusters_.size();
    completed = completed_clusters_.size();
}

std::unique_ptr<IClusterer> LegacyClusterer::clone() const {
    auto clone = std::make_unique<LegacyClusterer>(max_gap_azimuth_, range_window_);
    return clone;
}

// ИСПРАВЛЕНО: добавлена реализация для double
void LegacyClusterer::set_param(const std::string& key, double value) {
    std::cout << "DEBUG: set_param(double) key=" << key << ", value=" << value << std::endl;
    if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = static_cast<int>(value);
        std::cout << "DEBUG: max_gap_azimuth set to " << max_gap_azimuth_ << std::endl;
    } else if (key == "range_window") {
        range_window_ = static_cast<int>(value);
        std::cout << "DEBUG: range_window set to " << range_window_ << std::endl;
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown double parameter: " + key);
    }
}

void LegacyClusterer::set_param(const std::string& key, int value) {
    std::cout << "DEBUG: set_param(int) key=" << key << ", value=" << value << std::endl;
    if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = value;
        std::cout << "DEBUG: max_gap_azimuth set to " << max_gap_azimuth_ << std::endl;
    } else if (key == "range_window") {
        range_window_ = value;
        std::cout << "DEBUG: range_window set to " << range_window_ << std::endl;
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

} // namespace radar
} // namespace vrl
