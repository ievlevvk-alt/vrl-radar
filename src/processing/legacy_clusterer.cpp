// src/processing/legacy_clusterer.cpp
#include "vrl/radar/processing/legacy_clusterer.h"
#include "vrl/radar/processing/cluster.h"  // <-- ДОБАВЛЯЕМ!
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>

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
    
    update_existing_clusters(scan);
    
    if (scan.has_replies()) {
        try_create_new_clusters(scan);
    }
    
    complete_expired_clusters(scan.azimuth);
}

void LegacyClusterer::update_existing_clusters(const ScanReplies& scan) {
    int updated = 0;
    
    for (auto& cluster : active_clusters_) {
        bool range_match = false;
        
        if (scan.has_replies()) {
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
        } else {
            if (cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
                range_match = true;
            }
        }
        
        if (range_match && cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
            cluster.add_scan(scan);
            updated++;
        }
    }
    
    if (updated > 0) {
        VRL_LOG_TRACE(modules::CLUSTER, "Updated " + std::to_string(updated) + " clusters");
    }
}

void LegacyClusterer::try_create_new_clusters(const ScanReplies& scan) {
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
    new_cluster.add_scan(scan);
    active_clusters_.push_back(std::move(new_cluster));
    total_clusters_formed_++;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created new cluster at azimuth " + std::to_string(scan.azimuth) +
                  " with " + std::to_string(scan.reply_count()) + " replies");
}

void LegacyClusterer::complete_expired_clusters(uint16_t current_azimuth) {
    auto it = active_clusters_.begin();
    int completed = 0;
    
    while (it != active_clusters_.end()) {
        if (!it->is_active(current_azimuth, max_gap_azimuth_)) {
            auto rbs = it->get_all_rbs();
            auto uvd = it->get_all_uvd();
            
            VRL_LOG_DEBUG(modules::CLUSTER, "Cluster completed: azimuth_span=" + 
                          std::to_string(it->azimuth_span()) + 
                          ", range_span=" + std::to_string(it->range_span()) +
                          ", replies=" + std::to_string(rbs.size() + uvd.size()));
            completed_clusters_.push_back(std::move(*it));
            it = active_clusters_.erase(it);
            completed++;
            total_clusters_completed_++;
        } else {
            ++it;
        }
    }
    
    if (completed > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Completed " + std::to_string(completed) + " clusters");
    }
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

void LegacyClusterer::set_param(const std::string& key, double value) {
    if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = static_cast<int>(value);
    } else if (key == "range_window") {
        range_window_ = static_cast<int>(value);
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown double parameter: " + key);
    }
}

void LegacyClusterer::set_param(const std::string& key, int value) {
    if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = value;
    } else if (key == "range_window") {
        range_window_ = value;
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

} // namespace radar
} // namespace vrl
