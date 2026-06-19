// src/processing/cluster.cpp
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/utils/utils.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ============================================================================
// TARGET CLUSTER IMPLEMENTATION
// ============================================================================

void TargetCluster::add_scan(const ScanReplies& scan) {
    if (scans.empty()) {
        start_azimuth = scan.azimuth;
        first_timestamp = scan.timestamp_ms;
        min_range = 65535;
        max_range = 0;
    }
    
    scans.push_back(scan);
    last_processed_azimuth = scan.azimuth;
    
    if (scan.has_replies()) {
        last_reply_azimuth = scan.azimuth;
        last_timestamp = scan.timestamp_ms;
    }
    
    for (const auto& reply : scan.rbs_replies) {
        rbs_by_azimuth[scan.azimuth].push_back(reply);
        min_range = std::min(min_range, reply.range);
        max_range = std::max(max_range, reply.range);
    }
    
    for (const auto& reply : scan.uvd_replies) {
        uvd_by_azimuth[scan.azimuth].push_back(reply);
        min_range = std::min(min_range, reply.range);
        max_range = std::max(max_range, reply.range);
    }
}

bool TargetCluster::is_active(uint16_t current_azimuth, int max_gap_azimuth) const {
    if (scans.empty()) return false;
    
    int16_t gap = current_azimuth - last_reply_azimuth;
    if (gap < 0) gap += 4096;
    return gap <= max_gap_azimuth;
}

std::vector<RBSReply> TargetCluster::get_all_rbs() const {
    std::vector<RBSReply> result;
    for (const auto& scan : scans) {
        result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
    }
    return result;
}

std::vector<UVDReply> TargetCluster::get_all_uvd() const {
    std::vector<UVDReply> result;
    for (const auto& scan : scans) {
        result.insert(result.end(), scan.uvd_replies.begin(), scan.uvd_replies.end());
    }
    return result;
}

uint16_t TargetCluster::azimuth_span() const {
    if (scans.empty()) return 0;
    
    int16_t span = last_reply_azimuth - start_azimuth;
    if (span < 0) span += 4096;
    return static_cast<uint16_t>(span);
}

uint16_t TargetCluster::range_span() const {
    return max_range - min_range;
}

size_t TargetCluster::reply_scans_count() const {
    size_t count = 0;
    for (const auto& scan : scans) {
        if (scan.has_replies()) count++;
    }
    return count;
}

// ============================================================================
// CLUSTER TRACKER IMPLEMENTATION
// ============================================================================

ClusterTracker::ClusterTracker(int max_gap_azimuth, int range_window)
    : max_gap_azimuth_(max_gap_azimuth), range_window_(range_window) {
    VRL_LOG_DEBUG(modules::CLUSTER, "ClusterTracker initialized: gap=" + 
                  std::to_string(max_gap_azimuth) + ", window=" + std::to_string(range_window));
}

void ClusterTracker::process_scan(const ScanReplies& scan) {
    VRL_LOG_TRACE(modules::CLUSTER, "Processing scan at azimuth " + std::to_string(scan.azimuth) +
                  " with " + std::to_string(scan.reply_count()) + " replies");
    
    update_existing_clusters(scan);
    
    if (scan.has_replies()) {
        try_create_new_clusters(scan);
    }
    
    complete_expired_clusters(scan.azimuth);
}

void ClusterTracker::update_existing_clusters(const ScanReplies& scan) {
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

void ClusterTracker::try_create_new_clusters(const ScanReplies& scan) {
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
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created new cluster at azimuth " + std::to_string(scan.azimuth) +
                  " with " + std::to_string(scan.reply_count()) + " replies");
}

void ClusterTracker::complete_expired_clusters(uint16_t current_azimuth) {
    auto it = active_clusters_.begin();
    int completed = 0;
    
    while (it != active_clusters_.end()) {
        if (!it->is_active(current_azimuth, max_gap_azimuth_)) {
            VRL_LOG_DEBUG(modules::CLUSTER, "Cluster completed: azimuth_span=" + 
                          std::to_string(it->azimuth_span()) + 
                          ", range_span=" + std::to_string(it->range_span()) +
                          ", replies=" + std::to_string(it->get_all_rbs().size() + 
                                                        it->get_all_uvd().size()));
            completed_clusters_.push_back(std::move(*it));
            it = active_clusters_.erase(it);
            completed++;
        } else {
            ++it;
        }
    }
    
    if (completed > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Completed " + std::to_string(completed) + " clusters");
    }
}

std::vector<TargetCluster> ClusterTracker::get_completed_clusters() {
    auto result = std::move(completed_clusters_);
    completed_clusters_.clear();
    return result;
}

const std::vector<TargetCluster>& ClusterTracker::get_active_clusters() const {
    return active_clusters_;
}

void ClusterTracker::reset() {
    size_t old_active = active_clusters_.size();
    size_t old_completed = completed_clusters_.size();
    active_clusters_.clear();
    completed_clusters_.clear();
    VRL_LOG_DEBUG(modules::CLUSTER, "Reset: cleared " + std::to_string(old_active) + 
                  " active and " + std::to_string(old_completed) + " completed clusters");
}

// ============================================================================
// CLUSTER PROCESSOR IMPLEMENTATION
// ============================================================================

ClusterProcessor::ClusterProcessor(const RadarConfig& config)
    : config_(config)
    , range_grouper_(5)
    , rbs_processor_(config)
    , uvd_processor_(config) {
    VRL_LOG_DEBUG(modules::CLUSTER, "ClusterProcessor initialized: range_bin_rbs=" + 
                  std::to_string(config.range_bin_rbs) + ", range_bin_uvd=" + 
                  std::to_string(config.range_bin_uvd));
}

void ClusterProcessor::set_garbling_solver(std::unique_ptr<GarblingSolver> solver) {
    garbling_solver_ = std::move(solver);
}

std::vector<TargetReport> ClusterProcessor::process_garbled_group(const RangeGrouper::RangeGroup& group) {
    std::vector<TargetReport> reports;
    
    if (!garbling_solver_ || group.rbs_replies.empty()) {
        return reports;
    }
    
    VRL_LOG_DEBUG(modules::CLUSTER, "Processing garbled group with " + 
                  std::to_string(group.rbs_replies.size()) + " RBS replies");
    
    std::vector<RBSReply> all_rbs;
    for (const auto* ptr : group.rbs_replies) {
        all_rbs.push_back(*ptr);
    }
    
    auto result = garbling_solver_->separate_rbs(all_rbs);
    
    double confidence_threshold = 0.5;
    
    if (result.confidence > confidence_threshold && !result.separated_replies.empty()) {
        VRL_LOG_INFO(modules::CLUSTER, "Split " + std::to_string(all_rbs.size()) + 
                     " replies into " + std::to_string(result.separated_replies.size()) + 
                     " targets (conf=" + std::to_string(result.confidence) + ")");
        
        for (const auto& separated : result.separated_replies) {
            TargetReport report = TargetReport::make_rbs();
            report.type = TargetReport::SourceType::RBS;
            double az_per_bin = 360.0 / 4096.0;
            report.azimuth_deg = separated.azimuth * az_per_bin;
            report.range_m = separated.range * config_.range_bin_rbs;
            report.rbs.mode3a_code = separated.code12;
            report.rbs.spi = separated.spi;
            report.is_garbled = false;
            report.is_sls_blanked = false;
            
            polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
            reports.push_back(report);
        }
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Failed to split garbled group (conf=" + 
                     std::to_string(result.confidence) + ")");
    }
    
    return reports;
}

std::vector<TargetReport> ClusterProcessor::process_cluster(const TargetCluster& cluster) {
    VRL_LOG_DEBUG(modules::CLUSTER, "Processing cluster with " + 
                  std::to_string(cluster.scans.size()) + " scans");
    
    std::vector<TargetReport> reports;
    
    // Группируем по дальности
    auto range_groups = range_grouper_.group(cluster);
    
    for (const auto& group : range_groups) {
        // Проверяем минимальное количество ответов
        if (static_cast<int>(group.total_replies()) < min_hits_) {
            VRL_LOG_TRACE(modules::CLUSTER, "Skipping group: insufficient replies (" + 
                          std::to_string(group.total_replies()) + " < " + 
                          std::to_string(min_hits_) + ")");
            continue;
        }
        
        // Обработка перекрытий (garbling)
        if (group.has_overlap() && split_garbled_ && garbling_solver_) {
            auto split_reports = process_garbled_group(group);
            reports.insert(reports.end(), split_reports.begin(), split_reports.end());
            continue;
        }
        
        // Обработка RBS
        if (group.has_rbs()) {
            auto report = rbs_processor_.process_group(group);
            if (report) {
                reports.push_back(*report);
            }
        }
        
        // Обработка UVD
        if (group.has_uvd()) {
            auto report = uvd_processor_.process_group(group);
            if (report) {
                reports.push_back(*report);
            }
        }
    }
    
    if (!reports.empty()) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Generated " + std::to_string(reports.size()) + 
                      " reports from cluster");
    }
    
    return reports;
}

} // namespace radar
} // namespace vrl
