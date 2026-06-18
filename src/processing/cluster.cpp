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
    : config_(config), reply_processor_(config) {
    VRL_LOG_DEBUG(modules::CLUSTER, "ClusterProcessor initialized: range_bin_rbs=" + 
                  std::to_string(config.range_bin_rbs) + ", range_bin_uvd=" + 
                  std::to_string(config.range_bin_uvd));
}

std::vector<ClusterProcessor::RangeGroup> ClusterProcessor::group_by_range(const TargetCluster& cluster) {
    VRL_LOG_TRACE(modules::CLUSTER, "Grouping " + std::to_string(cluster.scans.size()) + " scans by range");
    
    std::vector<RangeGroup> groups;
    std::map<uint16_t, RangeGroup> range_map;
    
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            bool added = false;
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_) {
                    group.add_rbs(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_rbs(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            bool added = false;
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_) {
                    group.add_uvd(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_uvd(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    for (auto& [_, group] : range_map) {
        groups.push_back(std::move(group));
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created " + std::to_string(groups.size()) + " range groups");
    return groups;
}

double ClusterProcessor::average_azimuth(const std::vector<uint16_t>& azimuths) {
    if (azimuths.empty()) return 0.0;
    
    bool has_wraparound = false;
    for (size_t i = 1; i < azimuths.size(); ++i) {
        if (std::abs(static_cast<int16_t>(azimuths[i] - azimuths[i-1])) > 2048) {
            has_wraparound = true;
            break;
        }
    }
    
    if (!has_wraparound) {
        double sum = 0.0;
        for (auto az : azimuths) sum += az;
        return sum / azimuths.size();
    } else {
        double sum_sin = 0.0, sum_cos = 0.0;
        for (auto az : azimuths) {
            double rad = az * RadarConfig::azimuth_per_bin * M_PI / 180.0;
            sum_sin += std::sin(rad);
            sum_cos += std::cos(rad);
        }
        double avg_rad = std::atan2(sum_sin, sum_cos);
        double avg_deg = avg_rad * 180.0 / M_PI;
        if (avg_deg < 0) avg_deg += 360.0;
        return avg_deg / RadarConfig::azimuth_per_bin;
    }
}

bool ClusterProcessor::check_sidelobe(const RBSReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

bool ClusterProcessor::check_sidelobe(const UVDReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

void ClusterProcessor::decode_uvd_info(uint32_t data20, TargetReport& report) {
    report.uvd.raw_data20 = data20;
    
    uint32_t octal_part = data20 & 0x7FFFF;
    for (int i = 4; i >= 0; --i) {
        report.uvd.octal_id[i] = (octal_part >> (i * 3)) & 0x7;
    }
    
    report.uvd.altitude = (data20 >> 3) & 0x7FF;
    report.uvd.fuel = (data20 >> 14) & 0x0F;
    report.uvd.pressure_ref = (data20 >> 18) & 0x01;
}

std::optional<TargetReport> ClusterProcessor::process_rbs_group(const RangeGroup& group) {
    if (group.rbs_replies.empty()) return std::nullopt;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing RBS group: " + std::to_string(group.rbs_replies.size()) + 
                  " replies at range " + std::to_string(group.nominal_range));
    
    TargetReport report = TargetReport::make_rbs();
    report.type = TargetReport::SourceType::RBS;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.rbs_replies) {
        azimuths.push_back(reply->azimuth);
        report.sources.push_back(reply);
    }
    
    report.azimuth_deg = average_azimuth(azimuths) * RadarConfig::azimuth_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_rbs;
    polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    std::map<uint16_t, int> code_counts;
    std::map<uint16_t, const RBSReply*> code_to_reply;
    std::map<uint16_t, bool> code_spi;
    std::map<uint16_t, double> code_confidence;
    
    for (const auto* reply : group.rbs_replies) {
        code_counts[reply->code12]++;
        code_to_reply[reply->code12] = reply;
        if (reply->spi) {
            code_spi[reply->code12] = true;
        }
        
        ReplyFeatures features = reply_processor_.analyze_rbs(*reply);
        code_confidence[reply->code12] = std::max(code_confidence[reply->code12], features.confidence);
    }
    
    uint16_t best_code = 0;
    int max_count = 0;
    double best_confidence = 0;
    
    for (const auto& [code, count] : code_counts) {
        double confidence = code_confidence[code];
        if (confidence > best_confidence || 
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_code = code;
        }
    }
    
    if (max_count > 0 && best_confidence > 0.3) {
        const auto* best_reply = code_to_reply[best_code];
        report.rbs.mode3a_code = best_code;
        report.rbs.spi = code_spi[best_code];
        report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
        report.is_sls_blanked = check_sidelobe(*best_reply);
        report.is_garbled = (best_confidence < 0.5);
        
        VRL_LOG_TRACE(modules::CLUSTER, "RBS target: code=0" + std::to_string(best_code) + 
                      ", conf=" + std::to_string(best_confidence));
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "RBS group rejected: low confidence (" + 
                     std::to_string(best_confidence) + ")");
        return std::nullopt;
    }
    
    return report;
}

std::optional<TargetReport> ClusterProcessor::process_uvd_group(const RangeGroup& group) {
    if (group.uvd_replies.empty()) return std::nullopt;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing UVD group: " + std::to_string(group.uvd_replies.size()) + 
                  " replies at range " + std::to_string(group.nominal_range));
    
    TargetReport report = TargetReport::make_uvd();
    report.type = TargetReport::SourceType::UVD;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.uvd_replies) {
        azimuths.push_back(reply->azimuth);
        report.sources.push_back(reply);
    }
    
    report.azimuth_deg = average_azimuth(azimuths) * RadarConfig::azimuth_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_uvd;
    polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    std::map<uint32_t, int> data_counts;
    std::map<uint32_t, const UVDReply*> data_to_reply;
    std::map<uint32_t, double> data_confidence;
    
    for (const auto* reply : group.uvd_replies) {
        data_counts[reply->data20]++;
        data_to_reply[reply->data20] = reply;
        
        ReplyFeatures features = reply_processor_.analyze_uvd(*reply);
        data_confidence[reply->data20] = std::max(data_confidence[reply->data20], features.confidence);
    }
    
    uint32_t best_data = 0;
    int max_count = 0;
    double best_confidence = 0;
    
    for (const auto& [data, count] : data_counts) {
        double confidence = data_confidence[data];
        if (confidence > best_confidence ||
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_data = data;
        }
    }
    
    if (max_count > 0 && best_confidence > 0.3) {
        const auto* best_reply = data_to_reply[best_data];
        decode_uvd_info(best_data, report);
        report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
        report.is_sls_blanked = check_sidelobe(*best_reply);
        report.is_garbled = (best_confidence < 0.5) || (best_reply->error_mask != 0);
        
        VRL_LOG_TRACE(modules::CLUSTER, "UVD target: data=0x" + std::to_string(best_data) + 
                      ", conf=" + std::to_string(best_confidence));
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "UVD group rejected: low confidence (" + 
                     std::to_string(best_confidence) + ")");
        return std::nullopt;
    }
    
    return report;
}

std::vector<TargetReport> ClusterProcessor::process_garbled_group(const RangeGroup& group) {
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
    
    if (result.confidence > 0.5 && !result.separated_replies.empty()) {
        VRL_LOG_INFO(modules::CLUSTER, "Split " + std::to_string(all_rbs.size()) + 
                     " replies into " + std::to_string(result.separated_replies.size()) + 
                     " targets (conf=" + std::to_string(result.confidence) + ")");
        
        for (const auto& separated : result.separated_replies) {
            TargetReport report = TargetReport::make_rbs();
            report.type = TargetReport::SourceType::RBS;
            report.azimuth_deg = separated.azimuth * RadarConfig::azimuth_per_bin;
            report.range_m = separated.range * config_.range_bin_rbs;
            report.rbs.mode3a_code = separated.code12;
            report.rbs.spi = separated.spi;
            report.is_garbled = false;
            report.is_sls_blanked = check_sidelobe(separated);
            
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
    
    auto range_groups = group_by_range(cluster);
    
    for (const auto& group : range_groups) {
        if (group.total_replies() < static_cast<size_t>(min_hits_)) {
            VRL_LOG_TRACE(modules::CLUSTER, "Skipping group: insufficient replies (" + 
                          std::to_string(group.total_replies()) + " < " + 
                          std::to_string(min_hits_) + ")");
            continue;
        }
        
        bool has_overlaps = false;
        if (!group.rbs_replies.empty() && group.rbs_replies.size() > 1) {
            has_overlaps = true;
        }
        
        if (has_overlaps && split_garbled_ && garbling_solver_) {
            auto split_reports = process_garbled_group(group);
            reports.insert(reports.end(), split_reports.begin(), split_reports.end());
        } else {
            if (!group.rbs_replies.empty()) {
                auto report = process_rbs_group(group);
                if (report) reports.push_back(*report);
            }
            
            if (!group.uvd_replies.empty()) {
                auto report = process_uvd_group(group);
                if (report) reports.push_back(*report);
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
