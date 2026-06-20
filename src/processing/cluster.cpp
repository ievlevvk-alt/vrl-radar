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

void TargetCluster::add_scan(const ScanReplies& scan, uint32_t revolution) {
    if (scans.empty()) {
        start_azimuth = scan.azimuth;
        first_timestamp = scan.timestamp_ms;
        min_range = 65535;
        max_range = 0;
        created_at_revolution = revolution;
        last_update_revolution = revolution;
        revolutions_since_update = 0;
    }
    
    scans.push_back(scan);
    last_processed_azimuth = scan.azimuth;
    
    if (scan.has_replies()) {
        last_reply_azimuth = scan.azimuth;
        last_timestamp = scan.timestamp_ms;
        last_update_revolution = revolution;
        revolutions_since_update = 0;
        marked_for_cleanup = false;
    } else {
        revolutions_since_update = revolution - last_update_revolution;
    }
    
    // Ограничение размера кластера
    const size_t MAX_SCANS_IN_CLUSTER = 500;
    if (scans.size() > MAX_SCANS_IN_CLUSTER) {
        size_t to_remove = scans.size() - MAX_SCANS_IN_CLUSTER;
        scans.erase(scans.begin(), scans.begin() + to_remove);
        if (!scans.empty()) {
            start_azimuth = scans.front().azimuth;
        }
        VRL_LOG_TRACE(modules::CLUSTER, "Cluster size limited, removed " + 
                      std::to_string(to_remove) + " old scans");
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

void TargetCluster::update_revolution(uint32_t revolution) {
    if (revolution > last_update_revolution) {
        revolutions_since_update = revolution - last_update_revolution;
    }
}

bool TargetCluster::is_active(uint16_t current_azimuth, int max_gap_azimuth) const {
    if (scans.empty()) return false;
    
    int16_t gap = current_azimuth - last_reply_azimuth;
    if (gap < 0) gap += 4096;
    return gap <= max_gap_azimuth;
}

bool TargetCluster::is_expired(uint32_t current_revolution) const {
    if (scans.empty()) return true;
    uint32_t age = current_revolution - last_update_revolution;
    return age > max_revolutions_no_update;
}

bool TargetCluster::should_be_cleaned(uint32_t current_revolution) const {
    if (is_expired(current_revolution)) {
        return true;
    }
    if (revolutions_since_update > max_revolutions_no_update / 2) {
        return true;
    }
    return false;
}

// Move-версии
std::vector<RBSReply> TargetCluster::get_all_rbs() && {
    std::vector<RBSReply> result;
    size_t total = 0;
    for (const auto& scan : scans) {
        total += scan.rbs_replies.size();
    }
    result.reserve(total);
    for (auto& scan : scans) {
        for (auto& reply : scan.rbs_replies) {
            result.push_back(std::move(reply));
        }
    }
    return result;
}

std::vector<RBSReply> TargetCluster::get_all_rbs() const& {
    std::vector<RBSReply> result;
    size_t total = 0;
    for (const auto& scan : scans) {
        total += scan.rbs_replies.size();
    }
    result.reserve(total);
    for (const auto& scan : scans) {
        result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
    }
    return result;
}

std::vector<UVDReply> TargetCluster::get_all_uvd() && {
    std::vector<UVDReply> result;
    size_t total = 0;
    for (const auto& scan : scans) {
        total += scan.uvd_replies.size();
    }
    result.reserve(total);
    for (auto& scan : scans) {
        for (auto& reply : scan.uvd_replies) {
            result.push_back(std::move(reply));
        }
    }
    return result;
}

std::vector<UVDReply> TargetCluster::get_all_uvd() const& {
    std::vector<UVDReply> result;
    size_t total = 0;
    for (const auto& scan : scans) {
        total += scan.uvd_replies.size();
    }
    result.reserve(total);
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

// ============================================================================
// НОВЫЙ КОНСТРУКТОР С КОНФИГУРАЦИЕЙ
// ============================================================================

ClusterTracker::ClusterTracker(const ClustererConfig& config)
    : config_(config) {
    clusterer_ = create_clusterer(config);
    clusterer_type_ = (config.type == ClustererConfig::Type::DBSCAN) ? 
                       ClustererType::DBSCAN : ClustererType::LEGACY;
    VRL_LOG_INFO(modules::CLUSTER, "ClusterTracker initialized with config: " +
                 get_clusterer_type_name() + 
                 ", max_revolutions=" + std::to_string(config.max_revolutions_no_update) +
                 ", max_active=" + std::to_string(config.max_active_clusters));
}

// ============================================================================
// СУЩЕСТВУЮЩИЙ КОНСТРУКТОР (обновлен)
// ============================================================================

ClusterTracker::ClusterTracker(int max_gap_azimuth, int range_window) {
    config_.type = ClustererConfig::Type::LEGACY;
    config_.max_gap_azimuth = max_gap_azimuth;
    config_.range_window = range_window;
    config_.max_revolutions_no_update = 5;
    config_.max_active_clusters = 100;
    
    clusterer_ = create_clusterer(config_);
    clusterer_type_ = ClustererType::LEGACY;
    
    VRL_LOG_INFO(modules::CLUSTER, "ClusterTracker initialized with " + 
                 get_clusterer_type_name() + ": gap=" + std::to_string(max_gap_azimuth) + 
                 ", window=" + std::to_string(range_window));
}

// ============================================================================
// КОНСТРУКТОР С КАСТОМНЫМ КЛАСТЕРИЗАТОРОМ
// ============================================================================

ClusterTracker::ClusterTracker(std::unique_ptr<IClusterer> clusterer)
    : clusterer_(std::move(clusterer)) {
    config_.type = ClustererConfig::Type::LEGACY;
    config_.max_revolutions_no_update = 5;
    config_.max_active_clusters = 100;
    clusterer_type_ = ClustererType::LEGACY;
    VRL_LOG_INFO(modules::CLUSTER, "ClusterTracker initialized with custom clusterer: " + 
                 (clusterer_ ? clusterer_->get_name() : "null"));
}

// ============================================================================
// СОЗДАНИЕ КЛАСТЕРИЗАТОРА ИЗ КОНФИГУРАЦИИ
// ============================================================================

std::unique_ptr<IClusterer> ClusterTracker::create_clusterer(const ClustererConfig& config) {
    switch (config.type) {
        case ClustererConfig::Type::DBSCAN: {
            // ИСПРАВЛЕНО: убрали min_points из сообщения
            std::string msg = "Creating DBSCANClusterer with config: range_gap=" + 
                              std::to_string(config.max_range_gap) +
                              ", azimuth_coeff=" + std::to_string(config.azimuth_gap_coefficient);
            VRL_LOG_DEBUG(modules::CLUSTER, msg);
            
            RadarConfig radar_config;
            radar_config.beamwidth_deg = 5.0;
            
            // ИСПРАВЛЕНО: убрали min_points (третий аргумент)
            auto clusterer = std::make_unique<DBSCANClusterer>(
                radar_config,
                config.max_range_gap,
                config.azimuth_gap_coefficient
            );
            
            return clusterer;
        }
        
        case ClustererConfig::Type::LEGACY:
        default: {
            std::string msg = "Creating LegacyClusterer: gap=" + 
                              std::to_string(config.max_gap_azimuth) +
                              ", window=" + std::to_string(config.range_window);
            VRL_LOG_DEBUG(modules::CLUSTER, msg);
            
            return std::make_unique<LegacyClusterer>(
                config.max_gap_azimuth,
                config.range_window
            );
        }
    }
}


// ============================================================================
// ОБНОВЛЕНИЕ КОНФИГУРАЦИИ
// ============================================================================

void ClusterTracker::update_config(const ClustererConfig& config) {
    config_ = config;
    clusterer_ = create_clusterer(config);
    clusterer_type_ = (config.type == ClustererConfig::Type::DBSCAN) ? 
                       ClustererType::DBSCAN : ClustererType::LEGACY;
    VRL_LOG_INFO(modules::CLUSTER, "Clusterer config updated to: " + get_clusterer_type_name());
}

// ============================================================================
// ВЫБОР АЛГОРИТМА (для обратной совместимости)
// ============================================================================

void ClusterTracker::set_clusterer_type(ClustererType type) {
    clusterer_type_ = type;
    
    switch (type) {
        case ClustererType::DBSCAN:
            config_.type = ClustererConfig::Type::DBSCAN;
            break;
        case ClustererType::LEGACY:
        default:
            config_.type = ClustererConfig::Type::LEGACY;
            break;
    }
    
    clusterer_ = create_clusterer(config_);
    VRL_LOG_INFO(modules::CLUSTER, "Clusterer changed to: " + get_clusterer_type_name());
}

std::string ClusterTracker::get_clusterer_type_name() const {
    switch (clusterer_type_) {
        case ClustererType::DBSCAN: return "DBSCANClusterer";
        case ClustererType::LEGACY: 
        default: return "LegacyClusterer";
    }
}

// ============================================================================
// УСТАНОВКА ПАРАМЕТРОВ (для обратной совместимости)
// ============================================================================

void ClusterTracker::set_max_gap_azimuth(int gap) {
    if (clusterer_) {
        clusterer_->set_param("max_gap_azimuth", gap);
    }
}

void ClusterTracker::set_range_window(int window) {
    if (clusterer_) {
        clusterer_->set_param("range_window", window);
    }
}

void ClusterTracker::set_max_revolutions_no_update(uint32_t max) {
    config_.max_revolutions_no_update = max;
    if (clusterer_) {
        clusterer_->set_param("max_revolutions_no_update", static_cast<int>(max));
    }
}

// ============================================================================
// ОСНОВНЫЕ МЕТОДЫ
// ============================================================================

void ClusterTracker::process_scan(const ScanReplies& scan) {
    if (!clusterer_) {
        VRL_LOG_WARN(modules::CLUSTER, "No clusterer set, scan ignored");
        return;
    }
    
    current_revolution_++;
    clusterer_->process_scan(scan);
    
    // Проверка на превышение лимита активных кластеров
    const auto& active = clusterer_->get_active_clusters();
    if (active.size() > config_.max_active_clusters) {
        VRL_LOG_WARN(modules::CLUSTER, "Too many active clusters (" + 
                     std::to_string(active.size()) + " > " + 
                     std::to_string(config_.max_active_clusters) + "), forcing cleanup");
        cleanup_stale_clusters(current_revolution_);
    }
}

std::vector<TargetCluster> ClusterTracker::get_completed_clusters() {
    if (clusterer_) {
        auto clusters = clusterer_->get_completed_clusters();
        const size_t MAX_RETURN = 100;
        if (clusters.size() > MAX_RETURN) {
            VRL_LOG_WARN(modules::CLUSTER, "Too many completed clusters (" + 
                         std::to_string(clusters.size()) + "), limiting to " + 
                         std::to_string(MAX_RETURN));
            clusters.resize(MAX_RETURN);
        }
        return clusters;
    }
    return {};
}

const std::vector<TargetCluster>& ClusterTracker::get_active_clusters() const {
    static const std::vector<TargetCluster> empty;
    if (clusterer_) {
        return clusterer_->get_active_clusters();
    }
    return empty;
}

void ClusterTracker::reset() {
    if (clusterer_) {
        clusterer_->reset();
    }
    current_revolution_ = 0;
    total_clusters_cleaned_ = 0;
}

void ClusterTracker::set_clusterer(std::unique_ptr<IClusterer> clusterer) {
    clusterer_ = std::move(clusterer);
    VRL_LOG_INFO(modules::CLUSTER, "Clusterer changed to: " + 
                 (clusterer_ ? clusterer_->get_name() : "null"));
}

std::string ClusterTracker::get_algorithm_name() const {
    if (clusterer_) {
        return clusterer_->get_name();
    }
    return "none";
}

size_t ClusterTracker::cleanup_stale_clusters(uint32_t current_revolution) {
    if (!clusterer_) return 0;
    
    size_t cleaned = 0;
    auto completed = clusterer_->get_completed_clusters();
    cached_completed_count_ += completed.size();
    
    auto& active = const_cast<std::vector<TargetCluster>&>(clusterer_->get_active_clusters());
    
    auto it = active.begin();
    while (it != active.end()) {
        it->update_revolution(current_revolution);
        if (it->should_be_cleaned(current_revolution)) {
            VRL_LOG_TRACE(modules::CLUSTER, "Cleaning stale cluster: age=" + 
                          std::to_string(current_revolution - it->last_update_revolution) +
                          " revs, scans=" + std::to_string(it->scans.size()));
            it = active.erase(it);
            cleaned++;
            total_clusters_cleaned_++;
        } else {
            ++it;
        }
    }
    
    if (cleaned > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Cleaned " + std::to_string(cleaned) + " stale clusters");
    }
    return cleaned;
}

ClusterTracker::ClusterStats ClusterTracker::get_stats() const {
    ClusterStats stats{};
    if (clusterer_) {
        size_t active = 0;
        size_t completed = 0;
        clusterer_->get_stats(active, completed);
        stats.active_count = active;
        stats.completed_count = completed + cached_completed_count_;
        stats.cleaned_count = total_clusters_cleaned_;
        stats.total_scans_processed = current_revolution_;
        stats.total_clusters_cleaned = total_clusters_cleaned_;
    }
    return stats;
}

// ============================================================================
// CLUSTER PROCESSOR IMPLEMENTATION
// ============================================================================

ClusterProcessor::ClusterProcessor(const RadarConfig& config)
    : config_(config)
    , range_grouper_(5)
    , rbs_processor_(config)
    , uvd_processor_(config) {
    VRL_LOG_DEBUG(modules::CLUSTER, "ClusterProcessor initialized");
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
    auto range_groups = range_grouper_.group(cluster);
    
    for (const auto& group : range_groups) {
        if (static_cast<int>(group.total_replies()) < min_hits_) {
            VRL_LOG_TRACE(modules::CLUSTER, "Skipping group: insufficient replies (" + 
                          std::to_string(group.total_replies()) + " < " + 
                          std::to_string(min_hits_) + ")");
            continue;
        }
        
        if (group.has_overlap() && split_garbled_ && garbling_solver_) {
            auto split_reports = process_garbled_group(group);
            reports.insert(reports.end(), split_reports.begin(), split_reports.end());
            continue;
        }
        
        if (group.has_rbs()) {
            auto report = rbs_processor_.process_group(group);
            if (report) {
                reports.push_back(*report);
            }
        }
        
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
