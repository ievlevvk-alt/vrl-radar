// src/processing/tracker.cpp
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ============================================================================
// КОНСТРУКТОРЫ
// ============================================================================

TrackManager::TrackManager(const TrackerConfig& config) 
    : config_(config)
    , track_pool_(TrackPool::instance()) {
    filter_type_ = FilterType::KALMAN;
    default_filter_ = create_default_filter();
    VRL_LOG_INFO(modules::TRACKER, "TrackManager initialized with filter: " + get_filter_name());
    VRL_LOG_DEBUG(modules::TRACKER, "Config: min_hits=" + std::to_string(config.min_hits_to_confirm) +
                  ", max_coast=" + std::to_string(config.max_coast_count) +
                  ", gate_dist=" + std::to_string(config.max_gate_distance) +
                  ", gate_az=" + std::to_string(config.max_gate_azimuth));
}

TrackManager::TrackManager(const TrackerConfig& config, 
                           std::unique_ptr<ITrackerFilter> filter)
    : config_(config)
    , default_filter_(std::move(filter))
    , track_pool_(TrackPool::instance()) {
    VRL_LOG_INFO(modules::TRACKER, "TrackManager initialized with custom filter: " + 
                 (default_filter_ ? default_filter_->get_name() : "null"));
}

std::unique_ptr<ITrackerFilter> TrackManager::create_default_filter() const {
    return create_filter(filter_type_);
}

std::unique_ptr<ITrackerFilter> TrackManager::create_filter(FilterType type) const {
    switch (type) {
        case FilterType::EXTENDED_KALMAN:
            VRL_LOG_DEBUG(modules::TRACKER, "Creating ExtendedKalmanFilter");
            return std::make_unique<ExtendedKalmanFilter>(
                config_.process_noise, config_.measurement_noise, true);
        case FilterType::UNSCENTED_KALMAN:
            VRL_LOG_DEBUG(modules::TRACKER, "Creating UnscentedKalmanFilter");
            return std::make_unique<UnscentedKalmanFilter>(
                config_.process_noise, config_.measurement_noise);
        case FilterType::KALMAN:
        default:
            VRL_LOG_DEBUG(modules::TRACKER, "Creating RevolutionKalmanFilter");
            return std::make_unique<RevolutionKalmanFilter>(
                config_.process_noise, config_.measurement_noise);
    }
}

void TrackManager::set_filter_type(FilterType type) {
    filter_type_ = type;
    default_filter_ = create_default_filter();
    VRL_LOG_INFO(modules::TRACKER, "Filter type changed to: " + get_filter_name());
}

std::string TrackManager::get_filter_name() const {
    if (default_filter_) {
        return default_filter_->get_name();
    }
    return "none";
}

void TrackManager::set_filter(std::unique_ptr<ITrackerFilter> filter) {
    default_filter_ = std::move(filter);
    VRL_LOG_INFO(modules::TRACKER, "Filter changed to: " + 
                 (default_filter_ ? default_filter_->get_name() : "null"));
}

// ============================================================================
// СТАРЫЕ МЕТОДЫ (ДЛЯ СОВМЕСТИМОСТИ)
// ============================================================================

void TrackManager::process_targets(const std::vector<TargetReport>& targets, uint32_t revolution) {
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(targets.size()) + 
                  " targets at revolution " + std::to_string(revolution));
    
    if (targets.empty()) {
        VRL_LOG_WARN(modules::TRACKER, "No targets to process");
        return;
    }
    
    // В новой архитектуре этот метод может быть упрощён
    // Пока оставляем заглушку
}

std::vector<Track> TrackManager::get_active_tracks() const {
    std::vector<Track> result;
    
    auto track_ptrs = track_pool_.get_all_tracks();
    for (Track* track : track_ptrs) {
        if (track->state == TrackState::ACTIVE || 
            track->state == TrackState::COASTING ||
            track->state == TrackState::NEW) {
            result.push_back(*track);
        }
    }
    
    std::sort(result.begin(), result.end(),
        [](const Track& a, const Track& b) {
            return a.confidence > b.confidence;
        });
    
    return result;
}

std::vector<Track> TrackManager::get_confirmed_tracks() const {
    std::vector<Track> result;
    auto track_ptrs = track_pool_.get_all_tracks();
    for (Track* track : track_ptrs) {
        if (track->is_confirmed()) {
            result.push_back(*track);
        }
    }
    return result;
}

void TrackManager::reset() {
    track_pool_.clear();
    next_id_ = 1;
    VRL_LOG_INFO(modules::TRACKER, "Reset all tracks");
}

std::vector<Track*> TrackManager::get_tracks_in_sector(int sector_index) {
    return track_pool_.get_tracks_in_sector(sector_index);
}

// ============================================================================
// МЕТОДЫ ДЛЯ РАБОТЫ С КЛАСТЕРАМИ
// ============================================================================

void TrackManager::process_clusters_in_sector(int sector_index, 
                                               const std::vector<uint64_t>& cluster_ids) {
    if (cluster_ids.empty()) return;
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(cluster_ids.size()) + 
                  " clusters in sector " + std::to_string(sector_index));
    
    auto& pool = ClusterPool::instance();
    auto tracks = track_pool_.get_tracks_in_sector(sector_index);
    
    for (uint64_t id : cluster_ids) {
        Cluster* cluster_ptr = pool.get_cluster(id);
        if (!cluster_ptr) continue;
        
        const Cluster& cluster = *cluster_ptr;
        
        if (cluster.is_empty()) continue;
        if (!is_valid_cluster(cluster)) continue;
        
        Track* best_track = find_best_track(cluster, tracks);
        
        if (best_track) {
            update_track_with_cluster(*best_track, cluster);
            VRL_LOG_TRACE(modules::TRACKER, "Cluster " + std::to_string(id) + 
                          " associated with track " + std::to_string(best_track->id));
        } else {
            create_track_from_cluster(cluster, sector_index);
            VRL_LOG_TRACE(modules::TRACKER, "New track created from cluster " + 
                          std::to_string(id));
        }
    }
}

void TrackManager::process_wide_clusters(const std::vector<uint64_t>& cluster_ids) {
    if (cluster_ids.empty()) return;
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(cluster_ids.size()) + 
                  " wide clusters");
    
    auto& pool = ClusterPool::instance();
    
    for (uint64_t id : cluster_ids) {
        Cluster* cluster_ptr = pool.get_cluster(id);
        if (!cluster_ptr) continue;
        
        const Cluster& cluster = *cluster_ptr;
        
        if (cluster.is_empty()) continue;
        
        auto reports = split_wide_cluster(cluster);
        
        for (const auto& report : reports) {
            if (!is_valid_report(report)) continue;
            
            auto all_tracks = track_pool_.get_all_tracks();
            Track* best_track = nullptr;
            double best_score = -1.0;
            
            for (Track* track : all_tracks) {
                double distance = std::sqrt(
                    std::pow(report.x - track->x, 2) + 
                    std::pow(report.y - track->y, 2)
                );
                double score = 1.0 / (1.0 + distance / 100.0);
                if (score > best_score) {
                    best_score = score;
                    best_track = track;
                }
            }
            
            if (best_track && best_score > 0.4) {
                best_track->x = report.x;
                best_track->y = report.y;
                best_track->azimuth_deg = report.azimuth_deg;
                best_track->range_m = report.range_m;
                best_track->hit_count++;
                best_track->coast_count = 0;
                best_track->add_history(report);
            } else {
                int sector = static_cast<int>(report.azimuth_deg / 11.25);
                if (sector >= TrackPool::NUM_SECTORS) sector = TrackPool::NUM_SECTORS - 1;
                track_pool_.create_track(report, sector, filter_type_);
            }
        }
    }
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================================

Track* TrackManager::find_best_track(const Cluster& cluster, 
                                      const std::vector<Track*>& tracks) {
    if (tracks.empty()) return nullptr;
    
    Track* best_track = nullptr;
    double best_score = -1.0;
    
    for (Track* track : tracks) {
        double distance = calculate_distance(cluster, *track);
        bool code_match = is_code_match(cluster, *track);
        double az_diff = calculate_azimuth_diff(
            cluster.get_last_azimuth() * 360.0 / 4096.0,
            track->azimuth_deg
        );
        
        double score = 0.0;
        double dist_score = std::max(0.0, 1.0 - distance / 500.0);
        score += dist_score * 0.4;
        
        double code_score = code_match ? 1.0 : 0.0;
        if (track->mode3a_code == 0 && track->uvd_data20 == 0) {
            code_score = 0.5;
        }
        score += code_score * 0.3;
        
        double az_score = std::max(0.0, 1.0 - az_diff / 30.0);
        score += az_score * 0.3;
        
        if (score > best_score) {
            best_score = score;
            best_track = track;
        }
    }
    
    if (best_score < 0.3) {
        return nullptr;
    }
    
    return best_track;
}

void TrackManager::update_track_with_cluster(Track& track, const Cluster& cluster) {
    auto report = build_report_from_cluster(cluster);
    
    track.x = report.x;
    track.y = report.y;
    track.azimuth_deg = report.azimuth_deg;
    track.range_m = report.range_m;
    track.last_revolution++;
    track.last_update_revolution = track.last_revolution;
    track.hit_count++;
    track.coast_count = 0;
    track.confidence = std::min(1.0, track.confidence + 0.1);
    
    if (report.type == TargetReport::SourceType::RBS) {
        track.mode3a_code = report.rbs.mode3a_code;
        track.spi = report.rbs.spi;
    } else {
        track.uvd_data20 = report.uvd.raw_data20;
        track.altitude = report.uvd.altitude;
    }
    
    track.add_history(report);
    
    if (track.hit_count >= 3 && track.state == TrackState::NEW) {
        track.state = TrackState::ACTIVE;
    }
    
    // Обновляем сектор трека
    int sector = static_cast<int>(track.azimuth_deg / 11.25);
    if (sector >= TrackPool::NUM_SECTORS) sector = TrackPool::NUM_SECTORS - 1;
}

void TrackManager::create_track_from_cluster(const Cluster& cluster, int sector) {
    auto report = build_report_from_cluster(cluster);
    track_pool_.create_track(report, sector, filter_type_);
}

std::vector<TargetReport> TrackManager::split_wide_cluster(const Cluster& cluster) {
    std::vector<TargetReport> reports;
    
    if (cluster.size() < 5) {
        auto report = build_report_from_cluster(cluster);
        if (is_valid_report(report)) {
            reports.push_back(report);
        }
        return reports;
    }
    
    auto report = build_report_from_cluster(cluster);
    if (is_valid_report(report)) {
        reports.push_back(report);
    }
    
    return reports;
}

bool TrackManager::is_valid_cluster(const Cluster& cluster) const {
    if (cluster.is_empty()) return false;
    if (cluster.size() < 2) return false;
    if (cluster.get_max_range() - cluster.get_min_range() < 2) return false;
    return true;
}

bool TrackManager::is_valid_report(const TargetReport& report) const {
    if (report.range_m < 0 || report.range_m > 500000) return false;
    if (report.azimuth_deg < 0 || report.azimuth_deg > 360) return false;
    if (report.signal_strength < 20) return false;
    return true;
}

TargetReport TrackManager::build_report_from_cluster(const Cluster& cluster) {
    auto& buffer = PointBuffer::instance();
    const auto& indices = cluster.get_indices();
    
    TargetReport report;
    report.type = cluster.has_rbs() ? TargetReport::SourceType::RBS : 
                                      TargetReport::SourceType::UVD;
    
    double sum_x = 0.0, sum_y = 0.0;
    double sum_range = 0.0;
    double sum_az_deg = 0.0;
    int count = 0;
    
    for (size_t idx : indices) {
        const auto& point = buffer.get_point(idx);
        
        double az_deg = point.azimuth * 360.0 / 4096.0;
        double range_m = point.range * (point.is_rbs ? 30.0 : 60.0);
        
        double x = range_m * std::sin(az_deg * M_PI / 180.0);
        double y = range_m * std::cos(az_deg * M_PI / 180.0);
        
        sum_x += x;
        sum_y += y;
        sum_range += range_m;
        sum_az_deg += az_deg;
        count++;
    }
    
    if (count > 0) {
        report.x = sum_x / count;
        report.y = sum_y / count;
        report.range_m = sum_range / count;
        report.azimuth_deg = sum_az_deg / count;
        report.signal_strength = static_cast<uint8_t>(
            std::min(255.0, cluster.size() * 25.0)
        );
    }
    
    if (!indices.empty()) {
        const auto& point = buffer.get_point(indices[0]);
        if (point.is_rbs) {
            report.rbs.mode3a_code = point.code12;
            report.rbs.spi = point.spi;
        } else {
            report.uvd.raw_data20 = point.data20;
        }
    }
    
    return report;
}

double TrackManager::calculate_distance(const TargetReport& target, const Track& track) const {
    double dx = target.x - track.x;
    double dy = target.y - track.y;
    return std::sqrt(dx*dx + dy*dy);
}

double TrackManager::calculate_distance(const Cluster& cluster, const Track& track) const {
    auto& buffer = PointBuffer::instance();
    const auto& indices = cluster.get_indices();
    
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    
    for (size_t idx : indices) {
        const auto& point = buffer.get_point(idx);
        double az_deg = point.azimuth * 360.0 / 4096.0;
        double range_m = point.range * (point.is_rbs ? 30.0 : 60.0);
        sum_x += range_m * std::sin(az_deg * M_PI / 180.0);
        sum_y += range_m * std::cos(az_deg * M_PI / 180.0);
        count++;
    }
    
    if (count == 0) return 1e9;
    
    double cx = sum_x / count;
    double cy = sum_y / count;
    
    double dx = cx - track.x;
    double dy = cy - track.y;
    return std::sqrt(dx*dx + dy*dy);
}

bool TrackManager::is_code_match(const TargetReport& target, const Track& track) const {
    if (target.type == TargetReport::SourceType::RBS) {
        if (track.mode3a_code != 0 && track.mode3a_code != target.rbs.mode3a_code) {
            return false;
        }
    }
    return true;
}

bool TrackManager::is_code_match(const Cluster& cluster, const Track& track) const {
    if (!cluster.has_rbs()) return true;
    
    auto& buffer = PointBuffer::instance();
    const auto& indices = cluster.get_indices();
    if (indices.empty()) return true;
    
    const auto& point = buffer.get_point(indices[0]);
    if (!point.is_rbs) return true;
    
    uint16_t cluster_code = point.code12;
    uint16_t track_code = track.mode3a_code;
    
    if (track_code == 0) return true;
    return cluster_code == track_code;
}

double TrackManager::calculate_azimuth_diff(double az1, double az2) const {
    double diff = std::abs(az1 - az2);
    return std::min(diff, 360.0 - diff);
}

} // namespace radar
} // namespace vrl
