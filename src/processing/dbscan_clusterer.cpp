// src/processing/dbscan_clusterer.cpp
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ========================================================================
// КОНСТРУКТОР
// ========================================================================

DBSCANClusterer::DBSCANClusterer(const RadarConfig& config,
                                 int max_range_gap,
                                 int min_points,
                                 double azimuth_gap_coefficient)
    : config_(config)
    , max_range_gap_(max_range_gap)
    , min_points_(min_points)
    , azimuth_gap_coefficient_(azimuth_gap_coefficient) {
    
    // Рассчитываем max_azimuth_gap из ширины диаграммы направленности
    double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
    max_azimuth_gap_ = static_cast<int>(beamwidth_bins * azimuth_gap_coefficient_);
    
    if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer initialized: "
                  "azimuth_gap=" + std::to_string(max_azimuth_gap_) + 
                  " (beamwidth=" + std::to_string(config_.beamwidth_deg) + "°" +
                  ", coeff=" + std::to_string(azimuth_gap_coefficient_) + ")" +
                  ", range_gap=" + std::to_string(max_range_gap_) +
                  ", min_points=" + std::to_string(min_points_));
}

// ========================================================================
// TargetCluster::to_target_cluster
// ========================================================================

TargetCluster DBSCANClusterer::RadarCluster::to_target_cluster(uint32_t revolution) const {
    TargetCluster cluster;
    
    std::map<uint16_t, ScanReplies> scan_map;
    
    for (const auto& point : points) {
        auto it = scan_map.find(point.azimuth);
        if (it == scan_map.end()) {
            ScanReplies scan(point.azimuth, 0);
            scan_map[point.azimuth] = scan;
            it = scan_map.find(point.azimuth);
        }
        
        if (point.is_rbs && point.rbs_reply) {
            it->second.rbs_replies.push_back(*point.rbs_reply);
        } else if (!point.is_rbs && point.uvd_reply) {
            it->second.uvd_replies.push_back(*point.uvd_reply);
        }
    }
    
    for (auto& [azimuth, scan] : scan_map) {
        cluster.add_scan(scan, revolution);
    }
    
    return cluster;
}

// ========================================================================
// ОТЛАДОЧНЫЙ ВЫВОД
// ========================================================================

void DBSCANClusterer::debug_print_clusters(const std::string& prefix) {
    if (!debug_) return;
    
    std::cout << prefix << "Active clusters: " << active_clusters_.size() << std::endl;
    
    for (size_t i = 0; i < active_clusters_.size(); ++i) {
        const auto& c = active_clusters_[i];
        std::cout << "  Cluster " << i << ": ";
        std::cout << "type=" << (c.type == RadarCluster::Type::RBS ? "RBS" : "UVD");
        std::cout << ", points=" << c.points.size();
        std::cout << ", az_range=[";
        
        if (c.azimuths.empty()) {
            std::cout << "empty";
        } else {
            auto it = c.azimuths.begin();
            std::cout << *it;
            auto last = c.azimuths.end();
            --last;
            if (*it != *last) {
                std::cout << "-" << *last;
            }
            if (c.crosses_north()) {
                std::cout << " (crosses North)";
            }
        }
        std::cout << "], range=[" << c.min_range << "-" << c.max_range << "]";
        std::cout << ", closed=" << (c.is_closed ? "yes" : "no");
        std::cout << std::endl;
    }
}

// ========================================================================
// IClusterer РЕАЛИЗАЦИЯ
// ========================================================================

void DBSCANClusterer::process_scan(const ScanReplies& scan) {
    total_scans_processed_++;
    current_revolution_++;
    current_azimuth_ = scan.azimuth;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing scan: azimuth=" + 
                  std::to_string(scan.azimuth) + 
                  ", rbs=" + std::to_string(scan.rbs_replies.size()) +
                  ", uvd=" + std::to_string(scan.uvd_replies.size()));
    
    if (debug_) {
        std::cout << "\n=== Processing scan " << total_scans_processed_ 
                  << " azimuth=" << scan.azimuth << " ===" << std::endl;
    }
    
    process_azimuth(scan.rbs_replies, scan.uvd_replies, 
                    scan.azimuth, current_revolution_);
    
    if (debug_) {
        debug_print_clusters("After adding points: ");
    }
    
    // Закрываем кластеры с большим разрывом
    close_expired_clusters(scan.azimuth, current_revolution_);
    
    if (debug_) {
        debug_print_clusters("After closing: ");
    }
    
    // Завершаем закрытые кластеры
    finalize_closed_clusters();
    
    if (debug_) {
        std::cout << "Completed clusters: " << completed_clusters_.size() << std::endl;
    }
}

void DBSCANClusterer::process_azimuth(const std::vector<RBSReply>& rbs_replies,
                                      const std::vector<UVDReply>& uvd_replies,
                                      uint16_t azimuth,
                                      uint32_t revolution) {
    (void)azimuth;
    (void)revolution;
    
    // Обрабатываем RBS
    for (const auto& reply : rbs_replies) {
        Point point = Point::from_rbs(reply);
        bool added = try_add_to_clusters(point, revolution);
        total_points_processed_++;
        
        if (debug_) {
            std::cout << "  RBS: az=" << point.azimuth << ", range=" << point.range;
            std::cout << ", added=" << (added ? "yes" : "no (new cluster)") << std::endl;
        }
    }
    
    // Обрабатываем UVD (отдельно)
    for (const auto& reply : uvd_replies) {
        Point point = Point::from_uvd(reply);
        bool added = try_add_to_clusters(point, revolution);
        total_points_processed_++;
        
        if (debug_) {
            std::cout << "  UVD: az=" << point.azimuth << ", range=" << point.range;
            std::cout << ", added=" << (added ? "yes" : "no (new cluster)") << std::endl;
        }
    }
    
    // Объединяем перекрывающиеся кластеры
    merge_overlapping_clusters();
    
    if (debug_) {
        std::cout << "After merge:" << std::endl;
        debug_print_clusters();
    }
}

// src/processing/dbscan_clusterer.cpp
// В методе try_add_to_clusters добавьте отладочный вывод:

bool DBSCANClusterer::try_add_to_clusters(const Point& point, uint32_t revolution) {
    bool added = false;
    
    // Отладочный вывод
    if (debug_) {
        std::cout << "  Trying to add point: az=" << point.azimuth 
                  << ", range=" << point.range 
                  << ", is_rbs=" << point.is_rbs 
                  << ", active_clusters=" << active_clusters_.size() << std::endl;
    }
    
    // Ищем подходящий кластер
    int best_range_diff = std::numeric_limits<int>::max();
    size_t best_cluster_idx = 0;
    bool found = false;
    
    for (size_t i = 0; i < active_clusters_.size(); ++i) {
        auto& cluster = active_clusters_[i];
        
        bool can_add = cluster.can_add_point(point, max_azimuth_gap_, max_range_gap_);
        
        if (debug_) {
            std::cout << "    Cluster " << i << ": can_add=" << can_add 
                      << ", type=" << (cluster.type == RadarCluster::Type::RBS ? "RBS" : "UVD")
                      << ", range=[" << cluster.min_range << "-" << cluster.max_range << "]"
                      << ", last_az=" << cluster.last_azimuth << std::endl;
        }
        
        if (can_add) {
            int avg_range = (cluster.min_range + cluster.max_range) / 2;
            int range_diff = std::abs(static_cast<int>(point.range) - avg_range);
            
            if (range_diff < best_range_diff) {
                best_range_diff = range_diff;
                best_cluster_idx = i;
                found = true;
            }
        }
    }
    
    if (found) {
        if (debug_) {
            std::cout << "    Adding to cluster " << best_cluster_idx << std::endl;
        }
        active_clusters_[best_cluster_idx].add_point(point);
        active_clusters_[best_cluster_idx].last_revolution = revolution;
        added = true;
    }
    
    if (!added) {
        if (debug_) {
            std::cout << "    Creating new cluster" << std::endl;
        }
        RadarCluster new_cluster;
        new_cluster.add_point(point);
        new_cluster.created_revolution = revolution;
        new_cluster.last_revolution = revolution;
        active_clusters_.push_back(std::move(new_cluster));
        total_clusters_formed_++;
        
        std::string type_str = point.is_rbs ? "RBS" : "UVD";
        VRL_LOG_TRACE(modules::CLUSTER, "Created new cluster: type=" + type_str +
                      ", az=" + std::to_string(point.azimuth) +
                      ", range=" + std::to_string(point.range));
    }
    
    return added;
}


void DBSCANClusterer::merge_overlapping_clusters() {
    bool merged = true;
    int merge_count = 0;
    
    while (merged) {
        merged = false;
        
        for (size_t i = 0; i < active_clusters_.size(); ++i) {
            for (size_t j = i + 1; j < active_clusters_.size(); ++j) {
                if (active_clusters_[i].overlaps(active_clusters_[j], max_range_gap_)) {
                    if (debug_) {
                        std::cout << "  Merging clusters " << i << " and " << j << std::endl;
                    }
                    
                    active_clusters_[i].merge(active_clusters_[j]);
                    active_clusters_.erase(active_clusters_.begin() + j);
                    merged = true;
                    merge_count++;
                    break;
                }
            }
            if (merged) break;
        }
    }
    
    if (merge_count > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Merged " + std::to_string(merge_count) + 
                      " cluster pairs");
    }
}

void DBSCANClusterer::close_expired_clusters(uint16_t current_azimuth, 
                                             uint32_t revolution) {
    int closed_count = 0;
    
    for (auto& cluster : active_clusters_) {
        if (cluster.is_closed) continue;
        if (cluster.points.empty()) continue;
        
        int16_t az_gap = current_azimuth - cluster.last_azimuth;
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
        if (debug_) {
            std::cout << "  Checking cluster: last_az=" << cluster.last_azimuth;
            std::cout << ", current_az=" << current_azimuth;
            std::cout << ", gap=" << az_gap;
            std::cout << ", max_gap=" << max_azimuth_gap_;
            std::cout << ", points=" << cluster.points.size();
            std::cout << std::endl;
        }
        
        if (az_gap > max_azimuth_gap_) {
            cluster.is_closed = true;
            cluster.last_revolution = revolution;
            closed_count++;
            
            VRL_LOG_TRACE(modules::CLUSTER, "Closed cluster: az_range=" +
                          std::to_string(cluster.min_azimuth) + "-" +
                          std::to_string(cluster.max_azimuth) +
                          ", range=" + std::to_string(cluster.min_range) +
                          "-" + std::to_string(cluster.max_range) +
                          ", points=" + std::to_string(cluster.points.size()));
        }
    }
    
    if (closed_count > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Closed " + std::to_string(closed_count) + 
                      " clusters");
    }
}


// ========================================================================
// ОСТАЛЬНЫЕ МЕТОДЫ IClusterer
// ========================================================================

const std::vector<TargetCluster>& DBSCANClusterer::get_active_clusters() const {
    static std::vector<TargetCluster> empty;
    return empty;
}

void DBSCANClusterer::reset() {
    active_clusters_.clear();
    completed_clusters_.clear();
    total_scans_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    total_points_processed_ = 0;
    current_revolution_ = 0;
    current_azimuth_ = 0;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer reset");
}


void DBSCANClusterer::get_stats(size_t& active, size_t& completed) const {
    active = active_clusters_.size();
    completed = completed_clusters_.size();
}

std::unique_ptr<IClusterer> DBSCANClusterer::clone() const {
    auto clone = std::make_unique<DBSCANClusterer>(
        config_, max_range_gap_, min_points_, azimuth_gap_coefficient_);
    clone->set_max_azimuth_gap(max_azimuth_gap_);
    clone->set_debug(debug_);
    return clone;
}

void DBSCANClusterer::set_param(const std::string& key, double value) {
    if (key == "azimuth_gap_coefficient") {
        azimuth_gap_coefficient_ = value;
        // НЕ пересчитываем max_azimuth_gap_ если он был установлен вручную
        // Используем флаг для отслеживания
        double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
        max_azimuth_gap_ = static_cast<int>(beamwidth_bins * value);
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: azimuth_gap_coefficient=" +
                      std::to_string(value) + " -> gap=" + 
                      std::to_string(max_azimuth_gap_));
    } else if (key == "max_range_gap") {
        max_range_gap_ = static_cast<int>(value);
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown double parameter: " + key);
    }
}

void DBSCANClusterer::set_param(const std::string& key, int value) {
    if (key == "max_azimuth_gap") {
        max_azimuth_gap_ = value;
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: max_azimuth_gap=" + 
                      std::to_string(value));
    } else if (key == "max_range_gap") {
        max_range_gap_ = value;
    } else if (key == "min_points") {
        min_points_ = value;
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

// ========================================================================
// ЗАВЕРШЕНИЕ ВСЕХ КЛАСТЕРОВ
// ========================================================================

std::vector<TargetCluster> DBSCANClusterer::finish_all() {
    VRL_LOG_DEBUG(modules::CLUSTER, "Finishing all clusters (" +
                  std::to_string(active_clusters_.size()) + " active)");
    
    // Принудительно закрываем все активные кластеры
    for (auto& cluster : active_clusters_) {
        if (!cluster.is_closed) {
            cluster.is_closed = true;
            cluster.last_revolution = current_revolution_;
        }
    }
    
    // Завершаем закрытые кластеры
    finalize_closed_clusters();
    
    // Возвращаем все завершенные
    auto result = get_completed_clusters();
    
    VRL_LOG_DEBUG(modules::CLUSTER, "Finished " + std::to_string(result.size()) + 
                  " clusters");
    
    return result;
}

// ========================================================================
// ЗАВЕРШЕНИЕ ЗАКРЫТЫХ КЛАСТЕРОВ
// ========================================================================

void DBSCANClusterer::finalize_closed_clusters() {
    auto it = active_clusters_.begin();
    int finalized = 0;
    
    while (it != active_clusters_.end()) {
        if (it->is_closed) {
            if (it->has_min_points(min_points_)) {
                TargetCluster target = it->to_target_cluster(current_revolution_);
                completed_clusters_.push_back(std::move(target));
                total_clusters_completed_++;
                finalized++;
                
                if (debug_) {
                    std::cout << "  Finalized cluster: points=" << it->points.size();
                    std::cout << ", az_range=[";
                    auto az_it = it->azimuths.begin();
                    std::cout << *az_it;
                    auto last = it->azimuths.end();
                    --last;
                    if (*az_it != *last) {
                        std::cout << "-" << *last;
                    }
                    std::cout << "], range=[" << it->min_range << "-" << it->max_range << "]";
                    std::cout << std::endl;
                }
            } else {
                if (debug_) {
                    std::cout << "  Discarded cluster: points=" << it->points.size();
                    std::cout << " < min_points=" << min_points_ << std::endl;
                }
            }
            it = active_clusters_.erase(it);
        } else {
            ++it;
        }
    }
    
    if (finalized > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Finalized " + std::to_string(finalized) + 
                      " clusters");
    }
}

// ========================================================================
// ПОЛУЧЕНИЕ ЗАВЕРШЕННЫХ КЛАСТЕРОВ
// ========================================================================

std::vector<TargetCluster> DBSCANClusterer::get_completed_clusters() {
    auto result = std::move(completed_clusters_);
    completed_clusters_.clear();
    
    if (debug_) {
        std::cout << "get_completed_clusters: returning " << result.size() 
                  << " clusters" << std::endl;
    }
    
    return result;
}

} // namespace radar
} // namespace vrl
