// src/processing/dbscan_clusterer.cpp
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ========================================================================
// КОНСТРУКТОР
// ========================================================================

DBSCANClusterer::DBSCANClusterer(const RadarConfig& config,
                                 int max_range_gap,
                                 double azimuth_gap_coefficient)
    : config_(config)
    , max_range_gap_(max_range_gap)
    , azimuth_gap_coefficient_(azimuth_gap_coefficient) {
    
    double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
    max_azimuth_gap_ = static_cast<int>(beamwidth_bins * azimuth_gap_coefficient_);
    if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
    
    wide_cluster_threshold_ = max_azimuth_gap_;
    
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[DBSCANClusterer] Constructor:" << std::endl;
        std::cout << "  beamwidth_deg = " << config_.beamwidth_deg << std::endl;
        std::cout << "  beamwidth_bins = " << beamwidth_bins << std::endl;
        std::cout << "  azimuth_gap_coefficient = " << azimuth_gap_coefficient_ << std::endl;
        std::cout << "  max_azimuth_gap = " << max_azimuth_gap_ << std::endl;
        std::cout << "  max_range_gap = " << max_range_gap_ << std::endl;
        std::cout << "  wide_cluster_threshold = " << wide_cluster_threshold_ << std::endl;
    }
#endif
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer initialized: "
                  "azimuth_gap=" + std::to_string(max_azimuth_gap_) + " MAI" +
                  ", range_gap=" + std::to_string(max_range_gap_) + " bins" +
                  ", wide_threshold=" + std::to_string(wide_cluster_threshold_) + " bins");
}

// ========================================================================
// ПОИСК КЛАСТЕРА ДЛЯ ТОЧКИ
// ========================================================================

Cluster* DBSCANClusterer::find_cluster(uint16_t azimuth, uint16_t range, bool is_rbs) {
    auto debug = debug_;
    
    // ИСПРАВЛЕНО: используем только активные кластеры
    auto active_clusters = ClusterPool::instance().get_active_clusters();
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[find_cluster] Looking for cluster: az=" << azimuth 
                  << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
        std::cout << "[find_cluster] active clusters=" << active_clusters.size() << std::endl;
    }
#endif
    
    for (Cluster* cluster : active_clusters) {
        if (!cluster || cluster->is_closed()) continue;
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[find_cluster] Checking cluster id=" << cluster->get_id()
                      << ": last_az=" << cluster->get_last_azimuth()
                      << ", min_range=" << cluster->get_min_range()
                      << ", max_range=" << cluster->get_max_range()
                      << ", has_rbs=" << cluster->has_rbs()
                      << ", has_uvd=" << cluster->has_uvd()
                      << ", is_closed=" << cluster->is_closed() << std::endl;
        }
#endif
        
        if (is_rbs && !cluster->has_rbs()) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[find_cluster]  SKIP: cluster has no RBS" << std::endl;
            }
#endif
            continue;
        }
        if (!is_rbs && !cluster->has_uvd()) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[find_cluster]  SKIP: cluster has no UVD" << std::endl;
            }
#endif
            continue;
        }
        
        int16_t az_gap = azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[find_cluster]  az_gap = " << az_gap 
                      << ", max_azimuth_gap = " << max_azimuth_gap_ << std::endl;
        }
#endif
        
        if (az_gap > max_azimuth_gap_) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[find_cluster]  SKIP: az_gap > max_azimuth_gap" << std::endl;
            }
#endif
            continue;
        }
        
        if (range < cluster->get_min_range() - max_range_gap_) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[find_cluster]  SKIP: range too low" << std::endl;
            }
#endif
            continue;
        }
        if (range > cluster->get_max_range() + max_range_gap_) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[find_cluster]  SKIP: range too high" << std::endl;
            }
#endif
            continue;
        }
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[find_cluster]  FOUND matching cluster id=" << cluster->get_id() << std::endl;
        }
#endif
        return cluster;
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[find_cluster] No matching cluster found" << std::endl;
    }
#endif
    return nullptr;
}

// ========================================================================
// СОЗДАНИЕ НОВОГО КЛАСТЕРА
// ========================================================================

void DBSCANClusterer::create_cluster(uint16_t azimuth, uint16_t range, bool is_rbs,
                                     size_t buffer_index) {
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[create_cluster] Creating new cluster: az=" << azimuth 
                  << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
    }
#endif
    
    uint64_t id = ClusterPool::instance().create_cluster();
    Cluster* cluster = ClusterPool::instance().get_cluster(id);
    if (cluster) {
        cluster->add_point(buffer_index);
        total_clusters_formed_++;
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[create_cluster] New cluster id=" << id
                      << ", total_clusters_formed=" << total_clusters_formed_ << std::endl;
        }
#endif
    } else {
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[create_cluster] ERROR: failed to create cluster" << std::endl;
        }
#endif
        VRL_LOG_ERROR(modules::CLUSTER, "Failed to create cluster");
    }
}

// ========================================================================
// ОБРАБОТКА ОДНОЙ ТОЧКИ
// ========================================================================

void DBSCANClusterer::process_point(uint16_t azimuth, uint16_t range, bool is_rbs,
                                    const StoredPoint& point, size_t buffer_index) {
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[process_point] Processing: az=" << azimuth 
                  << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
    }
#endif
    
    close_expired_clusters(azimuth);
    
    Cluster* target = find_cluster(azimuth, range, is_rbs);
    
    if (target) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[process_point] Adding point to existing cluster id=" 
                      << target->get_id() << std::endl;
        }
#endif
        target->add_point(buffer_index);
        
        // Проверяем на широкость
        int azimuth_span = target->get_azimuth_span();
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[process_point] azimuth_span=" << azimuth_span 
                      << ", wide_threshold=" << wide_cluster_threshold_ << std::endl;
        }
#endif
        
        if (azimuth_span > wide_cluster_threshold_) {
            uint64_t id = target->get_id();
            if (id > 0) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
                if (debug) {
                    std::cout << "[process_point] Cluster id=" << id 
                              << " became WIDE (span=" << azimuth_span << ")" << std::endl;
                }
#endif
                ClusterPool::instance().add_to_wide(id);
            }
        }
    } else {
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[process_point] Creating new cluster" << std::endl;
        }
#endif
        create_cluster(azimuth, range, is_rbs, buffer_index);
    }
    
    merge_overlapping_clusters();
}

// ========================================================================
// ЗАКРЫТИЕ КЛАСТЕРОВ С БОЛЬШИМ РАЗРЫВОМ
// ========================================================================

void DBSCANClusterer::close_expired_clusters(uint16_t current_azimuth) {
    auto debug = debug_;
    
    // ИСПРАВЛЕНО: используем только активные кластеры
    auto active_clusters = ClusterPool::instance().get_active_clusters();
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[close_expired_clusters] current_azimuth=" << current_azimuth
                  << ", active clusters=" << active_clusters.size() << std::endl;
    }
#endif
    
    int closed_count = 0;
    
    for (Cluster* cluster : active_clusters) {
        if (!cluster) continue;
        if (cluster->is_closed()) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[close_expired_clusters]  cluster id=" << cluster->get_id() 
                          << " already closed" << std::endl;
            }
#endif
            continue;
        }
        if (cluster->is_empty()) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[close_expired_clusters]  cluster id=" << cluster->get_id() 
                          << " empty" << std::endl;
            }
#endif
            continue;
        }
        
        int16_t az_gap = current_azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[close_expired_clusters]  cluster id=" << cluster->get_id()
                      << ": last_az=" << cluster->get_last_azimuth()
                      << ", az_gap=" << az_gap << ", max_azimuth_gap=" << max_azimuth_gap_ << std::endl;
        }
#endif
        
        if (az_gap > max_azimuth_gap_) {
            cluster->close();
            uint64_t id = cluster->get_id();
            
#ifdef CMAKE_BUILD_TYPE_DEBUG
            if (debug) {
                std::cout << "[close_expired_clusters]  CLOSING cluster id=" << id << std::endl;
            }
#endif
            
            if (id > 0) {
                int sector = cluster->get_last_azimuth() / ClusterPool::SECTOR_SIZE;
                if (sector >= ClusterPool::NUM_SECTORS) sector = ClusterPool::NUM_SECTORS - 1;
                ClusterPool::instance().close_cluster(id, sector);
            }
            
            closed_count++;
            total_clusters_completed_++;
        }
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug && closed_count > 0) {
        std::cout << "[close_expired_clusters] closed " << closed_count << " clusters" << std::endl;
    }
#endif
}

// ========================================================================
// СТАТИСТИКА
// ========================================================================

void DBSCANClusterer::get_stats(size_t& active, size_t& completed) const {
    active = ClusterPool::instance().count_active_clusters();
    completed = ClusterPool::instance().count_closed_clusters();
}

// ========================================================================
// MERGE ОВЕРЛАППИНГ КЛАСТЕРОВ
// ========================================================================

bool DBSCANClusterer::clusters_overlap(const Cluster& a, const Cluster& b) const {
    if (a.has_rbs() != b.has_rbs()) return false;
    if (a.is_empty() || b.is_empty()) return false;
    
    if (a.get_min_range() > b.get_max_range() + max_range_gap_) return false;
    if (b.get_min_range() > a.get_max_range() + max_range_gap_) return false;
    
    int16_t az_gap = std::abs(static_cast<int16_t>(a.get_last_azimuth() - 
                                                   b.get_last_azimuth()));
    if (az_gap > AZIMUTH_BINS / 2) az_gap = AZIMUTH_BINS - az_gap;
    
    return az_gap <= max_azimuth_gap_;
}

void DBSCANClusterer::merge_overlapping_clusters() {
    auto debug = debug_;
    
    bool merged = true;
    int merge_count = 0;
    int max_iterations = 100;  // Защита от бесконечного цикла
    int iteration = 0;
    
    while (merged && iteration < max_iterations) {
        merged = false;
        iteration++;
        
        // ИСПРАВЛЕНО: используем только активные кластеры
        auto active = ClusterPool::instance().get_active_clusters();
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[merge_overlapping_clusters] iteration=" << iteration 
                      << ", active_count=" << active.size() << std::endl;
        }
#endif
        
        for (size_t i = 0; i < active.size(); ++i) {
            for (size_t j = i + 1; j < active.size(); ++j) {
                Cluster* a = active[i];
                Cluster* b = active[j];
                
                if (!a || !b) continue;
                if (a->is_closed() || b->is_closed()) continue;

                if (clusters_overlap(*a, *b)) {
#ifdef CMAKE_BUILD_TYPE_DEBUG
                    if (debug) {
                        std::cout << "[merge_overlapping_clusters] Merging clusters " 
                                << a->get_id() << " and " << b->get_id() << std::endl;
                    }
#endif
                    
                    // Используем готовую функцию из ClusterPool
                    ClusterPool::instance().merge_clusters(a->get_id(), b->get_id());
                    total_clusters_completed_++;
                    
                    merged = true;
                    merge_count++;
                    break;
                }                
            }
            if (merged) break;  // Выходим из внешнего цикла
        }
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        if (iteration >= max_iterations) {
            std::cout << "[merge_overlapping_clusters] WARNING: reached max iterations=" 
                      << max_iterations << std::endl;
        }
        if (merge_count > 0) {
            std::cout << "[merge_overlapping_clusters] Merged " << merge_count 
                      << " pairs in " << iteration << " iterations" << std::endl;
        }
    }
#endif
}

// ========================================================================
// ОСТАЛЬНЫЕ МЕТОДЫ
// ========================================================================

void DBSCANClusterer::process_scan(const ScanReplies& scan) {
    total_scans_processed_++;
    current_revolution_++;
    current_azimuth_ = scan.azimuth;
    
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "\n[process_scan] Processing scan: az=" << scan.azimuth 
                  << ", rbs=" << scan.rbs_replies.size()
                  << ", uvd=" << scan.uvd_replies.size() << std::endl;
    }
#endif
    
    for (const auto& reply : scan.rbs_replies) {
        StoredPoint point;
        point.azimuth = reply.azimuth;
        point.range = reply.range;
        point.is_rbs = true;
        point.amplitude = (reply.ether_amplitudes[0] + reply.ether_amplitudes[14]) / 2;
        point.spi = reply.spi;
        point.code12 = reply.code12;
        point.rbs_reply = &reply;
        
        size_t buffer_idx = PointBuffer::instance().add_point(point);
        process_point(reply.azimuth, reply.range, true, point, buffer_idx);
    }
    
    for (const auto& reply : scan.uvd_replies) {
        StoredPoint point;
        point.azimuth = reply.azimuth;
        point.range = reply.range;
        point.is_rbs = false;
        uint32_t sum = 0;
        for (uint8_t amp : reply.ether_amplitudes) {
            sum += amp;
        }
        point.amplitude = static_cast<uint16_t>(sum / reply.ether_amplitudes.size());
        point.data20 = reply.data20;
        point.uvd_reply = &reply;
        
        size_t buffer_idx = PointBuffer::instance().add_point(point);
        process_point(reply.azimuth, reply.range, false, point, buffer_idx);
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[process_scan] Done. active_clusters=" << count_active_clusters() << std::endl;
    }
#endif
}

std::vector<TargetCluster> DBSCANClusterer::get_completed_clusters() {
    return {};
}

const std::vector<TargetCluster>& DBSCANClusterer::get_active_clusters() const {
    static const std::vector<TargetCluster> empty;
    return empty;
}

void DBSCANClusterer::reset() {
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[reset] Resetting DBSCANClusterer" << std::endl;
    }
#endif
    
    ClusterPool::instance().clear();
    total_scans_processed_ = 0;
    total_points_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    current_revolution_ = 0;
    current_azimuth_ = 0;
}

std::vector<TargetCluster> DBSCANClusterer::finish_all() {
    auto debug = debug_;
    
    // ИСПРАВЛЕНО: используем только активные кластеры
    auto active_clusters = ClusterPool::instance().get_active_clusters();
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[finish_all] Closing " << active_clusters.size() << " clusters" << std::endl;
    }
#endif
    
    for (Cluster* cluster : active_clusters) {
        if (!cluster) continue;
        if (!cluster->is_closed()) {
            cluster->close();
            uint64_t id = cluster->get_id();
            if (id > 0) {
                int sector = cluster->get_last_azimuth() / ClusterPool::SECTOR_SIZE;
                if (sector >= ClusterPool::NUM_SECTORS) sector = ClusterPool::NUM_SECTORS - 1;
                ClusterPool::instance().close_cluster(id, sector);
            }
            total_clusters_completed_++;
        }
    }
    
    return get_completed_clusters();
}

std::unique_ptr<IClusterer> DBSCANClusterer::clone() const {
    auto clone = std::make_unique<DBSCANClusterer>(
        config_, max_range_gap_, azimuth_gap_coefficient_);
    clone->max_azimuth_gap_ = max_azimuth_gap_;
    clone->wide_cluster_threshold_ = wide_cluster_threshold_;
    clone->set_debug(debug_);
    return clone;
}

void DBSCANClusterer::set_param(const std::string& key, double value) {
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[set_param] key=" << key << ", value=" << value << std::endl;
    }
#endif
    
    if (key == "azimuth_gap_coefficient") {
        azimuth_gap_coefficient_ = value;
        double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
        max_azimuth_gap_ = static_cast<int>(beamwidth_bins * value);
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] azimuth_gap_coefficient=" << value 
                      << " -> max_azimuth_gap=" << max_azimuth_gap_ << std::endl;
        }
#endif
    } else if (key == "max_range_gap") {
        max_range_gap_ = static_cast<int>(value);
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] max_range_gap=" << max_range_gap_ << std::endl;
        }
#endif
    } else if (key == "wide_cluster_threshold") {
        wide_cluster_threshold_ = static_cast<int>(value);
        if (wide_cluster_threshold_ < 1) wide_cluster_threshold_ = 1;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] wide_cluster_threshold=" << wide_cluster_threshold_ << std::endl;
        }
#endif
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown double parameter: " + key);
    }
}

void DBSCANClusterer::set_param(const std::string& key, int value) {
    auto debug = debug_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[set_param] key=" << key << ", value=" << value << std::endl;
    }
#endif
    
    if (key == "max_azimuth_gap") {
        max_azimuth_gap_ = value;
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] max_azimuth_gap=" << value << std::endl;
        }
#endif
    } else if (key == "max_range_gap") {
        max_range_gap_ = value;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] max_range_gap=" << value << std::endl;
        }
#endif
    } else if (key == "wide_cluster_threshold") {
        wide_cluster_threshold_ = value;
        if (wide_cluster_threshold_ < 1) wide_cluster_threshold_ = 1;
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[set_param] wide_cluster_threshold=" << value << std::endl;
        }
#endif
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

size_t DBSCANClusterer::count_active_clusters() const {
    // ИСПРАВЛЕНО: используем готовый метод ClusterPool
    return ClusterPool::instance().count_active_clusters();
}

} // namespace radar
} // namespace vrl
