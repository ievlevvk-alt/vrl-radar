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
    
    std::cout << "[DBSCANClusterer] Constructor:" << std::endl;
    std::cout << "  beamwidth_deg = " << config_.beamwidth_deg << std::endl;
    std::cout << "  beamwidth_bins = " << beamwidth_bins << std::endl;
    std::cout << "  azimuth_gap_coefficient = " << azimuth_gap_coefficient_ << std::endl;
    std::cout << "  max_azimuth_gap = " << max_azimuth_gap_ << std::endl;
    std::cout << "  max_range_gap = " << max_range_gap_ << std::endl;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer initialized: "
                  "azimuth_gap=" + std::to_string(max_azimuth_gap_) + " MAI" +
                  ", range_gap=" + std::to_string(max_range_gap_) + " bins");
}

// ========================================================================
// ПОИСК КЛАСТЕРА ДЛЯ ТОЧКИ
// ========================================================================

Cluster* DBSCANClusterer::find_cluster(uint16_t azimuth, uint16_t range, bool is_rbs) {
    refresh_active_clusters();
    
    std::cout << "[find_cluster] Looking for cluster: az=" << azimuth 
              << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
    std::cout << "[find_cluster] active_clusters_.size()=" << active_clusters_.size() << std::endl;
    
    for (Cluster* cluster : active_clusters_) {
        std::cout << "[find_cluster] Checking cluster: last_az=" << cluster->get_last_azimuth()
                  << ", min_range=" << cluster->get_min_range()
                  << ", max_range=" << cluster->get_max_range()
                  << ", has_rbs=" << cluster->has_rbs()
                  << ", has_uvd=" << cluster->has_uvd()
                  << ", is_closed=" << cluster->is_closed() << std::endl;
        
        if (is_rbs && !cluster->has_rbs()) {
            std::cout << "[find_cluster]  SKIP: cluster has no RBS" << std::endl;
            continue;
        }
        if (!is_rbs && !cluster->has_uvd()) {
            std::cout << "[find_cluster]  SKIP: cluster has no UVD" << std::endl;
            continue;
        }
        if (cluster->is_closed()) {
            std::cout << "[find_cluster]  SKIP: cluster is closed" << std::endl;
            continue;
        }
        
        int16_t az_gap = azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
        std::cout << "[find_cluster]  az_gap = " << az_gap 
                  << ", max_azimuth_gap = " << max_azimuth_gap_ << std::endl;
        
        if (az_gap > max_azimuth_gap_) {
            std::cout << "[find_cluster]  SKIP: az_gap > max_azimuth_gap" << std::endl;
            continue;
        }
        
        if (range < cluster->get_min_range() - max_range_gap_) {
            std::cout << "[find_cluster]  SKIP: range too low" << std::endl;
            continue;
        }
        if (range > cluster->get_max_range() + max_range_gap_) {
            std::cout << "[find_cluster]  SKIP: range too high" << std::endl;
            continue;
        }
        
        std::cout << "[find_cluster]  FOUND matching cluster!" << std::endl;
        return cluster;
    }
    
    std::cout << "[find_cluster] No matching cluster found" << std::endl;
    return nullptr;
}

// ========================================================================
// СОЗДАНИЕ НОВОГО КЛАСТЕРА
// ========================================================================

void DBSCANClusterer::create_cluster(uint16_t azimuth, uint16_t range, bool is_rbs,
                                     size_t buffer_index) {
    std::cout << "[create_cluster] Creating new cluster: az=" << azimuth 
              << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
    
    uint64_t id = ClusterPool::instance().create_cluster();
    Cluster* cluster = ClusterPool::instance().get_cluster(id);
    cluster->add_point(buffer_index);
    total_clusters_formed_++;
    
    std::cout << "[create_cluster] New cluster id=" << id
              << ", total_clusters_formed=" << total_clusters_formed_ << std::endl;
}

// ========================================================================
// ОБРАБОТКА ОДНОЙ ТОЧКИ
// ========================================================================

void DBSCANClusterer::process_point(uint16_t azimuth, uint16_t range, bool is_rbs,
                                    const StoredPoint& point, size_t buffer_index) {
    std::cout << "[process_point] Processing: az=" << azimuth 
              << ", range=" << range << ", is_rbs=" << is_rbs << std::endl;
    
    close_expired_clusters(azimuth);
    
    Cluster* target = find_cluster(azimuth, range, is_rbs);
    
    if (target) {
        std::cout << "[process_point] Adding point to existing cluster" << std::endl;
        target->add_point(buffer_index);
    } else {
        std::cout << "[process_point] Creating new cluster" << std::endl;
        create_cluster(azimuth, range, is_rbs, buffer_index);
    }
    
    merge_overlapping_clusters();
}

// ========================================================================
// ЗАКРЫТИЕ КЛАСТЕРОВ С БОЛЬШИМ РАЗРЫВОМ
// ========================================================================

void DBSCANClusterer::close_expired_clusters(uint16_t current_azimuth) {
    refresh_active_clusters();
    
    std::cout << "[close_expired_clusters] current_azimuth=" << current_azimuth
              << ", active_clusters_.size()=" << active_clusters_.size() << std::endl;
    
    int closed_count = 0;
    
    for (Cluster* cluster : active_clusters_) {
        if (cluster->is_closed()) {
            std::cout << "[close_expired_clusters]  cluster already closed" << std::endl;
            continue;
        }
        if (cluster->is_empty()) {
            std::cout << "[close_expired_clusters]  cluster empty" << std::endl;
            continue;
        }
        
        int16_t az_gap = current_azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
        std::cout << "[close_expired_clusters]  cluster: last_az=" << cluster->get_last_azimuth()
                  << ", az_gap=" << az_gap << ", max_azimuth_gap=" << max_azimuth_gap_ << std::endl;
        
        if (az_gap > max_azimuth_gap_) {
            cluster->close();
            
            // Ищем ID кластера по указателю
            uint64_t id = 0;
            bool found = false;
            auto ids = ClusterPool::instance().get_all_ids();
            for (uint64_t cid : ids) {
                if (ClusterPool::instance().get_cluster(cid) == cluster) {
                    id = cid;
                    found = true;
                    break;
                }
            }
            
            if (found) {
                int sector = cluster->get_last_azimuth() / ClusterPool::SECTOR_SIZE;
                if (sector >= ClusterPool::NUM_SECTORS) sector = ClusterPool::NUM_SECTORS - 1;
                
                std::cout << "[close_expired_clusters]  CLOSING cluster id=" << id
                          << ", sector=" << sector << std::endl;
                
                ClusterPool::instance().close_cluster(id, sector);
            }
            
            closed_count++;
            total_clusters_completed_++;
        }
    }
    
    std::cout << "[close_expired_clusters] closed " << closed_count << " clusters" << std::endl;
}

// ========================================================================
// ОБНОВЛЕНИЕ СПИСКА АКТИВНЫХ КЛАСТЕРОВ
// ========================================================================

void DBSCANClusterer::refresh_active_clusters() const {
    active_clusters_.clear();
    
    auto clusters = ClusterPool::instance().get_all_clusters();
    
    std::cout << "[refresh_active_clusters] total clusters=" << clusters.size() << std::endl;
    
    for (Cluster* cluster : clusters) {
        if (cluster && !cluster->is_closed()) {
            active_clusters_.push_back(cluster);
            std::cout << "[refresh_active_clusters]  added cluster: size=" 
                      << cluster->size() << ", last_az=" 
                      << cluster->get_last_azimuth() << std::endl;
        } else if (cluster && cluster->is_closed()) {
            std::cout << "[refresh_active_clusters]  skipping closed cluster" << std::endl;
        }
    }
    
    std::cout << "[refresh_active_clusters] active_clusters_.size()=" 
              << active_clusters_.size() << std::endl;
}

// ========================================================================
// СТАТИСТИКА
// ========================================================================

void DBSCANClusterer::get_stats(size_t& active, size_t& completed) const {
    refresh_active_clusters();
    active = active_clusters_.size();
    completed = ClusterPool::instance().count_closed_clusters();
}

// ========================================================================
// ОСТАЛЬНЫЕ МЕТОДЫ
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
    refresh_active_clusters();
    
    bool merged = true;
    int merge_count = 0;
    
    while (merged) {
        merged = false;
        
        for (size_t i = 0; i < active_clusters_.size(); ++i) {
            for (size_t j = i + 1; j < active_clusters_.size(); ++j) {
                Cluster* a = active_clusters_[i];
                Cluster* b = active_clusters_[j];
                
                if (!a || !b) continue;
                if (a->is_closed() || b->is_closed()) continue;
                
                if (clusters_overlap(*a, *b)) {
                    std::cout << "[merge_overlapping_clusters] Merging clusters " << i << " and " << j << std::endl;
                    
                    for (size_t idx : b->get_indices()) {
                        a->add_point(idx);
                    }
                    
                    // Находим ID b
                    uint64_t b_id = 0;
                    bool found = false;
                    auto ids = ClusterPool::instance().get_all_ids();
                    for (uint64_t cid : ids) {
                        if (ClusterPool::instance().get_cluster(cid) == b) {
                            b_id = cid;
                            found = true;
                            break;
                        }
                    }
                    
                    if (found) {
                        ClusterPool::instance().remove_cluster(b_id);
                        total_clusters_completed_++;
                    }
                    
                    refresh_active_clusters();
                    merged = true;
                    merge_count++;
                    break;
                }
            }
            if (merged) break;
        }
    }
    
    if (merge_count > 0) {
        std::cout << "[merge_overlapping_clusters] Merged " << merge_count << " pairs" << std::endl;
    }
}

void DBSCANClusterer::process_scan(const ScanReplies& scan) {
    total_scans_processed_++;
    current_revolution_++;
    current_azimuth_ = scan.azimuth;
    
    std::cout << "\n[process_scan] Processing scan: az=" << scan.azimuth 
              << ", rbs=" << scan.rbs_replies.size()
              << ", uvd=" << scan.uvd_replies.size() << std::endl;
    
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
    
    std::cout << "[process_scan] Done. active_clusters=" << count_active_clusters() << std::endl;
}

std::vector<TargetCluster> DBSCANClusterer::get_completed_clusters() {
    return {};
}

const std::vector<TargetCluster>& DBSCANClusterer::get_active_clusters() const {
    static const std::vector<TargetCluster> empty;
    return empty;
}

void DBSCANClusterer::reset() {
    std::cout << "[reset] Resetting DBSCANClusterer" << std::endl;
    ClusterPool::instance().clear();
    active_clusters_.clear();
    total_scans_processed_ = 0;
    total_points_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    current_revolution_ = 0;
    current_azimuth_ = 0;
}

std::vector<TargetCluster> DBSCANClusterer::finish_all() {
    refresh_active_clusters();
    
    for (Cluster* cluster : active_clusters_) {
        if (!cluster->is_closed()) {
            cluster->close();
            
            // Ищем ID кластера
            uint64_t id = 0;
            bool found = false;
            auto ids = ClusterPool::instance().get_all_ids();
            for (uint64_t cid : ids) {
                if (ClusterPool::instance().get_cluster(cid) == cluster) {
                    id = cid;
                    found = true;
                    break;
                }
            }
            
            if (found) {
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
    clone->set_debug(debug_);
    return clone;
}

void DBSCANClusterer::set_param(const std::string& key, double value) {
    if (key == "azimuth_gap_coefficient") {
        azimuth_gap_coefficient_ = value;
        double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
        max_azimuth_gap_ = static_cast<int>(beamwidth_bins * value);
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
        std::cout << "[set_param] azimuth_gap_coefficient=" << value 
                  << " -> max_azimuth_gap=" << max_azimuth_gap_ << std::endl;
    } else if (key == "max_range_gap") {
        max_range_gap_ = static_cast<int>(value);
        std::cout << "[set_param] max_range_gap=" << max_range_gap_ << std::endl;
    }
}

void DBSCANClusterer::set_param(const std::string& key, int value) {
    if (key == "max_azimuth_gap") {
        max_azimuth_gap_ = value;
        if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
        std::cout << "[set_param] max_azimuth_gap=" << value << std::endl;
    } else if (key == "max_range_gap") {
        max_range_gap_ = value;
        std::cout << "[set_param] max_range_gap=" << value << std::endl;
    }
}

size_t DBSCANClusterer::count_active_clusters() const {
    refresh_active_clusters();
    return active_clusters_.size();
}

} // namespace radar
} // namespace vrl
