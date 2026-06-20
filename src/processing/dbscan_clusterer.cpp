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
    
    // Рассчитываем max_azimuth_gap из ширины диаграммы направленности
    double beamwidth_bins = config_.beamwidth_deg * AZIMUTH_BINS / 360.0;
    max_azimuth_gap_ = static_cast<int>(beamwidth_bins * azimuth_gap_coefficient_);
    if (max_azimuth_gap_ < 1) max_azimuth_gap_ = 1;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer initialized: "
                  "azimuth_gap=" + std::to_string(max_azimuth_gap_) + " MAI" +
                  ", range_gap=" + std::to_string(max_range_gap_) + " bins");
}

// ========================================================================
// ПОИСК КЛАСТЕРА ДЛЯ ТОЧКИ
// ========================================================================

Cluster* DBSCANClusterer::find_cluster(uint16_t azimuth, uint16_t range, bool is_rbs) {
    // Обновляем список активных кластеров
    refresh_active_clusters();
    
    for (Cluster* cluster : active_clusters_) {
        // 1. Проверяем тип (RBS и UVD не смешиваем)
        if (is_rbs && !cluster->has_rbs()) continue;
        if (!is_rbs && !cluster->has_uvd()) continue;
        if (cluster->is_closed()) continue;
        
        // 2. Проверяем азимут (правый край + разрыв)
        int16_t az_gap = azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        if (az_gap > max_azimuth_gap_) continue;
        
        // 3. Проверяем дальность
        if (range < cluster->get_min_range() - max_range_gap_) continue;
        if (range > cluster->get_max_range() + max_range_gap_) continue;
        
        // Все проверки пройдены → подходит
        return cluster;
    }
    
    return nullptr;
}

// ========================================================================
// СОЗДАНИЕ НОВОГО КЛАСТЕРА
// ========================================================================

void DBSCANClusterer::create_cluster(uint16_t azimuth, uint16_t range, bool is_rbs,
                                     size_t buffer_index) {
    (void)azimuth;
    (void)range;
    
    Cluster& cluster = ClusterPool::instance().create_cluster();
    cluster.add_point(buffer_index);
    total_clusters_formed_++;
    
    if (debug_) {
        std::cout << "Created new cluster: az=" << azimuth 
                  << ", range=" << range 
                  << ", type=" << (is_rbs ? "RBS" : "UVD") << std::endl;
    }
}

// ========================================================================
// ОБРАБОТКА ОДНОЙ ТОЧКИ
// ========================================================================

void DBSCANClusterer::process_point(uint16_t azimuth, uint16_t range, bool is_rbs,
                                    const StoredPoint& point, size_t buffer_index) {
    (void)point;
    
    // 1. Закрываем кластеры с большим разрывом
    close_expired_clusters(azimuth);
    
    // 2. Ищем подходящий кластер
    Cluster* target = find_cluster(azimuth, range, is_rbs);
    
    // 3. Добавляем точку или создаем новый кластер
    if (target) {
        target->add_point(buffer_index);
        if (debug_) {
            std::cout << "Added point to cluster: az=" << azimuth 
                      << ", range=" << range << std::endl;
        }
    } else {
        create_cluster(azimuth, range, is_rbs, buffer_index);
    }
    
    // 4. Объединяем перекрывающиеся кластеры
    merge_overlapping_clusters();
}

// ========================================================================
// ЗАКРЫТИЕ КЛАСТЕРОВ С БОЛЬШИМ РАЗРЫВОМ
// ========================================================================

void DBSCANClusterer::close_expired_clusters(uint16_t current_azimuth) {
    refresh_active_clusters();
    
    int closed_count = 0;
    
    for (Cluster* cluster : active_clusters_) {
        if (cluster->is_closed()) continue;
        if (cluster->is_empty()) continue;
        
        int16_t az_gap = current_azimuth - cluster->get_last_azimuth();
        if (az_gap < 0) az_gap += AZIMUTH_BINS;
        
        if (az_gap > max_azimuth_gap_) {
            cluster->close();
            closed_count++;
            total_clusters_completed_++;
            
            if (debug_) {
                std::cout << "Closed cluster: az_span=" 
                          << cluster->get_azimuth_span() 
                          << ", points=" << cluster->size() << std::endl;
            }
        }
    }
    
    if (closed_count > 0 && debug_) {
        std::cout << "Closed " << closed_count << " clusters" << std::endl;
    }
}

// ========================================================================
// ОБНОВЛЕНИЕ СПИСКА АКТИВНЫХ КЛАСТЕРОВ
// ========================================================================

void DBSCANClusterer::refresh_active_clusters() const {
    active_clusters_.clear();
    auto active = ClusterPool::instance().get_active_clusters();
    for (Cluster* cluster : active) {
        if (!cluster->is_closed()) {
            active_clusters_.push_back(cluster);
        }
    }
}

// ========================================================================
// СТАТИСТИКА (const)
// ========================================================================

void DBSCANClusterer::get_stats(size_t& active, size_t& completed) const {
    refresh_active_clusters();
    active = active_clusters_.size();
    completed = ClusterPool::instance().get_closed_clusters().size();
}

// ========================================================================
// ПРОВЕРКА ПЕРЕКРЫТИЯ КЛАСТЕРОВ
// ========================================================================

bool DBSCANClusterer::clusters_overlap(const Cluster& a, const Cluster& b) const {
    if (debug_) {
        std::cout << "Checking overlap:" << std::endl;
        std::cout << "  A: range=[" << a.get_min_range() << "," << a.get_max_range() 
                  << "], last_az=" << a.get_last_azimuth() << std::endl;
        std::cout << "  B: range=[" << b.get_min_range() << "," << b.get_max_range() 
                  << "], last_az=" << b.get_last_azimuth() << std::endl;
    }
    
    // Разные типы не пересекаются
    if (a.has_rbs() != b.has_rbs()) return false;
    if (a.is_empty() || b.is_empty()) return false;
    
    // Проверка дальности
    if (a.get_min_range() > b.get_max_range() + max_range_gap_) {
        if (debug_) std::cout << "  FAIL: range gap (min > max+gap)" << std::endl;
        return false;
    }
    if (b.get_min_range() > a.get_max_range() + max_range_gap_) {
        if (debug_) std::cout << "  FAIL: range gap (b.min > a.max+gap)" << std::endl;
        return false;
    }
    
    // Проверка азимута
    int16_t az_gap = std::abs(static_cast<int16_t>(a.get_last_azimuth() - 
                                                   b.get_last_azimuth()));
    if (az_gap > AZIMUTH_BINS / 2) az_gap = AZIMUTH_BINS - az_gap;
    
    if (debug_) std::cout << "  az_gap=" << az_gap << ", max_az_gap=" << max_azimuth_gap_ << std::endl;
    
    if (az_gap > max_azimuth_gap_) {
        if (debug_) std::cout << "  FAIL: azimuth gap" << std::endl;
        return false;
    }
    
    if (debug_) std::cout << "  OVERLAP: TRUE" << std::endl;
    return true;
}


// ========================================================================
// ОБЪЕДИНЕНИЕ ПЕРЕКРЫВАЮЩИХСЯ КЛАСТЕРОВ
// ========================================================================

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
                    // Объединяем b в a
                    for (size_t idx : b->get_indices()) {
                        a->add_point(idx);
                    }
                    
                    // Удаляем b из пула (не просто закрываем!)
                    ClusterPool::instance().remove_cluster(b);
                    total_clusters_completed_++;
                    
                    refresh_active_clusters();
                    merged = true;
                    merge_count++;
                    
                    if (debug_) {
                        std::cout << "Merged clusters: " << merge_count << std::endl;
                    }
                    break;
                }
            }
            if (merged) break;
        }
    }
    
    if (merge_count > 0 && debug_) {
        std::cout << "Merged " << merge_count << " cluster pairs" << std::endl;
    }
}


// ========================================================================
// ОТЛАДОЧНЫЙ ВЫВОД
// ========================================================================

void DBSCANClusterer::debug_print(const std::string& msg) {
    if (!debug_) return;
    
    refresh_active_clusters();
    
    std::cout << msg << std::endl;
    std::cout << "Active clusters: " << active_clusters_.size() << std::endl;
    
    for (size_t i = 0; i < active_clusters_.size(); ++i) {
        const Cluster* c = active_clusters_[i];
        if (!c) continue;
        std::cout << "  Cluster " << i << ": ";
        std::cout << "points=" << c->size();
        std::cout << ", range=[" << c->get_min_range() << "-" << c->get_max_range() << "]";
        std::cout << ", span=" << c->get_azimuth_span();
        std::cout << ", type=" << (c->has_rbs() ? "RBS" : "UVD");
        std::cout << ", closed=" << (c->is_closed() ? "yes" : "no");
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
    
    // Обрабатываем RBS
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
    
    // Обрабатываем UVD
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
    
    if (debug_) {
        debug_print("After processing:");
    }
}

// ========================================================================
// ВОЗВРАТ ЗАВЕРШЕННЫХ КЛАСТЕРОВ
// ========================================================================

std::vector<TargetCluster> DBSCANClusterer::get_completed_clusters() {
    std::vector<TargetCluster> result;
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    for (Cluster* cluster : closed) {
        // TODO: Преобразовать Cluster в TargetCluster
        TargetCluster tc;
        // Временно пустой
        result.push_back(tc);
    }
    
    return result;
}

const std::vector<TargetCluster>& DBSCANClusterer::get_active_clusters() const {
    static const std::vector<TargetCluster> empty;
    return empty;
}

void DBSCANClusterer::reset() {
    ClusterPool::instance().clear();
    active_clusters_.clear();
    total_scans_processed_ = 0;
    total_points_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    current_revolution_ = 0;
    current_azimuth_ = 0;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer reset");
}

std::vector<TargetCluster> DBSCANClusterer::finish_all() {
    // Закрываем все активные кластеры
    refresh_active_clusters();
    for (Cluster* cluster : active_clusters_) {
        if (!cluster->is_closed()) {
            cluster->close();
            total_clusters_completed_++;
        }
    }
    
    return get_completed_clusters();
}


// ========================================================================
// КЛОНИРОВАНИЕ (исправлено)
// ========================================================================

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
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

} // namespace radar
} // namespace vrl
