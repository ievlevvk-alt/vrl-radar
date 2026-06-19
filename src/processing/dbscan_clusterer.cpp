// src/processing/dbscan_clusterer.cpp
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <queue>
#include <limits>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

DBSCANClusterer::DBSCANClusterer(double eps_range, double eps_azimuth_deg, 
                                 int min_pts, double range_bin_m)
    : eps_range_(eps_range), eps_azimuth_deg_(eps_azimuth_deg), 
      min_pts_(min_pts), range_bin_m_(range_bin_m) {
    // Переводим eps_azimuth_deg в радианы
    eps_azimuth_rad_ = eps_azimuth_deg * DEG_TO_RAD;
    
    VRL_LOG_INFO(modules::CLUSTER, "DBSCANClusterer initialized: eps_range=" + 
                 std::to_string(eps_range) + "m, eps_azimuth=" + 
                 std::to_string(eps_azimuth_deg) + "° (" + 
                 std::to_string(eps_azimuth_rad_) + " rad), min_pts=" + 
                 std::to_string(min_pts) + ", range_bin=" + 
                 std::to_string(range_bin_m) + "m");
}

double DBSCANClusterer::distance(const Point& a, const Point& b) const {
    // Разница по дальности в метрах
    double range_diff = std::abs(static_cast<int>(a.range) - static_cast<int>(b.range)) * range_bin_m_;
    
    // Если разница по дальности больше допуска - не соседи
    if (range_diff > eps_range_) {
        return std::numeric_limits<double>::max();
    }
    
    // Разница по азимуту в бинах
    int16_t az_diff = static_cast<int16_t>(a.azimuth) - static_cast<int16_t>(b.azimuth);
    if (az_diff < 0) az_diff = -az_diff;
    if (az_diff > AZIMUTH_BINS / 2) {
        az_diff = AZIMUTH_BINS - az_diff;
    }
    
    // Переводим разницу азимута в радианы
    double az_diff_rad = static_cast<double>(az_diff) * 2.0 * M_PI / AZIMUTH_BINS;
    
    // Проверяем азимутальную погрешность на средней дальности
    double avg_range_m = (static_cast<double>(a.range) + static_cast<double>(b.range)) / 2.0 * range_bin_m_;
    double az_diff_m = avg_range_m * std::sin(az_diff_rad);
    
    // Если азимутальное расстояние превышает допуск - не соседи
    if (az_diff_m > eps_range_) {
        return std::numeric_limits<double>::max();
    }
    
    // Евклидово расстояние в метрах (для сортировки и отладки)
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<size_t> DBSCANClusterer::find_neighbors(size_t point_idx) {
    std::vector<size_t> neighbors;
    const Point& p = points_[point_idx];
    
    for (size_t i = 0; i < points_.size(); ++i) {
        if (i == point_idx) continue;
        
        double d = distance(p, points_[i]);
        if (d <= eps_range_) {
            neighbors.push_back(i);
        }
    }
    
    return neighbors;
}

void DBSCANClusterer::process_scan(const ScanReplies& scan) {
    total_scans_processed_++;
    current_azimuth_ = scan.azimuth;
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing scan at azimuth " + 
                  std::to_string(scan.azimuth) + " with " + 
                  std::to_string(scan.reply_count()) + " replies");
    
    // Собираем точки из скана
    std::vector<Point> scan_points;
    
    // RBS ответы
    for (const auto& reply : scan.rbs_replies) {
        Point p;
        p.x = reply.x;
        p.y = reply.y;
        p.range_m = static_cast<double>(reply.range) * range_bin_m_;
        p.azimuth = reply.azimuth;
        p.range = reply.range;
        p.rbs_reply = &reply;
        p.is_rbs = true;
        p.scan_index = total_scans_processed_;
        scan_points.push_back(p);
    }
    
    // UVD ответы
    for (const auto& reply : scan.uvd_replies) {
        Point p;
        p.x = reply.x;
        p.y = reply.y;
        p.range_m = static_cast<double>(reply.range) * range_bin_m_;
        p.azimuth = reply.azimuth;
        p.range = reply.range;
        p.uvd_reply = &reply;
        p.is_rbs = false;
        p.scan_index = total_scans_processed_;
        scan_points.push_back(p);
    }
    
    if (scan_points.empty()) {
        complete_expired_clusters();
        return;
    }
    
    // Добавляем точки в общий список
    points_.insert(points_.end(), scan_points.begin(), scan_points.end());
    
    // Выполняем DBSCAN на новых точках
    run_dbscan();
    
    // Обновляем активные кластеры
    update_active_clusters();
    
    // Завершаем кластеры, которые уже не активны
    complete_expired_clusters();
}

void DBSCANClusterer::run_dbscan() {
    if (points_.empty()) return;
    
    int cluster_id = 0;
    
    // Находим максимальный существующий ID кластера
    for (const auto& p : points_) {
        if (p.cluster_id > cluster_id) {
            cluster_id = p.cluster_id;
        }
    }
    
    // Проходим по всем точкам
    for (size_t i = 0; i < points_.size(); ++i) {
        if (points_[i].visited) continue;
        
        points_[i].visited = true;
        
        auto neighbors = find_neighbors(i);
        
        if (neighbors.size() < static_cast<size_t>(min_pts_)) {
            // Шум
            points_[i].cluster_id = -1;
            total_noise_points_++;
        } else {
            // Создаем новый кластер
            cluster_id++;
            points_[i].cluster_id = cluster_id;
            
            // Расширяем кластер
            std::queue<size_t> queue;
            for (size_t idx : neighbors) {
                queue.push(idx);
            }
            
            while (!queue.empty()) {
                size_t idx = queue.front();
                queue.pop();
                
                if (!points_[idx].visited) {
                    points_[idx].visited = true;
                    auto new_neighbors = find_neighbors(idx);
                    
                    if (new_neighbors.size() >= static_cast<size_t>(min_pts_)) {
                        for (size_t n_idx : new_neighbors) {
                            if (!points_[n_idx].visited) {
                                queue.push(n_idx);
                            }
                        }
                    }
                }
                
                if (points_[idx].cluster_id == -1 || points_[idx].cluster_id == 0) {
                    points_[idx].cluster_id = cluster_id;
                }
            }
        }
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "DBSCAN complete: " + 
                  std::to_string(cluster_id) + " clusters, " +
                  std::to_string(total_noise_points_) + " noise points");
}

TargetCluster DBSCANClusterer::create_cluster_from_points(
    const std::vector<size_t>& point_indices) {
    
    TargetCluster cluster;
    
    if (point_indices.empty()) return cluster;
    
    // Сортируем точки по времени
    std::vector<size_t> sorted_indices = point_indices;
    std::sort(sorted_indices.begin(), sorted_indices.end(),
        [this](size_t a, size_t b) {
            return points_[a].scan_index < points_[b].scan_index;
        });
    
    // Собираем сканы из точек
    std::map<uint16_t, ScanReplies> scan_map;
    
    for (size_t idx : sorted_indices) {
        const Point& p = points_[idx];
        
        auto it = scan_map.find(p.azimuth);
        if (it == scan_map.end()) {
            ScanReplies scan(p.azimuth, 0);
            scan_map[p.azimuth] = scan;
            it = scan_map.find(p.azimuth);
        }
        
        if (p.is_rbs && p.rbs_reply) {
            it->second.rbs_replies.push_back(*p.rbs_reply);
        } else if (!p.is_rbs && p.uvd_reply) {
            it->second.uvd_replies.push_back(*p.uvd_reply);
        }
    }
    
    // Добавляем сканы в кластер
    for (auto& [azimuth, scan] : scan_map) {
        cluster.add_scan(scan, total_scans_processed_);
    }
    
    // Устанавливаем время
    if (!cluster.scans.empty()) {
        cluster.first_timestamp = total_scans_processed_;
        cluster.last_timestamp = total_scans_processed_;
        cluster.last_reply_azimuth = cluster.scans.back().azimuth;
        cluster.last_processed_azimuth = cluster.scans.back().azimuth;
    }
    
    return cluster;
}

void DBSCANClusterer::update_active_clusters() {
    // Группируем точки по ID кластера
    std::map<int, std::vector<size_t>> cluster_points;
    
    for (size_t i = 0; i < points_.size(); ++i) {
        int cluster_id = points_[i].cluster_id;
        if (cluster_id > 0) {  // Не шум
            cluster_points[cluster_id].push_back(i);
        }
    }
    
    // Создаем новые кластеры
    for (auto& [cluster_id, indices] : cluster_points) {
        if (indices.size() < static_cast<size_t>(min_pts_)) continue;
        
        // Проверяем, существует ли уже этот кластер
        bool found = false;
        for (auto& cluster : active_clusters_) {
            if (cluster.scans.size() == indices.size()) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            auto cluster = create_cluster_from_points(indices);
            if (!cluster.scans.empty()) {
                active_clusters_.push_back(std::move(cluster));
                total_clusters_formed_++;
                VRL_LOG_TRACE(modules::CLUSTER, "Created new cluster with " + 
                              std::to_string(indices.size()) + " points");
            }
        }
    }
}

void DBSCANClusterer::complete_expired_clusters() {
    auto it = active_clusters_.begin();
    int completed = 0;
    
    while (it != active_clusters_.end()) {
        bool active = it->is_active(current_azimuth_, max_gap_azimuth_);
        
        if (!active) {
            VRL_LOG_DEBUG(modules::CLUSTER, "Cluster completed: azimuth_span=" + 
                          std::to_string(it->azimuth_span()) + 
                          ", range_span=" + std::to_string(it->range_span()) +
                          ", scans=" + std::to_string(it->scans.size()));
            completed_clusters_.push_back(std::move(*it));
            it = active_clusters_.erase(it);
            completed++;
            total_clusters_completed_++;
        } else {
            ++it;
        }
    }
    
    if (completed > 0) {
        VRL_LOG_DEBUG(modules::CLUSTER, "Completed " + std::to_string(completed) + 
                      " clusters");
    }
}

std::vector<TargetCluster> DBSCANClusterer::get_completed_clusters() {
    auto result = std::move(completed_clusters_);
    completed_clusters_.clear();
    return result;
}

const std::vector<TargetCluster>& DBSCANClusterer::get_active_clusters() const {
    return active_clusters_;
}

std::vector<TargetCluster> DBSCANClusterer::finish_all() {
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

void DBSCANClusterer::reset() {
    size_t old_active = active_clusters_.size();
    size_t old_completed = completed_clusters_.size();
    
    points_.clear();
    active_clusters_.clear();
    completed_clusters_.clear();
    
    total_scans_processed_ = 0;
    total_clusters_formed_ = 0;
    total_clusters_completed_ = 0;
    total_noise_points_ = 0;
    current_azimuth_ = 0;
    
    VRL_LOG_DEBUG(modules::CLUSTER, "Reset: cleared " + std::to_string(old_active) + 
                  " active and " + std::to_string(old_completed) + " completed clusters");
}

void DBSCANClusterer::get_stats(size_t& active, size_t& completed) const {
    active = active_clusters_.size();
    completed = completed_clusters_.size();
}

std::unique_ptr<IClusterer> DBSCANClusterer::clone() const {
    auto clone = std::make_unique<DBSCANClusterer>(eps_range_, eps_azimuth_deg_, 
                                                    min_pts_, range_bin_m_);
    clone->max_gap_azimuth_ = max_gap_azimuth_;
    clone->range_window_ = range_window_;
    return clone;
}

void DBSCANClusterer::set_param(const std::string& key, double value) {
    if (key == "eps_range") {
        eps_range_ = value;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: eps_range=" + std::to_string(eps_range_) + "m");
    } else if (key == "eps_azimuth_deg") {
        eps_azimuth_deg_ = value;
        eps_azimuth_rad_ = eps_azimuth_deg_ * DEG_TO_RAD;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: eps_azimuth_deg=" + std::to_string(eps_azimuth_deg_) + "°");
    } else if (key == "range_bin") {
        range_bin_m_ = value;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: range_bin=" + std::to_string(range_bin_m_) + "m");
    } else if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = static_cast<int>(value);
    } else if (key == "range_window") {
        range_window_ = static_cast<int>(value);
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown double parameter: " + key);
    }
}

void DBSCANClusterer::set_param(const std::string& key, int value) {
    if (key == "min_pts") {
        min_pts_ = value;
        VRL_LOG_DEBUG(modules::CLUSTER, "set_param: min_pts=" + std::to_string(min_pts_));
    } else if (key == "max_gap_azimuth") {
        max_gap_azimuth_ = value;
    } else if (key == "range_window") {
        range_window_ = value;
    } else {
        VRL_LOG_WARN(modules::CLUSTER, "Unknown int parameter: " + key);
    }
}

} // namespace radar
} // namespace vrl
