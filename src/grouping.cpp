// file: src/grouping.cpp
#include "radar/grouping.h"
#include "radar/utils.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <set>

namespace radar {

// ---- ReplyCluster implementation ----

void ReplyCluster::add_reply(const RawReply& reply) {
    if (replies.empty()) {
        start_azimuth = reply.azimuth;
        end_azimuth = reply.azimuth;
        scan_number = reply.scan_number;
    } else {
        // Обновляем конечный азимут
        end_azimuth = reply.azimuth;
    }
    
    replies.push_back(reply);
    
    // Пересчитываем статистику
    double sum_az = 0.0, sum_range = 0.0;
    double min_az = 4096.0, max_az = 0.0;
    double min_range = 1e9, max_range = 0.0;
    
    for (const auto& r : replies) {
        sum_az += r.azimuth;
        sum_range += r.range;
        
        double az = r.azimuth;
        min_az = std::min(min_az, az);
        max_az = std::max(max_az, az);
        
        min_range = std::min(min_range, static_cast<double>(r.range));
        max_range = std::max(max_range, static_cast<double>(r.range));
    }
    
    // Проверяем, не перескочили ли через 0/4096
    if (max_az - min_az > 2048) {
        // Есть переход через 0, корректируем
        azimuth_span = (4096 - max_az) + min_az;
    } else {
        azimuth_span = max_az - min_az;
    }
    
    range_span = max_range - min_range;
    avg_azimuth = sum_az / replies.size();
    avg_range = sum_range / replies.size();
}

void ReplyCluster::detect_overlaps(const RadarConfig& cfg) {
    overlapping_pairs.clear();
    has_overlap = false;
    
    size_t n = replies.size();
    if (n < 2) return;
    
    // Проверяем все пары ответов в кластере
    for (size_t i = 0; i < n - 1; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            const auto& r1 = replies[i];
            const auto& r2 = replies[j];
            
            // Проверяем потенциальное перекрытие
            bool overlap = radar::utils::is_potential_overlap(
                r1.azimuth, r1.range,
                r2.azimuth, r2.range,
                cfg
            );
            
            if (overlap) {
                overlapping_pairs.emplace_back(i, j);
                has_overlap = true;
            }
        }
    }
}

// ---- ScanPostProcessor implementation ----

ScanPostProcessor::ScanPostProcessor(const RadarConfig& cfg) : config_(cfg) {}

void ScanPostProcessor::add_reply(const RBSReply& reply, uint32_t scan_number, uint32_t timestamp_ms) {
    if (scan_number != current_scan_number_) {
        if (!current_scan_.empty()) {
            current_scan_.clear();
        }
        current_scan_number_ = scan_number;
    }
    current_scan_.emplace_back(reply, scan_number, timestamp_ms);
}

void ScanPostProcessor::add_reply(const UVDReply& reply, uint32_t scan_number, uint32_t timestamp_ms) {
    if (scan_number != current_scan_number_) {
        if (!current_scan_.empty()) {
            current_scan_.clear();
        }
        current_scan_number_ = scan_number;
    }
    current_scan_.emplace_back(reply, scan_number, timestamp_ms);
}

std::vector<size_t> ScanPostProcessor::find_neighbors(size_t idx, const std::vector<RawReply>& replies) const {
    std::vector<size_t> neighbors;
    const auto& reply = replies[idx];
    
    // Используем те же параметры, что и в is_potential_overlap, но с небольшим запасом
    const uint16_t AZ_WINDOW = config_.max_azimuth_diff_for_overlap * 2;  // Увеличиваем в 2 раза для кластеризации
    const uint16_t RANGE_WINDOW = config_.max_range_diff_for_overlap * 2;
    
    for (size_t j = 0; j < replies.size(); ++j) {
        if (j == idx) continue;
        
        const auto& other = replies[j];
        
        // Вычисляем разницу по азимуту с учётом цикличности
        int16_t az_diff = std::abs(static_cast<int16_t>(reply.azimuth - other.azimuth));
        az_diff = std::min(az_diff, static_cast<int16_t>(4096 - az_diff));
        
        uint16_t range_diff = std::abs(static_cast<int16_t>(reply.range - other.range));
        
        if (az_diff <= AZ_WINDOW && range_diff <= RANGE_WINDOW) {
            neighbors.push_back(j);
        }
    }
    
    return neighbors;
}

std::vector<std::vector<size_t>> ScanPostProcessor::cluster_replies(const std::vector<RawReply>& replies) const {
    std::vector<std::vector<size_t>> clusters;
    std::vector<bool> visited(replies.size(), false);
    
    for (size_t i = 0; i < replies.size(); ++i) {
        if (visited[i]) continue;
        
        // BFS для поиска связной компоненты
        std::queue<size_t> q;
        q.push(i);
        visited[i] = true;
        
        std::vector<size_t> cluster;
        
        while (!q.empty()) {
            size_t current = q.front();
            q.pop();
            cluster.push_back(current);
            
            auto neighbors = find_neighbors(current, replies);
            for (size_t neighbor : neighbors) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }
        
        if (cluster.size() >= 1) {
            // Сортируем по азимуту
            std::sort(cluster.begin(), cluster.end(),
                [&replies](size_t a, size_t b) {
                    return replies[a].azimuth < replies[b].azimuth;
                });
            clusters.push_back(std::move(cluster));
        }
    }
    
    return clusters;
}

std::vector<ReplyCluster> ScanPostProcessor::finish_scan(uint32_t scan_number) {
    std::vector<ReplyCluster> result;
    
    if (current_scan_.empty() || scan_number != current_scan_number_) {
        return result;
    }
    
    // Сортируем ответы по азимуту
    std::sort(current_scan_.begin(), current_scan_.end(),
        [](const RawReply& a, const RawReply& b) {
            return a.azimuth < b.azimuth;
        });
    
    // Кластеризация
    auto clusters_idx = cluster_replies(current_scan_);
    
    // Формируем результат
    for (const auto& idx_vec : clusters_idx) {
        ReplyCluster cluster;
        for (size_t idx : idx_vec) {
            cluster.add_reply(current_scan_[idx]);
        }
        
        // Проверяем перекрытия внутри кластера
        cluster.detect_overlaps(config_);
        
        result.push_back(std::move(cluster));
    }
    
    // Очищаем для следующего оборота
    current_scan_.clear();
    
    return result;
}

void ScanPostProcessor::reset() {
    current_scan_.clear();
    current_scan_number_ = 0;
}

} // namespace radar