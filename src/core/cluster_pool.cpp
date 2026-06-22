// src/core/cluster_pool.cpp
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

ClusterPool& ClusterPool::instance() {
    static ClusterPool instance;
    return instance;
}

// ============================================================================
// СОЗДАНИЕ
// ============================================================================

uint64_t ClusterPool::create_cluster() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    uint64_t id = next_id_++;
    auto cluster = std::make_unique<Cluster>();
    
    clusters_[id] = std::move(cluster);
    active_ids_.push_back(id);
    metadata_[id] = Metadata{};
    
    std::cout << "[DEBUG] create_cluster: id=" << id 
              << ", active_ids_.size()=" << active_ids_.size() << std::endl;
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster created, id=" + std::to_string(id));
    
    return id;
}

// ============================================================================
// ЗАКРЫТИЕ
// ============================================================================

void ClusterPool::close_cluster(uint64_t id, int sector) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "[DEBUG] close_cluster: id=" << id 
              << ", sector=" << sector 
              << ", active_ids_.size() before=" << active_ids_.size() << std::endl;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    auto& cluster = clusters_[id];
    cluster->close();
    
    metadata_[id].state = ClusterState::CLOSED;
    metadata_[id].sector_index = sector;
    metadata_[id].close_azimuth = cluster->get_last_azimuth();
    
    remove_from_active(id);
    
    if (sector >= 0 && sector < NUM_SECTORS) {
        closed_by_sector_[sector].push_back(id);
        std::cout << "[DEBUG] close_cluster: added to closed_by_sector[" << sector 
                  << "], size=" << closed_by_sector_[sector].size() << std::endl;
    } else {
        VRL_LOG_WARN(modules::CORE, "Invalid sector for cluster: " + std::to_string(sector));
    }
}

// ============================================================================
// ШИРОКИЙ КЛАСТЕР
// ============================================================================

void ClusterPool::mark_as_wide(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "[DEBUG] mark_as_wide: id=" << id << std::endl;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    // НЕ УДАЛЯЕМ из активных! Широкий кластер всё ещё активен
    metadata_[id].state = ClusterState::WIDE;
    wide_ids_.push_back(id);
    
    std::cout << "[DEBUG] mark_as_wide: wide_ids_.size()=" << wide_ids_.size() << std::endl;
}

// ============================================================================
// ВЗЯТИЕ КЛАСТЕРОВ ДЛЯ ОБРАБОТКИ
// ============================================================================

std::vector<uint64_t> ClusterPool::take_closed_clusters(int sector) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint64_t> result;
    
    if (sector < 0 || sector >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::CORE, "Invalid sector index: " + std::to_string(sector));
        return result;
    }
    
    std::cout << "[DEBUG] take_closed_clusters: sector=" << sector 
              << ", closed_by_sector_[sector].size()=" << closed_by_sector_[sector].size() << std::endl;
    
    result = std::move(closed_by_sector_[sector]);
    closed_by_sector_[sector].clear();
    
    std::cout << "[DEBUG] take_closed_clusters: result.size()=" << result.size() << std::endl;
    
    for (uint64_t id : result) {
        std::cout << "[DEBUG] take_closed_clusters: processing id=" << id << std::endl;
        
        auto it = metadata_.find(id);
        if (it != metadata_.end()) {
            std::cout << "[DEBUG] take_closed_clusters: found in metadata, state=" 
                      << static_cast<int>(it->second.state) << std::endl;
            it->second.state = ClusterState::DELAYED;
            it->second.delayed_since = static_cast<uint32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count()
            );
            std::cout << "[DEBUG] take_closed_clusters: updated state to DELAYED" << std::endl;
        } else {
            std::cout << "[DEBUG] take_closed_clusters: NOT found in metadata!" << std::endl;
        }
        delayed_ids_.push_back(id);
    }
    
    std::cout << "[DEBUG] take_closed_clusters: delayed_ids_.size()=" << delayed_ids_.size() << std::endl;
    
    return result;
}

std::vector<uint64_t> ClusterPool::take_wide_clusters() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint64_t> result = std::move(wide_ids_);
    wide_ids_.clear();
    
    std::cout << "[DEBUG] take_wide_clusters: took " << result.size() << " wide clusters" << std::endl;
    
    return result;
}

// ============================================================================
// УДАЛЕНИЕ КЛАСТЕРА
// ============================================================================

void ClusterPool::remove_cluster(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::cout << "[DEBUG] remove_cluster: id=" << id << std::endl;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    // Удаляем из всех списков
    remove_from_active(id);
    remove_from_closed(id);
    remove_from_wide(id);
    remove_from_delayed(id);
    
    // Удаляем метаданные
    metadata_.erase(id);
    
    // Удаляем кластер
    clusters_.erase(id);
    
    std::cout << "[DEBUG] remove_cluster: done, clusters_.size()=" << clusters_.size() << std::endl;
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ДЛЯ УДАЛЕНИЯ ИЗ СПИСКОВ
// ============================================================================

void ClusterPool::remove_from_active(uint64_t id) {
    auto it = std::find(active_ids_.begin(), active_ids_.end(), id);
    if (it != active_ids_.end()) {
        active_ids_.erase(it);
        std::cout << "[DEBUG] remove_from_active: removed id=" << id << std::endl;
    } else {
        std::cout << "[DEBUG] remove_from_active: id=" << id << " not found in active_ids_" << std::endl;
    }
}

void ClusterPool::remove_from_closed(uint64_t id) {
    for (auto& vec : closed_by_sector_) {
        auto it = std::find(vec.begin(), vec.end(), id);
        if (it != vec.end()) {
            vec.erase(it);
            std::cout << "[DEBUG] remove_from_closed: removed id=" << id << std::endl;
            return;
        }
    }
    std::cout << "[DEBUG] remove_from_closed: id=" << id << " not found in any closed_by_sector_" << std::endl;
}

void ClusterPool::remove_from_wide(uint64_t id) {
    auto it = std::find(wide_ids_.begin(), wide_ids_.end(), id);
    if (it != wide_ids_.end()) {
        wide_ids_.erase(it);
        std::cout << "[DEBUG] remove_from_wide: removed id=" << id << std::endl;
    } else {
        std::cout << "[DEBUG] remove_from_wide: id=" << id << " not found in wide_ids_" << std::endl;
    }
}

void ClusterPool::remove_from_delayed(uint64_t id) {
    auto it = std::find(delayed_ids_.begin(), delayed_ids_.end(), id);
    if (it != delayed_ids_.end()) {
        delayed_ids_.erase(it);
        std::cout << "[DEBUG] remove_from_delayed: removed id=" << id << std::endl;
    } else {
        std::cout << "[DEBUG] remove_from_delayed: id=" << id << " not found in delayed_ids_" << std::endl;
    }
}

// ============================================================================
// ОЧИСТКА ЗАДЕРЖАННЫХ КЛАСТЕРОВ
// ============================================================================

size_t ClusterPool::cleanup_delayed_clusters(uint32_t current_revolution, double max_delay) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    size_t cleaned = 0;
    auto it = delayed_ids_.begin();
    
    while (it != delayed_ids_.end()) {
        uint64_t id = *it;
        
        // Проверяем, существует ли ещё кластер
        if (clusters_.find(id) == clusters_.end()) {
            it = delayed_ids_.erase(it);
            cleaned++;
            continue;
        }
        
        // Проверяем, не истекло ли время
        bool should_remove = false;
        auto meta_it = metadata_.find(id);
        if (meta_it != metadata_.end()) {
            if (max_delay == 0.0) {
                should_remove = true;
            } else {
                uint32_t age = current_revolution - meta_it->second.delayed_since;
                if (age >= static_cast<uint32_t>(max_delay * 4096)) {
                    should_remove = true;
                }
            }
        } else {
            should_remove = true;
        }
        
        if (should_remove) {
            clusters_.erase(id);
            metadata_.erase(id);
            it = delayed_ids_.erase(it);
            cleaned++;
        } else {
            ++it;
        }
    }
    
    if (cleaned > 0) {
        VRL_LOG_DEBUG(modules::CORE, "Cleaned " + std::to_string(cleaned) + " delayed clusters");
    }
    
    return cleaned;
}

// ============================================================================
// ПОЛУЧЕНИЕ КЛАСТЕРОВ
// ============================================================================

Cluster* ClusterPool::get_cluster(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = clusters_.find(id);
    if (it != clusters_.end()) {
        return it->second.get();
    }
    
    VRL_LOG_ERROR(modules::CORE, "Cluster not found: " + std::to_string(id));
    return nullptr;
}

const Cluster* ClusterPool::get_cluster(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = clusters_.find(id);
    if (it != clusters_.end()) {
        return it->second.get();
    }
    
    VRL_LOG_ERROR(modules::CORE, "Cluster not found: " + std::to_string(id));
    return nullptr;
}

std::vector<Cluster*> ClusterPool::get_all_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Cluster*> result;
    result.reserve(clusters_.size());
    
    for (const auto& [id, cluster] : clusters_) {
        if (cluster) {
            result.push_back(cluster.get());
        }
    }
    
    return result;
}

std::vector<uint64_t> ClusterPool::get_all_ids() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<uint64_t> result;
    result.reserve(clusters_.size());
    
    for (const auto& [id, cluster] : clusters_) {
        if (cluster) {
            result.push_back(id);
        }
    }
    
    return result;
}

// ============================================================================
// ПРОВЕРКИ
// ============================================================================

bool ClusterPool::has_closed_clusters(int sector) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (sector < 0 || sector >= NUM_SECTORS) {
        return false;
    }
    
    return !closed_by_sector_[sector].empty();
}

bool ClusterPool::has_wide_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !wide_ids_.empty();
}

bool ClusterPool::has_delayed_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !delayed_ids_.empty();
}

bool ClusterPool::exists(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clusters_.find(id) != clusters_.end();
}

bool ClusterPool::is_valid_id(uint64_t id) const {
    return clusters_.find(id) != clusters_.end() && clusters_.at(id) != nullptr;
}

// ============================================================================
// ПОДСЧЁТ
// ============================================================================

size_t ClusterPool::count_active_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto& [id, meta] : metadata_) {
        if (meta.state == ClusterState::ACTIVE || meta.state == ClusterState::WIDE) {
            count++;
        }
    }
    return count;
}

size_t ClusterPool::count_closed_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    std::cout << "[DEBUG] count_closed_clusters: metadata_.size()=" << metadata_.size() << std::endl;
    for (const auto& [id, meta] : metadata_) {
        std::cout << "[DEBUG] count_closed_clusters: id=" << id 
                  << ", state=" << static_cast<int>(meta.state) << std::endl;
        if (meta.state == ClusterState::CLOSED) {
            count++;
        }
    }
    std::cout << "[DEBUG] count_closed_clusters: count=" << count << std::endl;
    return count;
}

size_t ClusterPool::count_delayed_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto& [id, meta] : metadata_) {
        if (meta.state == ClusterState::DELAYED) {
            count++;
        }
    }
    return count;
}

size_t ClusterPool::count_wide_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto& [id, meta] : metadata_) {
        if (meta.state == ClusterState::WIDE) {
            count++;
        }
    }
    return count;
}

size_t ClusterPool::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clusters_.size();
}

bool ClusterPool::is_empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clusters_.empty();
}

// ============================================================================
// УПРАВЛЕНИЕ
// ============================================================================

void ClusterPool::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    clusters_.clear();
    active_ids_.clear();
    for (auto& vec : closed_by_sector_) {
        vec.clear();
    }
    wide_ids_.clear();
    delayed_ids_.clear();
    metadata_.clear();
    next_id_ = 1;
    
    VRL_LOG_INFO(modules::CORE, "ClusterPool cleared");
}

// ============================================================================
// СТАТИСТИКА
// ============================================================================

ClusterPool::Stats ClusterPool::get_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Stats stats;
    stats.total_clusters = clusters_.size();
    stats.active_clusters = active_ids_.size();
    
    for (const auto& vec : closed_by_sector_) {
        stats.closed_clusters += vec.size();
    }
    stats.wide_clusters = wide_ids_.size();
    stats.delayed_clusters = delayed_ids_.size();
    
    for (int i = 0; i < NUM_SECTORS; ++i) {
        stats.clusters_by_sector[i] = closed_by_sector_[i].size();
    }
    
    return stats;
}

} // namespace radar
} // namespace vrl
