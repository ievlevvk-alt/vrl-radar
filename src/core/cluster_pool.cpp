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
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

void ClusterPool::init(size_t max_clusters) {
    if (initialized_) {
        VRL_LOG_WARN(modules::CORE, "ClusterPool already initialized");
        return;
    }
    
    if (max_clusters == 0) {
        VRL_LOG_ERROR(modules::CORE, "max_clusters must be > 0");
        return;
    }
    
    max_clusters_ = max_clusters;
    clusters_.resize(max_clusters_);
    metadata_.resize(max_clusters_);
    
    for (size_t i = 0; i < max_clusters_; ++i) {
        clusters_[i].set_id(i + 1);
        clusters_[i].enable_debug(debug_enabled_);
        metadata_[i] = Metadata{};
    }
    
    next_slot_ = 0;
    initialized_ = true;
    
    VRL_LOG_INFO(modules::CORE, "ClusterPool initialized with " + std::to_string(max_clusters_) + " slots");
}

// ============================================================================
// СОЗДАНИЕ
// ============================================================================

uint64_t ClusterPool::create_cluster() {
    auto debug = debug_enabled_;
    
    if (!initialized_) {
        VRL_LOG_ERROR(modules::CORE, "ClusterPool not initialized");
        return 0;
    }
    
    size_t slot = next_slot_++;
    if (slot >= max_clusters_) {
        slot = 0;
        // Циклическое переполнение - потребитель должен был завершить работу с кластером
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            uint64_t old_id = clusters_[slot].get_id();
            std::cout << "[ClusterPool] create_cluster: OVERWRITE slot=" << slot 
                      << ", old_id=" << old_id << " (consumer must have finished with it)" << std::endl;
        }
#endif
        // НЕ удаляем old_id из списков - это ответственность потребителя
    }
    
    uint64_t id = static_cast<uint64_t>(slot + 1);
    
    clusters_[slot] = Cluster();
    clusters_[slot].set_id(id);
    clusters_[slot].enable_debug(debug_enabled_);
    metadata_[slot] = Metadata{};
    metadata_[slot].state = ClusterState::ACTIVE;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] create_cluster: id=" << id << ", slot=" << slot << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster created, id=" + std::to_string(id) + ", slot=" + std::to_string(slot));
    
    return id;
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ
// ============================================================================

bool ClusterPool::is_valid_id(uint64_t id) const {
    if (id == 0) return false;
    size_t slot = static_cast<size_t>(id - 1);
    return slot < max_clusters_;
}

// ============================================================================
// ПОЛУЧЕНИЕ КЛАСТЕРА
// ============================================================================

Cluster* ClusterPool::get_cluster(uint64_t id) {
    if (!is_valid_id(id)) {
        return nullptr;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    if (slot >= max_clusters_) {
        return nullptr;
    }
    
    if (clusters_[slot].get_id() != id) {
        return nullptr;
    }
    
    return &clusters_[slot];
}

const Cluster* ClusterPool::get_cluster(uint64_t id) const {
    if (!is_valid_id(id)) {
        return nullptr;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    if (slot >= max_clusters_) {
        return nullptr;
    }
    
    if (clusters_[slot].get_id() != id) {
        return nullptr;
    }
    
    return &clusters_[slot];
}

std::vector<uint64_t> ClusterPool::get_closed_clusters() const {
    std::vector<uint64_t> result;
    for (const auto& vec : closed_by_sector_) {
        result.insert(result.end(), vec.begin(), vec.end());
    }
    return result;
}

void ClusterPool::add_to_delayed(uint64_t id, int sector) {
    auto debug = debug_enabled_;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    if (sector < 0 || sector >= NUM_SECTORS) {
        VRL_LOG_ERROR(modules::CORE, "Invalid sector: " + std::to_string(sector));
        return;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    
    // Удаляем из закрытых списков
    remove_from_closed(id);
    
    // Добавляем в задержанные
    metadata_[slot].state = ClusterState::DELAYED;
    metadata_[slot].sector_index = sector;
    delayed_ids_.push_back(id);
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] add_to_delayed: id=" << id 
                  << ", sector=" << sector << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster " + std::to_string(id) + 
                  " added to delayed sector " + std::to_string(sector));
}


std::vector<uint64_t> ClusterPool::take_delayed_clusters(int sector) {
    auto debug = debug_enabled_;
    
    std::vector<uint64_t> result;
    
    if (sector < 0 || sector >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::CORE, "Invalid sector index: " + std::to_string(sector));
        return result;
    }
    
    // Собираем все задержанные кластеры из указанного сектора
    // Задержанные кластеры хранятся в delayed_ids_ с привязкой к сектору
    // Нужно пройти по delayed_ids_ и выбрать те, у которых sector_index == sector
    
    auto it = delayed_ids_.begin();
    while (it != delayed_ids_.end()) {
        uint64_t id = *it;
        size_t slot = static_cast<size_t>(id - 1);
        
        if (slot < max_clusters_ && metadata_[slot].sector_index == sector) {
            result.push_back(id);
            it = delayed_ids_.erase(it);
        } else {
            ++it;
        }
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] take_delayed_clusters: sector=" << sector 
                  << ", count=" << result.size() << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Took " + std::to_string(result.size()) + 
                  " delayed clusters from sector " + std::to_string(sector));
    
    return result;
}

bool ClusterPool::exists(uint64_t id) const {
    if (!is_valid_id(id)) {
        return false;
    }
    
    // Проверяем наличие в активных
    auto it_active = std::find(active_ids_.begin(), active_ids_.end(), id);
    if (it_active != active_ids_.end()) {
        return true;
    }
    
    // Проверяем наличие в закрытых (по всем секторам)
    for (const auto& vec : closed_by_sector_) {
        auto it_closed = std::find(vec.begin(), vec.end(), id);
        if (it_closed != vec.end()) {
            return true;
        }
    }
    
    // Проверяем наличие в широких
    auto it_wide = std::find(wide_ids_.begin(), wide_ids_.end(), id);
    if (it_wide != wide_ids_.end()) {
        return true;
    }
    
    // Проверяем наличие в задержанных
    auto it_delayed = std::find(delayed_ids_.begin(), delayed_ids_.end(), id);
    if (it_delayed != delayed_ids_.end()) {
        return true;
    }
    
    return false;
}

std::vector<Cluster*> ClusterPool::get_all_clusters() const {
    std::vector<Cluster*> result;
    result.reserve(max_clusters_);
    for (size_t i = 0; i < max_clusters_; ++i) {
        result.push_back(const_cast<Cluster*>(&clusters_[i]));
    }
    return result;
}

std::vector<Cluster*> ClusterPool::get_active_clusters() const {
    std::vector<Cluster*> result;
    result.reserve(active_ids_.size());
    for (uint64_t id : active_ids_) {
        const Cluster* const_cluster = get_cluster(id);
        if (const_cluster && !const_cluster->is_closed()) {
            result.push_back(const_cast<Cluster*>(const_cluster));
        }
    }
    return result;
}


std::vector<uint64_t> ClusterPool::get_all_ids() const {
    std::vector<uint64_t> result;
    result.reserve(active_ids_.size());
    for (uint64_t id : active_ids_) {
        result.push_back(id);
    }
    return result;
}


void ClusterPool::merge_clusters(uint64_t keep_id, uint64_t remove_id) {
    auto debug = debug_enabled_;
    
    if (!is_valid_id(keep_id) || !is_valid_id(remove_id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id for merge");
        return;
    }
    
    if (keep_id == remove_id) {
        VRL_LOG_WARN(modules::CORE, "Attempt to merge cluster with itself");
        return;
    }
    
    size_t keep_slot = static_cast<size_t>(keep_id - 1);
    size_t remove_slot = static_cast<size_t>(remove_id - 1);
    
    auto& keep_cluster = clusters_[keep_slot];
    auto& remove_cluster = clusters_[remove_slot];
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] merge_clusters: keep=" << keep_id 
                  << ", remove=" << remove_id << std::endl;
    }
#endif
    
    // Переносим все точки из remove_cluster в keep_cluster
    for (size_t idx : remove_cluster.get_indices()) {
        keep_cluster.add_point(idx);
    }
    
    // Удаляем remove_id из всех списков
    remove_from_active(remove_id);
    
    // Сбрасываем удалённый кластер
    remove_cluster = Cluster();
    remove_cluster.set_id(remove_id);
    remove_cluster.enable_debug(debug_enabled_);
    metadata_[remove_slot] = Metadata{};
    
    VRL_LOG_DEBUG(modules::CORE, "Clusters merged: " + std::to_string(keep_id) + 
                  " kept, " + std::to_string(remove_id) + " removed");
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ДЛЯ УДАЛЕНИЯ ИЗ СПИСКОВ
// ============================================================================

void ClusterPool::remove_from_active(uint64_t id) {
    auto it = std::find(active_ids_.begin(), active_ids_.end(), id);
    if (it != active_ids_.end()) {
        active_ids_.erase(it);
    }
}

void ClusterPool::remove_from_closed(uint64_t id) {
    for (auto& vec : closed_by_sector_) {
        auto it = std::find(vec.begin(), vec.end(), id);
        if (it != vec.end()) {
            vec.erase(it);
            return;
        }
    }
}

void ClusterPool::remove_from_wide(uint64_t id) {
    auto it = std::find(wide_ids_.begin(), wide_ids_.end(), id);
    if (it != wide_ids_.end()) {
        wide_ids_.erase(it);
    }
}

void ClusterPool::remove_from_delayed(uint64_t id) {
    auto it = std::find(delayed_ids_.begin(), delayed_ids_.end(), id);
    if (it != delayed_ids_.end()) {
        delayed_ids_.erase(it);
    }
}

// ============================================================================
// ЗАКРЫТИЕ
// ============================================================================

void ClusterPool::close_cluster(uint64_t id, int sector) {
    auto debug = debug_enabled_;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    auto& cluster = clusters_[slot];
    auto& meta = metadata_[slot];
    
    cluster.close();
    
    if (sector >= 0 && sector < NUM_SECTORS) {
        meta.state = ClusterState::CLOSED;
        meta.sector_index = sector;
        meta.close_azimuth = cluster.get_last_azimuth();
        
        remove_from_active(id);
        remove_from_wide(id);
        closed_by_sector_[sector].push_back(id);
        
#ifdef CMAKE_BUILD_TYPE_DEBUG
        if (debug) {
            std::cout << "[ClusterPool] close_cluster: id=" << id << ", sector=" << sector 
                      << ", size=" << cluster.size() << std::endl;
        }
#endif
        
        VRL_LOG_DEBUG(modules::CORE, "Cluster " + std::to_string(id) + 
                      " closed in sector " + std::to_string(sector));
    } else {
        VRL_LOG_WARN(modules::CORE, "Invalid sector for cluster: " + std::to_string(sector));
        meta.state = ClusterState::PROCESSED;
        remove_from_active(id);
        remove_from_wide(id);
    }
}

void ClusterPool::add_to_active(uint64_t id) {
    auto debug = debug_enabled_;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    // Проверяем, что кластер уже не в списке
    auto it = std::find(active_ids_.begin(), active_ids_.end(), id);
    if (it != active_ids_.end()) {
        return;
    }
    
    active_ids_.push_back(id);
    metadata_[id - 1].state = ClusterState::ACTIVE;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] add_to_active: id=" << id 
                  << ", active_count=" << active_ids_.size() << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster " + std::to_string(id) + " added to active list");
}

// ============================================================================
// ШИРОКИЕ КЛАСТЕРЫ
// ============================================================================

void ClusterPool::add_to_wide(uint64_t id) {
    auto debug = debug_enabled_;
    
    if (!is_valid_id(id)) {
        VRL_LOG_ERROR(modules::CORE, "Invalid cluster id: " + std::to_string(id));
        return;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    metadata_[slot].state = ClusterState::WIDE;
    wide_ids_.push_back(id);
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] add_to_wide: id=" << id << ", wide_count=" << wide_ids_.size() << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster " + std::to_string(id) + " added to wide list");
}

std::vector<uint64_t> ClusterPool::take_wide_clusters() {
    auto debug = debug_enabled_;
    
    std::vector<uint64_t> result = std::move(wide_ids_);
    wide_ids_.clear();
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] take_wide_clusters: count=" << result.size() << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Took " + std::to_string(result.size()) + " wide clusters");
    
    return result;
}

// ============================================================================
// ПОЛУЧЕНИЕ ЗАКРЫТЫХ КЛАСТЕРОВ
// ============================================================================

std::vector<uint64_t> ClusterPool::take_closed_clusters(int sector) {
    auto debug = debug_enabled_;
    
    std::vector<uint64_t> result;
    
    if (sector < 0 || sector >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::CORE, "Invalid sector index: " + std::to_string(sector));
        return result;
    }
    
    result = std::move(closed_by_sector_[sector]);
    closed_by_sector_[sector].clear();
    
    for (uint64_t id : result) {
        size_t slot = static_cast<size_t>(id - 1);
        if (slot < max_clusters_) {
            metadata_[slot].state = ClusterState::DELAYED;
            metadata_[slot].delayed_since = 0;
        }
        delayed_ids_.push_back(id);
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] take_closed_clusters: sector=" << sector 
                  << ", count=" << result.size() << std::endl;
    }
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Took " + std::to_string(result.size()) + 
                  " clusters from sector " + std::to_string(sector));
    
    return result;
}

// ============================================================================
// ОЧИСТКА ЗАДЕРЖАННЫХ
// ============================================================================

size_t ClusterPool::cleanup_delayed_clusters(uint32_t current_revolution, double max_delay) {
    auto debug = debug_enabled_;
    
    size_t cleaned = 0;
    auto it = delayed_ids_.begin();
    
    while (it != delayed_ids_.end()) {
        uint64_t id = *it;
        size_t slot = static_cast<size_t>(id - 1);
        
        if (slot >= max_clusters_) {
            it = delayed_ids_.erase(it);
            cleaned++;
            continue;
        }
        
        bool should_remove = false;
        if (max_delay == 0.0) {
            should_remove = true;
        } else {
            uint32_t age = current_revolution - metadata_[slot].delayed_since;
            if (age >= static_cast<uint32_t>(max_delay * 4096)) {
                should_remove = true;
            }
        }
        
        if (should_remove) {
            clusters_[slot] = Cluster();
            clusters_[slot].set_id(id);
            clusters_[slot].enable_debug(debug_enabled_);
            metadata_[slot] = Metadata{};
            
            it = delayed_ids_.erase(it);
            cleaned++;
        } else {
            ++it;
        }
    }
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug && cleaned > 0) {
        std::cout << "[ClusterPool] cleanup_delayed_clusters: cleaned=" << cleaned << std::endl;
    }
#endif
    
    if (cleaned > 0) {
        VRL_LOG_DEBUG(modules::CORE, "Cleaned " + std::to_string(cleaned) + " delayed clusters");
    }
    
    return cleaned;
}

// ============================================================================
// ПРОВЕРКИ
// ============================================================================

bool ClusterPool::has_closed_clusters(int sector) const {
    if (sector < 0 || sector >= NUM_SECTORS) {
        return false;
    }
    return !closed_by_sector_[sector].empty();
}

bool ClusterPool::has_wide_clusters() const {
    return !wide_ids_.empty();
}

bool ClusterPool::has_delayed_clusters() const {
    return !delayed_ids_.empty();
}

// ============================================================================
// ПОДСЧЁТ
// ============================================================================

size_t ClusterPool::count_active_clusters() const {
    return active_ids_.size();
}

size_t ClusterPool::count_closed_clusters() const {
    size_t count = 0;
    for (const auto& vec : closed_by_sector_) {
        count += vec.size();
    }
    return count;
}

size_t ClusterPool::count_delayed_clusters() const {
    return delayed_ids_.size();
}

size_t ClusterPool::count_wide_clusters() const {
    size_t count = 0;
    for (size_t i = 0; i < max_clusters_; ++i) {
        if (metadata_[i].state == ClusterState::WIDE) {
            count++;
        }
    }
    return count;
}

size_t ClusterPool::size() const {
    return active_ids_.size() + count_closed_clusters() + delayed_ids_.size();
}

bool ClusterPool::is_empty() const {
    return active_ids_.empty() && count_closed_clusters() == 0 && delayed_ids_.empty();
}

// ============================================================================
// УПРАВЛЕНИЕ
// ============================================================================

void ClusterPool::clear() {
    auto debug = debug_enabled_;
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    if (debug) {
        std::cout << "[ClusterPool] clear: called" << std::endl;
    }
#endif
    
    for (size_t i = 0; i < max_clusters_; ++i) {
        clusters_[i] = Cluster();
        clusters_[i].set_id(i + 1);
        clusters_[i].enable_debug(debug_enabled_);
        metadata_[i] = Metadata{};
    }
    
    active_ids_.clear();
    for (auto& vec : closed_by_sector_) {
        vec.clear();
    }
    wide_ids_.clear();
    delayed_ids_.clear();
    next_slot_ = 0;
    
    VRL_LOG_INFO(modules::CORE, "ClusterPool cleared");
}

// ============================================================================
// СТАТИСТИКА
// ============================================================================

ClusterPool::Stats ClusterPool::get_stats() const {
    Stats stats;
    stats.total_clusters = size();
    stats.active_clusters = active_ids_.size();
    
    for (const auto& vec : closed_by_sector_) {
        stats.closed_clusters += vec.size();
    }
    stats.wide_clusters = count_wide_clusters();
    stats.delayed_clusters = delayed_ids_.size();
    
    for (int i = 0; i < NUM_SECTORS; ++i) {
        stats.clusters_by_sector[i] = closed_by_sector_[i].size();
    }
    
    return stats;
}

} // namespace radar
} // namespace vrl
