// src/core/cluster_pool.cpp
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/utils/logger.h"

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

ClusterPool& ClusterPool::instance() {
    static ClusterPool instance;
    return instance;
}

Cluster& ClusterPool::create_cluster() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto cluster = std::make_unique<Cluster>();
    clusters_.push_back(std::move(cluster));
    
    VRL_LOG_DEBUG(modules::CORE, "Cluster created, total: " + std::to_string(clusters_.size()));
    
    return *clusters_.back();
}

Cluster& ClusterPool::get_cluster(size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= clusters_.size()) {
        VRL_LOG_ERROR(modules::CORE, "Cluster index out of range: " + std::to_string(index));
        static Cluster empty;
        return empty;
    }
    
    return *clusters_[index];
}

const Cluster& ClusterPool::get_cluster(size_t index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= clusters_.size()) {
        VRL_LOG_ERROR(modules::CORE, "Cluster index out of range: " + std::to_string(index));
        static Cluster empty;
        return empty;
    }
    
    return *clusters_[index];
}

const std::vector<std::unique_ptr<Cluster>>& ClusterPool::get_all_clusters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return clusters_;
}

std::vector<Cluster*> ClusterPool::get_active_clusters() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Cluster*> result;
    result.reserve(clusters_.size());
    
    for (auto& cluster : clusters_) {
        if (cluster && !cluster->is_closed()) {
            result.push_back(cluster.get());
        }
    }
    
    return result;
}

std::vector<Cluster*> ClusterPool::get_closed_clusters() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Cluster*> result;
    result.reserve(clusters_.size());
    
    for (auto& cluster : clusters_) {
        if (cluster && cluster->is_closed()) {
            result.push_back(cluster.get());
        }
    }
    
    return result;
}

void ClusterPool::remove_cluster(size_t index) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (index >= clusters_.size()) {
        VRL_LOG_ERROR(modules::CORE, "Cannot remove cluster: index out of range: " + std::to_string(index));
        return;
    }
    
    clusters_.erase(clusters_.begin() + index);
    VRL_LOG_DEBUG(modules::CORE, "Cluster removed, remaining: " + std::to_string(clusters_.size()));
}

// НОВЫЙ МЕТОД
void ClusterPool::remove_cluster(Cluster* cluster) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (size_t i = 0; i < clusters_.size(); ++i) {
        if (clusters_[i].get() == cluster) {
            clusters_.erase(clusters_.begin() + i);
            VRL_LOG_DEBUG(modules::CORE, "Cluster removed by pointer, remaining: " + 
                          std::to_string(clusters_.size()));
            return;
        }
    }
    
    VRL_LOG_WARN(modules::CORE, "Cluster not found in pool");
}

void ClusterPool::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    clusters_.clear();
    VRL_LOG_INFO(modules::CORE, "ClusterPool cleared");
}

} // namespace radar
} // namespace vrl
