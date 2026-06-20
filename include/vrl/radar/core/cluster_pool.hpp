// include/vrl/radar/core/cluster_pool.hpp
#pragma once

#include "cluster.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace vrl {
namespace radar {

class ClusterPool {
public:
    static ClusterPool& instance();
    
    Cluster& create_cluster();
    Cluster& get_cluster(size_t index);
    const Cluster& get_cluster(size_t index) const;
    const std::vector<std::unique_ptr<Cluster>>& get_all_clusters() const;
    std::vector<Cluster*> get_active_clusters();
    std::vector<Cluster*> get_closed_clusters();
    
    void remove_cluster(size_t index);
    void remove_cluster(Cluster* cluster);  // <-- НОВЫЙ МЕТОД
    
    void clear();
    size_t size() const { return clusters_.size(); }
    bool is_empty() const { return clusters_.empty(); }

private:
    ClusterPool() = default;
    ~ClusterPool() = default;
    ClusterPool(const ClusterPool&) = delete;
    ClusterPool& operator=(const ClusterPool&) = delete;
    
    std::vector<std::unique_ptr<Cluster>> clusters_;
    mutable std::mutex mutex_;
};

} // namespace radar
} // namespace vrl
