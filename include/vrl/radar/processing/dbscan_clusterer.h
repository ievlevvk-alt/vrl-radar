// include/vrl/radar/processing/dbscan_clusterer.h
#pragma once

#include "i_clusterer.h"
#include "../core/point_buffer.hpp"
#include "../core/cluster.hpp"
#include "../core/cluster_pool.hpp"
#include "../core/replies.h"
#include "../core/config.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

namespace vrl {
namespace radar {

class DBSCANClusterer : public IClusterer {
public:
    explicit DBSCANClusterer(const RadarConfig& config,
                             int max_range_gap = 3,
                             double azimuth_gap_coefficient = 1.2);
    
    ~DBSCANClusterer() override = default;
    
    // === Реализация IClusterer ===
    void process_scan(const ScanReplies& scan) override;
    std::vector<TargetCluster> get_completed_clusters() override;
    const std::vector<TargetCluster>& get_active_clusters() const override;
    void reset() override;
    std::vector<TargetCluster> finish_all() override;
    
    std::string get_name() const override { return "DBSCANClusterer"; }
    void get_stats(size_t& active, size_t& completed) const override;
    
    std::unique_ptr<IClusterer> clone() const override;
    
    void set_param(const std::string& key, double value) override;
    void set_param(const std::string& key, int value) override;
    
    // === Дополнительные методы ===
    void set_debug(bool enable) { debug_ = enable; }
    int get_max_azimuth_gap() const { return max_azimuth_gap_; }
    int get_max_range_gap() const { return max_range_gap_; }
    
    // Публичный для тестов
    void close_expired_clusters(uint16_t current_azimuth);
    
    size_t count_active_clusters() const;

private:
    void process_point(uint16_t azimuth, uint16_t range, bool is_rbs,
                       const StoredPoint& point, size_t buffer_index);
    Cluster* find_cluster(uint16_t azimuth, uint16_t range, bool is_rbs);
    void create_cluster(uint16_t azimuth, uint16_t range, bool is_rbs,
                        size_t buffer_index);
    void merge_overlapping_clusters();
    bool clusters_overlap(const Cluster& a, const Cluster& b) const;
    void refresh_active_clusters() const;
    void debug_print(const std::string& msg = "");
    
    int max_azimuth_gap_{68};
    int max_range_gap_{3};
    double azimuth_gap_coefficient_{1.2};
    bool debug_{false};
    
    mutable std::vector<Cluster*> active_clusters_;
    
    size_t total_scans_processed_{0};
    size_t total_points_processed_{0};
    size_t total_clusters_formed_{0};
    size_t total_clusters_completed_{0};
    
    uint32_t current_revolution_{0};
    uint16_t current_azimuth_{0};
    
    RadarConfig config_;
    
    static constexpr int AZIMUTH_BINS = 4096;
};

} // namespace radar
} // namespace vrl
