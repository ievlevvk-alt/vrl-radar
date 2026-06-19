// include/vrl/radar/processing/cluster.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "reply_processor.h"
#include "garbling_solver.h"
#include "range_grouper.h"  // <-- УЖЕ ЕСТЬ
#include "rbs_processor.h"
#include "uvd_processor.h"
#include "i_clusterer.h"
#include "legacy_clusterer.h"
#include <vector>
#include <map>
#include <set>
#include <optional>
#include <memory>

namespace vrl {
namespace radar {

// ============================================================================
// TARGET CLUSTER
// ============================================================================

struct TargetCluster {
    std::vector<ScanReplies> scans;
    uint16_t start_azimuth{0};
    uint16_t last_reply_azimuth{0};
    uint16_t last_processed_azimuth{0};
    uint16_t min_range{65535};
    uint16_t max_range{0};
    uint32_t first_timestamp{0};
    uint32_t last_timestamp{0};
    
    uint32_t created_at_revolution{0};
    uint32_t last_update_revolution{0};
    uint32_t revolutions_since_update{0};
    uint32_t max_revolutions_no_update{5};
    bool marked_for_cleanup{false};
    
    std::map<uint16_t, std::vector<RBSReply>> rbs_by_azimuth;
    std::map<uint16_t, std::vector<UVDReply>> uvd_by_azimuth;
    
    void add_scan(const ScanReplies& scan, uint32_t revolution);
    bool is_active(uint16_t current_azimuth, int max_gap_azimuth) const;
    bool is_expired(uint32_t current_revolution) const;
    bool should_be_cleaned(uint32_t current_revolution) const;
    void mark_for_cleanup() { marked_for_cleanup = true; }
    bool needs_cleanup() const { return marked_for_cleanup; }
    void update_revolution(uint32_t revolution);
    uint32_t get_revolutions_since_update() const { return revolutions_since_update; }
    bool has_replies() const { return !rbs_by_azimuth.empty() || !uvd_by_azimuth.empty(); }
    
    std::vector<RBSReply> get_all_rbs() &&;
    std::vector<RBSReply> get_all_rbs() const&;
    std::vector<UVDReply> get_all_uvd() &&;
    std::vector<UVDReply> get_all_uvd() const&;
    
    uint16_t azimuth_span() const;
    uint16_t range_span() const;
    size_t reply_scans_count() const;
};

// ============================================================================
// CLUSTER TRACKER
// ============================================================================

class ClusterTracker {
public:
    ClusterTracker(int max_gap_azimuth = 8, int range_window = 30);
    explicit ClusterTracker(std::unique_ptr<IClusterer> clusterer);
    ~ClusterTracker() = default;
    
    void process_scan(const ScanReplies& scan);
    std::vector<TargetCluster> get_completed_clusters();
    const std::vector<TargetCluster>& get_active_clusters() const;
    void reset();
    void set_clusterer(std::unique_ptr<IClusterer> clusterer);
    IClusterer* get_clusterer() const { return clusterer_.get(); }
    std::string get_algorithm_name() const;
    
    void set_max_revolutions_no_update(uint32_t max) { max_revolutions_no_update_ = max; }
    uint32_t get_max_revolutions_no_update() const { return max_revolutions_no_update_; }
    void set_max_active_clusters(size_t max) { max_active_clusters_ = max; }
    size_t get_max_active_clusters() const { return max_active_clusters_; }
    size_t cleanup_stale_clusters(uint32_t current_revolution);
    
    void set_max_gap_azimuth(int gap);
    void set_range_window(int window);
    
    struct ClusterStats {
        size_t active_count{0};
        size_t completed_count{0};
        size_t cleaned_count{0};
        size_t total_scans_processed{0};
        size_t total_clusters_formed{0};
        size_t total_clusters_completed{0};
        size_t total_clusters_cleaned{0};
    };
    ClusterStats get_stats() const;
    
private:
    std::unique_ptr<IClusterer> clusterer_;
    
    uint32_t max_revolutions_no_update_{5};
    size_t max_active_clusters_{100};
    uint32_t current_revolution_{0};
    mutable size_t total_clusters_cleaned_{0};
    mutable size_t cached_completed_count_{0};
};

// ============================================================================
// CLUSTER PROCESSOR
// ============================================================================

class ClusterProcessor {
public:
    explicit ClusterProcessor(const RadarConfig& config);
    
    std::vector<TargetReport> process_cluster(const TargetCluster& cluster);
    
    void set_range_tolerance(uint16_t bins) { 
        range_grouper_.set_tolerance(bins); 
    }
    void set_min_hits(int hits) { 
        min_hits_ = hits;
        rbs_processor_.set_min_hits(hits);
        uvd_processor_.set_min_hits(hits);
    }
    void set_confidence_threshold(double thresh) {
        rbs_processor_.set_min_confidence(thresh);
        uvd_processor_.set_min_confidence(thresh);
    }
    void enable_garbling_splitting(bool enable) { split_garbled_ = enable; }
    void set_garbling_solver(std::unique_ptr<GarblingSolver> solver);
    void set_debug(bool enable) { debug_ = enable; }
    
    RBSProcessor& get_rbs_processor() { return rbs_processor_; }
    UVDProcessor& get_uvd_processor() { return uvd_processor_; }
    RangeGrouper& get_range_grouper() { return range_grouper_; }
    
private:
    std::vector<TargetReport> process_garbled_group(const RangeGrouper::RangeGroup& group);
    
    RadarConfig config_;
    RangeGrouper range_grouper_;
    RBSProcessor rbs_processor_;
    UVDProcessor uvd_processor_;
    
    int min_hits_{2};
    bool split_garbled_{true};
    bool debug_{false};
    std::unique_ptr<GarblingSolver> garbling_solver_;
};

} // namespace radar
} // namespace vrl
