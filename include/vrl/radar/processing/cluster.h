// include/vrl/radar/processing/cluster.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "reply_processor.h"
#include "garbling_solver.h"
#include "range_grouper.h"
#include "rbs_processor.h"
#include "uvd_processor.h"
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
    
    std::map<uint16_t, std::vector<RBSReply>> rbs_by_azimuth;
    std::map<uint16_t, std::vector<UVDReply>> uvd_by_azimuth;
    
    void add_scan(const ScanReplies& scan);
    bool is_active(uint16_t current_azimuth, int max_gap_azimuth) const;
    std::vector<RBSReply> get_all_rbs() const;
    std::vector<UVDReply> get_all_uvd() const;
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
    
    void process_scan(const ScanReplies& scan);
    std::vector<TargetCluster> get_completed_clusters();
    const std::vector<TargetCluster>& get_active_clusters() const;
    void reset();
    
    void set_max_gap_azimuth(int gap) { max_gap_azimuth_ = gap; }
    void set_range_window(int window) { range_window_ = window; }
    
private:
    void update_existing_clusters(const ScanReplies& scan);
    void try_create_new_clusters(const ScanReplies& scan);
    void complete_expired_clusters(uint16_t current_azimuth);
    
    std::vector<TargetCluster> active_clusters_;
    std::vector<TargetCluster> completed_clusters_;
    
    int max_gap_azimuth_;
    int range_window_;
};

// ============================================================================
// CLUSTER PROCESSOR - ОБНОВЛЕННАЯ ВЕРСИЯ
// ============================================================================

class ClusterProcessor {
public:
    explicit ClusterProcessor(const RadarConfig& config);
    
    std::vector<TargetReport> process_cluster(const TargetCluster& cluster);
    
    // Настройка параметров
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
    
    // Доступ к подпроцессорам
    RBSProcessor& get_rbs_processor() { return rbs_processor_; }
    UVDProcessor& get_uvd_processor() { return uvd_processor_; }
    RangeGrouper& get_range_grouper() { return range_grouper_; }
    
private:
    /**
     * @brief Обработать группу с перекрытием (garbled)
     * @param group группа ответов
     * @return вектор отчетов о целях
     */
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
