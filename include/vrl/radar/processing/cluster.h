// include/vrl/radar/processing/cluster.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "reply_processor.h"
#include "garbling_solver.h"
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
    
    void add_scan(const ScanReplies& scan) {
        if (scans.empty()) {
            start_azimuth = scan.azimuth;
            first_timestamp = scan.timestamp_ms;
            min_range = 65535;
            max_range = 0;
        }
        
        scans.push_back(scan);
        last_processed_azimuth = scan.azimuth;
        
        if (scan.has_replies()) {
            last_reply_azimuth = scan.azimuth;
            last_timestamp = scan.timestamp_ms;
        }
        
        for (const auto& reply : scan.rbs_replies) {
            rbs_by_azimuth[scan.azimuth].push_back(reply);
            min_range = std::min(min_range, reply.range);
            max_range = std::max(max_range, reply.range);
        }
        
        for (const auto& reply : scan.uvd_replies) {
            uvd_by_azimuth[scan.azimuth].push_back(reply);
            min_range = std::min(min_range, reply.range);
            max_range = std::max(max_range, reply.range);
        }
    }
    
    bool is_active(uint16_t current_azimuth, int max_gap_azimuth) const {
        if (scans.empty()) return false;
        
        int16_t gap = current_azimuth - last_reply_azimuth;
        if (gap < 0) gap += 4096;
        return gap <= max_gap_azimuth;
    }
    
    std::vector<RBSReply> get_all_rbs() const {
        std::vector<RBSReply> result;
        for (const auto& scan : scans) {
            result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
        }
        return result;
    }
    
    std::vector<UVDReply> get_all_uvd() const {
        std::vector<UVDReply> result;
        for (const auto& scan : scans) {
            result.insert(result.end(), scan.uvd_replies.begin(), scan.uvd_replies.end());
        }
        return result;
    }
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
// CLUSTER PROCESSOR
// ============================================================================

class ClusterProcessor {
public:
    explicit ClusterProcessor(const RadarConfig& config);
    
    std::vector<TargetReport> process_cluster(const TargetCluster& cluster);
    
    void set_range_tolerance(uint16_t bins) { range_tolerance_ = bins; }
    void set_min_hits(int hits) { min_hits_ = hits; }
    void set_confidence_threshold(uint8_t thresh) { confidence_threshold_ = thresh; }
    void enable_garbling_splitting(bool enable) { split_garbled_ = enable; }
    void set_garbling_solver(std::unique_ptr<GarblingSolver> solver) {
        garbling_solver_ = std::move(solver);
    }
    void set_debug(bool enable) { debug_ = enable; }
    
private:
    struct RangeGroup {
        uint16_t nominal_range{0};
        std::vector<const RBSReply*> rbs_replies;
        std::vector<const UVDReply*> uvd_replies;
        std::set<uint16_t> azimuths;
        
        void add_rbs(const RBSReply* reply) {
            rbs_replies.push_back(reply);
            azimuths.insert(reply->azimuth);
        }
        
        void add_uvd(const UVDReply* reply) {
            uvd_replies.push_back(reply);
            azimuths.insert(reply->azimuth);
        }
        
        size_t total_replies() const {
            return rbs_replies.size() + uvd_replies.size();
        }
    };
    
    std::vector<RangeGroup> group_by_range(const TargetCluster& cluster);
    double average_azimuth(const std::vector<uint16_t>& azimuths);
    bool check_sidelobe(const RBSReply& reply) const;
    bool check_sidelobe(const UVDReply& reply) const;
    void decode_uvd_info(uint32_t data20, TargetReport& report);
    
    std::optional<TargetReport> process_rbs_group(const RangeGroup& group);
    std::optional<TargetReport> process_uvd_group(const RangeGroup& group);
    std::vector<TargetReport> process_garbled_group(const RangeGroup& group);
    
    RadarConfig config_;
    ReplyProcessor reply_processor_;
    uint16_t range_tolerance_{5};
    int min_hits_{2};
    uint8_t confidence_threshold_{50};
    bool split_garbled_{true};
    bool debug_{false};
    std::unique_ptr<GarblingSolver> garbling_solver_;
};

} // namespace radar
} // namespace vrl
