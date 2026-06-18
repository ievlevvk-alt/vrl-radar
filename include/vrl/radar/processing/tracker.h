// include/vrl/radar/processing/tracker.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "kalman_filter.h"  // <-- ДОБАВЛЯЕМ ЭТУ СТРОКУ
#include <vector>
#include <map>
#include <memory>
#include <optional>

namespace vrl {
namespace radar {

// ============================================================================
// TRACK
// ============================================================================

struct Track {
    uint64_t id{0};
    TrackState state{TrackState::NEW};
    
    double x{0.0}, y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    double vx{0.0}, vy{0.0};
    double ground_speed{0.0};
    double course_deg{0.0};
    
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    uint32_t first_revolution{0};
    uint32_t last_revolution{0};
    uint32_t last_update_revolution{0};
    uint32_t coast_count{0};
    uint32_t hit_count{0};
    
    double confidence{0.0};
    double position_error{0.0};
    bool code_reliable{true};
    bool altitude_reliable{true};
    
    std::vector<TargetReport> history;
    size_t max_history{10};
    
    void add_history(const TargetReport& report) {
        history.push_back(report);
        if (history.size() > max_history) {
            history.erase(history.begin());
        }
    }
    
    bool is_confirmed() const {
        return hit_count >= 3 && state == TrackState::ACTIVE;
    }
};

// ============================================================================
// TRACK MANAGER
// ============================================================================

class TrackManager {
public:
    explicit TrackManager(const TrackerConfig& config = TrackerConfig());
    ~TrackManager() = default;
    
    void process_targets(const std::vector<TargetReport>& targets, uint32_t revolution);
    std::vector<Track> get_active_tracks() const;
    std::vector<Track> get_confirmed_tracks() const;
    void reset();
    void set_debug(bool enable) { config_.debug_mode = enable; }
    const TrackerConfig& get_config() const { return config_; }
    
private:
    struct TrackWithFilter {
        Track track;
        RevolutionKalmanFilter filter;  // Теперь это определено через include
        
        TrackWithFilter() = default;
        TrackWithFilter(const Track& t, const RevolutionKalmanFilter& f)
            : track(t), filter(f) {}
    };
    
    void update_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void create_new_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void manage_track_states(uint32_t revolution);
    double calculate_distance(const TargetReport& target, const Track& track) const;
    bool is_code_match(const TargetReport& target, const Track& track) const;
    double calculate_azimuth_diff(double az1, double az2) const;
    
    TrackerConfig config_;
    uint64_t next_id_{1};
    std::map<uint64_t, TrackWithFilter> tracks_;
};

} // namespace radar
} // namespace vrl
