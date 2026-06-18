// src/processing/tracker.cpp
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ============================================================================
// REVOLUTION KALMAN FILTER
// ============================================================================

RevolutionKalmanFilter::RevolutionKalmanFilter() 
    : Q_(0.1), R_(1.0), initialized_(false) {
    update_matrices();
}

RevolutionKalmanFilter::RevolutionKalmanFilter(double process_noise, double measurement_noise)
    : Q_(process_noise), R_(measurement_noise), initialized_(false) {
    update_matrices();
}

void RevolutionKalmanFilter::update_matrices() {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P_[i][j] = (i == j) ? 100.0 : 0.0;
        }
    }
}

void RevolutionKalmanFilter::init(double x, double y, uint32_t revolution) {
    VRL_LOG_DEBUG(modules::KALMAN, "Initializing filter at (" + std::to_string(x) + ", " + 
                  std::to_string(y) + ") rev " + std::to_string(revolution));
    
    x_ = x;
    y_ = y;
    vx_ = 0.0;
    vy_ = 0.0;
    last_revolution_ = revolution;
    
    for (int i = 0; i < 4; ++i) {
        P_[i][i] = 100.0;
    }
    
    initialized_ = true;
}

void RevolutionKalmanFilter::predict(uint32_t delta_revolutions) {
    if (!initialized_ || delta_revolutions == 0) return;
    
    double dt = static_cast<double>(delta_revolutions);
    
    double old_x = x_;
    double old_y = y_;
    
    x_ = x_ + vx_ * dt;
    y_ = y_ + vy_ * dt;
    
    for (int i = 0; i < 4; ++i) {
        P_[i][i] += Q_ * dt;
    }
    
    VRL_LOG_TRACE(modules::KALMAN, "Predicted: (" + std::to_string(x_) + ", " + 
                  std::to_string(y_) + ") from (" + std::to_string(old_x) + ", " + 
                  std::to_string(old_y) + ") dt=" + std::to_string(dt));
}

void RevolutionKalmanFilter::update(double x, double y, uint32_t revolution) {
    if (!initialized_) {
        init(x, y, revolution);
        return;
    }
    
    uint32_t delta_rev = revolution - last_revolution_;
    if (delta_rev > 0) {
        predict(delta_rev);
    }
    
    double dx = x - x_;
    double dy = y - y_;
    
    double K_x = P_[0][0] / (P_[0][0] + R_);
    double K_y = P_[1][1] / (P_[1][1] + R_);
    
    x_ = x_ + K_x * dx;
    y_ = y_ + K_y * dy;
    
    if (delta_rev > 0) {
        vx_ = (x_ - (x_ - vx_ * delta_rev)) / delta_rev;
        vy_ = (y_ - (y_ - vy_ * delta_rev)) / delta_rev;
    }
    
    P_[0][0] = (1 - K_x) * P_[0][0];
    P_[1][1] = (1 - K_y) * P_[1][1];
    
    last_revolution_ = revolution;
    
    VRL_LOG_TRACE(modules::KALMAN, "Updated: (" + std::to_string(x_) + ", " + 
                  std::to_string(y_) + ") speed=(" + std::to_string(vx_) + ", " + 
                  std::to_string(vy_) + ")");
}

std::pair<double, double> RevolutionKalmanFilter::predict_position(uint32_t delta_revolutions) const {
    if (!initialized_) return {0.0, 0.0};
    return {x_ + vx_ * delta_revolutions, y_ + vy_ * delta_revolutions};
}

// ============================================================================
// TRACK MANAGER
// ============================================================================

// Структура TrackWithFilter определена в заголовочном файле,
// поэтому здесь её НЕ определяем.

TrackManager::TrackManager(const TrackerConfig& config) : config_(config), next_id_(1) {
    VRL_LOG_INFO(modules::TRACKER, "TrackManager initialized");
    VRL_LOG_DEBUG(modules::TRACKER, "Config: min_hits=" + std::to_string(config.min_hits_to_confirm) +
                  ", max_coast=" + std::to_string(config.max_coast_count) +
                  ", gate_dist=" + std::to_string(config.max_gate_distance) +
                  ", gate_az=" + std::to_string(config.max_gate_azimuth));
}

void TrackManager::process_targets(const std::vector<TargetReport>& targets, uint32_t revolution) {
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(targets.size()) + 
                  " targets at revolution " + std::to_string(revolution));
    
    if (targets.empty()) {
        VRL_LOG_WARN(modules::TRACKER, "No targets to process at revolution " + 
                     std::to_string(revolution));
        return;
    }
    
    size_t prev_tracks = tracks_.size();
    
    update_tracks(targets, revolution);
    create_new_tracks(targets, revolution);
    manage_track_states(revolution);
    
    VRL_LOG_DEBUG(modules::TRACKER, "Tracks: " + std::to_string(tracks_.size()) + 
                  " (was " + std::to_string(prev_tracks) + ")");
    
    if (config_.debug_mode) {
        int confirmed = 0, coasting = 0, new_tracks = 0;
        for (const auto& [id, twf] : tracks_) {
            if (twf.track.state == TrackState::ACTIVE) confirmed++;
            else if (twf.track.state == TrackState::COASTING) coasting++;
            else if (twf.track.state == TrackState::NEW) new_tracks++;
        }
        VRL_LOG_DEBUG(modules::TRACKER, "Track states: ACTIVE=" + std::to_string(confirmed) +
                      ", COASTING=" + std::to_string(coasting) + ", NEW=" + std::to_string(new_tracks));
    }
}

void TrackManager::update_tracks(const std::vector<TargetReport>& targets, uint32_t revolution) {
    std::vector<bool> target_matched(targets.size(), false);
    int updates = 0;
    
    for (auto& [id, twf] : tracks_) {
        auto& track = twf.track;
        auto& filter = twf.filter;
        
        if (track.state == TrackState::DROPPED) continue;
        
        uint32_t delta_rev = revolution - track.last_update_revolution;
        
        if (delta_rev > 0 && track.state == TrackState::ACTIVE) {
            auto [pred_x, pred_y] = filter.predict_position(delta_rev);
            track.x = pred_x;
            track.y = pred_y;
        }
        
        int best_idx = -1;
        double best_distance = config_.max_gate_distance;
        
        for (size_t i = 0; i < targets.size(); ++i) {
            if (target_matched[i]) continue;
            
            const auto& target = targets[i];
            
            if ((target.type == TargetReport::SourceType::RBS && !config_.enable_rbs_tracking) ||
                (target.type == TargetReport::SourceType::UVD && !config_.enable_uvd_tracking)) {
                continue;
            }
            
            if (!is_code_match(target, track)) continue;
            
            double distance = calculate_distance(target, track);
            
            double az_diff = calculate_azimuth_diff(target.azimuth_deg, track.azimuth_deg);
            if (az_diff > config_.max_gate_azimuth) continue;
            
            if (distance < best_distance) {
                best_distance = distance;
                best_idx = static_cast<int>(i);
            }
        }
        
        if (best_idx >= 0) {
            const auto& target = targets[best_idx];
            target_matched[best_idx] = true;
            
            filter.update(target.x, target.y, revolution);
            
            track.x = filter.get_x();
            track.y = filter.get_y();
            track.vx = filter.get_vx();
            track.vy = filter.get_vy();
            track.ground_speed = filter.get_speed();
            track.course_deg = filter.get_course();
            track.azimuth_deg = target.azimuth_deg;
            track.range_m = target.range_m;
            track.last_revolution = revolution;
            track.last_update_revolution = revolution;
            track.hit_count++;
            track.coast_count = 0;
            
            if (target.type == TargetReport::SourceType::RBS) {
                track.mode3a_code = target.rbs.mode3a_code;
                track.spi = target.rbs.spi;
                if (target.rbs.modec_altitude > 0) {
                    track.altitude = target.rbs.modec_altitude;
                }
            } else {
                track.uvd_data20 = target.uvd.raw_data20;
                track.altitude = target.uvd.altitude;
            }
            
            track.confidence = std::min(1.0, static_cast<double>(track.hit_count) / 10.0);
            track.position_error = best_distance;
            track.add_history(target);
            updates++;
            
            if (track.hit_count >= config_.min_hits_to_confirm && 
                track.state == TrackState::NEW) {
                track.state = TrackState::ACTIVE;
                VRL_LOG_INFO(modules::TRACKER, "Track " + std::to_string(track.id) + 
                             " confirmed at rev " + std::to_string(revolution) +
                             " (hits=" + std::to_string(track.hit_count) + ")");
            }
        } else {
            track.coast_count += (delta_rev > 0) ? delta_rev : 1;
            track.last_update_revolution = revolution;
            
            if (track.coast_count >= config_.max_coast_count) {
                track.state = TrackState::DROPPED;
                VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(track.id) + 
                              " dropped (coast=" + std::to_string(track.coast_count) + ")");
            } else if (track.state == TrackState::ACTIVE && track.coast_count > 0) {
                track.state = TrackState::COASTING;
                VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(track.id) + 
                              " coasting (coast=" + std::to_string(track.coast_count) + ")");
                if (delta_rev > 0) {
                    auto [pred_x, pred_y] = filter.predict_position(delta_rev);
                    track.x = pred_x;
                    track.y = pred_y;
                    track.range_m = std::sqrt(track.x*track.x + track.y*track.y);
                    track.azimuth_deg = std::atan2(track.x, track.y) * 180.0 / M_PI;
                    if (track.azimuth_deg < 0) track.azimuth_deg += 360.0;
                }
            }
        }
    }
    
    if (updates > 0) {
        VRL_LOG_TRACE(modules::TRACKER, "Updated " + std::to_string(updates) + " tracks");
    }
    
    auto it = tracks_.begin();
    while (it != tracks_.end()) {
        if (it->second.track.state == TrackState::DROPPED) {
            it = tracks_.erase(it);
        } else {
            ++it;
        }
    }
}

void TrackManager::create_new_tracks(const std::vector<TargetReport>& targets, uint32_t revolution) {
    int created = 0;
    
    for (const auto& target : targets) {
        bool already_tracked = false;
        
        for (const auto& [id, twf] : tracks_) {
            double distance = calculate_distance(target, twf.track);
            if (distance < config_.max_gate_distance) {
                already_tracked = true;
                break;
            }
        }
        
        if (!already_tracked) {
            Track new_track;
            new_track.id = next_id_++;
            new_track.state = TrackState::NEW;
            new_track.x = target.x;
            new_track.y = target.y;
            new_track.azimuth_deg = target.azimuth_deg;
            new_track.range_m = target.range_m;
            new_track.first_revolution = revolution;
            new_track.last_revolution = revolution;
            new_track.last_update_revolution = revolution;
            new_track.hit_count = 1;
            new_track.coast_count = 0;
            new_track.confidence = 0.1;
            
            if (target.type == TargetReport::SourceType::RBS) {
                new_track.mode3a_code = target.rbs.mode3a_code;
                new_track.spi = target.rbs.spi;
            } else {
                new_track.uvd_data20 = target.uvd.raw_data20;
                new_track.altitude = target.uvd.altitude;
            }
            
            new_track.add_history(target);
            
            RevolutionKalmanFilter filter(config_.process_noise, config_.measurement_noise);
            filter.init(target.x, target.y, revolution);
            
            tracks_[new_track.id] = TrackWithFilter(new_track, filter);
            created++;
            
            VRL_LOG_DEBUG(modules::TRACKER, "Created new track " + std::to_string(new_track.id) + 
                          " at rev " + std::to_string(revolution) +
                          " pos=(" + std::to_string(target.x) + ", " + std::to_string(target.y) + ")");
        }
    }
    
    if (created > 0) {
        VRL_LOG_DEBUG(modules::TRACKER, "Created " + std::to_string(created) + " new tracks");
    }
}

void TrackManager::manage_track_states(uint32_t revolution) {
    (void)revolution;
    int coasting_updated = 0;
    
    for (auto& [id, twf] : tracks_) {
        auto& track = twf.track;
        
        if (track.state == TrackState::ACTIVE && track.coast_count == 0) {
            continue;
        }
        
        if (track.state == TrackState::COASTING) {
            track.confidence = std::max(0.0, track.confidence - 0.05);
            coasting_updated++;
        }
    }
    
    if (coasting_updated > 0) {
        VRL_LOG_TRACE(modules::TRACKER, "Updated " + std::to_string(coasting_updated) + " coasting tracks");
    }
}

double TrackManager::calculate_distance(const TargetReport& target, const Track& track) const {
    double dx = target.x - track.x;
    double dy = target.y - track.y;
    return std::sqrt(dx*dx + dy*dy);
}

bool TrackManager::is_code_match(const TargetReport& target, const Track& track) const {
    if (target.type == TargetReport::SourceType::RBS) {
        if (track.mode3a_code != 0 && track.mode3a_code != target.rbs.mode3a_code) {
            return false;
        }
    }
    return true;
}

double TrackManager::calculate_azimuth_diff(double az1, double az2) const {
    double diff = std::abs(az1 - az2);
    return std::min(diff, 360.0 - diff);
}

std::vector<Track> TrackManager::get_active_tracks() const {
    std::vector<Track> result;
    for (const auto& [id, twf] : tracks_) {
        if (twf.track.state == TrackState::ACTIVE || 
            twf.track.state == TrackState::COASTING ||
            twf.track.state == TrackState::NEW) {
            result.push_back(twf.track);
        }
    }
    
    std::sort(result.begin(), result.end(),
        [](const Track& a, const Track& b) {
            return a.confidence > b.confidence;
        });
    
    return result;
}

std::vector<Track> TrackManager::get_confirmed_tracks() const {
    std::vector<Track> result;
    for (const auto& [id, twf] : tracks_) {
        if (twf.track.is_confirmed()) {
            result.push_back(twf.track);
        }
    }
    return result;
}

void TrackManager::reset() {
    size_t old_size = tracks_.size();
    tracks_.clear();
    next_id_ = 1;
    VRL_LOG_INFO(modules::TRACKER, "Reset all tracks (cleared " + std::to_string(old_size) + " tracks)");
}

} // namespace radar
} // namespace vrl
