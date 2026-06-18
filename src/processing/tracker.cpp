// src/processing/tracker.cpp
#include "vrl/radar/processing/tracker.h"  // Теперь включает kalman_filter.h
#include <algorithm>
#include <cmath>
#include <iostream>

namespace vrl {
namespace radar {

// ============================================================================
// REVOLUTION KALMAN FILTER - ЭТИ ФУНКЦИИ ДОЛЖНЫ БЫТЬ В kalman_filter.cpp
// НО ДЛЯ ТЕСТА МОЖЕМ ОСТАВИТЬ ИХ ЗДЕСЬ
// ============================================================================

// Если RevolutionKalmanFilter определен в kalman_filter.h,
// то эти функции должны быть в kalman_filter.cpp.
// Но для простоты мы можем оставить их здесь.

// ============================================================================
// TRACK MANAGER IMPLEMENTATION
// ============================================================================

TrackManager::TrackManager(const TrackerConfig& config) : config_(config), next_id_(1) {}

void TrackManager::process_targets(const std::vector<TargetReport>& targets, uint32_t revolution) {
    if (config_.debug_mode) {
        std::cout << "[TrackManager] Revolution " << revolution 
                  << ": processing " << targets.size() << " targets\n";
    }
    
    update_tracks(targets, revolution);
    create_new_tracks(targets, revolution);
    manage_track_states(revolution);
    
    if (config_.debug_mode) {
        std::cout << "[TrackManager] Active tracks: " << tracks_.size() << "\n";
    }
}

void TrackManager::update_tracks(const std::vector<TargetReport>& targets, uint32_t revolution) {
    std::vector<bool> target_matched(targets.size(), false);
    
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
            
            if (track.hit_count >= config_.min_hits_to_confirm && 
                track.state == TrackState::NEW) {
                track.state = TrackState::ACTIVE;
                if (config_.debug_mode) {
                    std::cout << "[TrackManager] Track " << track.id << " confirmed\n";
                }
            }
        } else {
            track.coast_count += (delta_rev > 0) ? delta_rev : 1;
            track.last_update_revolution = revolution;
            
            if (track.coast_count >= config_.max_coast_count) {
                track.state = TrackState::DROPPED;
                if (config_.debug_mode) {
                    std::cout << "[TrackManager] Track " << track.id << " dropped\n";
                }
            } else if (track.state == TrackState::ACTIVE && track.coast_count > 0) {
                track.state = TrackState::COASTING;
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
            
            if (config_.debug_mode) {
                std::cout << "[TrackManager] Created new track " << new_track.id 
                          << " at rev " << revolution << "\n";
            }
        }
    }
}

void TrackManager::manage_track_states(uint32_t revolution) {
    (void)revolution;  // Подавляем предупреждение о неиспользуемом параметре
    for (auto& [id, twf] : tracks_) {
        auto& track = twf.track;
        
        if (track.state == TrackState::ACTIVE && track.coast_count == 0) {
            continue;
        }
        
        if (track.state == TrackState::COASTING) {
            track.confidence = std::max(0.0, track.confidence - 0.05);
        }
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
    tracks_.clear();
    next_id_ = 1;
    if (config_.debug_mode) {
        std::cout << "[TrackManager] Reset all tracks\n";
    }
}

} // namespace radar
} // namespace vrl