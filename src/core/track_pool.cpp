// src/core/track_pool.cpp
#include "vrl/radar/core/track_pool.hpp"
#include "vrl/radar/core/replies.h"      // <-- Для TargetReport
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

TrackPool& TrackPool::instance() {
    static TrackPool instance;
    return instance;
}

uint64_t TrackPool::create_track(const TargetReport& report, int sector, FilterType filter_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Track track;
    track.id = next_id_++;
    track.state = TrackState::NEW;
    track.x = report.x;
    track.y = report.y;
    track.azimuth_deg = report.azimuth_deg;
    track.range_m = report.range_m;
    track.first_revolution = 0;
    track.last_revolution = 0;
    track.last_update_revolution = 0;
    track.hit_count = 1;
    track.coast_count = 0;
    track.confidence = 0.1;
    track.filter_type = filter_type;
    
    if (report.type == TargetReport::SourceType::RBS) {
        track.mode3a_code = report.rbs.mode3a_code;
        track.spi = report.rbs.spi;
    } else {
        track.uvd_data20 = report.uvd.raw_data20;
        track.altitude = report.uvd.altitude;
    }
    
    track.add_history(report);
    
    tracks_[track.id] = std::move(track);
    add_to_sector(track.id, sector);
    
    VRL_LOG_DEBUG(modules::TRACKER, "Created track " + std::to_string(track.id) + 
                  " in sector " + std::to_string(sector));
    
    return track.id;
}

Track* TrackPool::get_track(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = tracks_.find(id);
    if (it != tracks_.end()) {
        return &it->second;
    }
    return nullptr;
}

const Track* TrackPool::get_track(uint64_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = tracks_.find(id);
    if (it != tracks_.end()) {
        return &it->second;
    }
    return nullptr;
}

std::vector<Track*> TrackPool::get_tracks_in_sector(int sector_index) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Track*> result;
    
    if (sector_index < 0 || sector_index >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid sector index: " + 
                     std::to_string(sector_index));
        return result;
    }
    
    result.reserve(tracks_by_sector_[sector_index].size());
    
    for (uint64_t id : tracks_by_sector_[sector_index]) {
        auto it = tracks_.find(id);
        if (it != tracks_.end() && it->second.state != TrackState::DROPPED) {
            result.push_back(&it->second);
        }
    }
    
    return result;
}

std::vector<Track*> TrackPool::get_all_tracks() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Track*> result;
    result.reserve(tracks_.size());
    
    for (auto& [id, track] : tracks_) {
        if (track.state != TrackState::DROPPED) {
            result.push_back(&track);
        }
    }
    
    return result;
}

void TrackPool::update_track(uint64_t id, const TargetReport& report, int sector) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = tracks_.find(id);
    if (it == tracks_.end()) {
        VRL_LOG_WARN(modules::TRACKER, "Track " + std::to_string(id) + " not found");
        return;
    }
    
    Track& track = it->second;
    
    track.x = report.x;
    track.y = report.y;
    track.azimuth_deg = report.azimuth_deg;
    track.range_m = report.range_m;
    track.last_revolution++;
    track.last_update_revolution = track.last_revolution;
    track.hit_count++;
    track.coast_count = 0;
    track.confidence = std::min(1.0, track.confidence + 0.1);
    
    if (report.type == TargetReport::SourceType::RBS) {
        track.mode3a_code = report.rbs.mode3a_code;
        track.spi = report.rbs.spi;
    } else {
        track.uvd_data20 = report.uvd.raw_data20;
        track.altitude = report.uvd.altitude;
    }
    
    track.add_history(report);
    
    int old_sector = track_sectors_[id];
    if (old_sector != sector) {
        remove_from_sector(id, old_sector);
        add_to_sector(id, sector);
    }
    
    if (track.hit_count >= 3 && track.state == TrackState::NEW) {
        track.state = TrackState::ACTIVE;
        VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(id) + " confirmed");
    }
}

void TrackPool::remove_track(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = tracks_.find(id);
    if (it == tracks_.end()) {
        VRL_LOG_WARN(modules::TRACKER, "Track " + std::to_string(id) + " not found");
        return;
    }
    
    int sector = track_sectors_[id];
    remove_from_sector(id, sector);
    track_sectors_.erase(id);
    
    tracks_.erase(it);
    VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(id) + " removed");
}

void TrackPool::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    tracks_.clear();
    track_sectors_.clear();
    for (auto& vec : tracks_by_sector_) {
        vec.clear();
    }
    next_id_ = 1;
    VRL_LOG_INFO(modules::TRACKER, "TrackPool cleared");
}

void TrackPool::add_to_sector(uint64_t track_id, int sector) {
    if (sector >= 0 && sector < NUM_SECTORS) {
        tracks_by_sector_[sector].push_back(track_id);
        track_sectors_[track_id] = sector;
    }
}

void TrackPool::remove_from_sector(uint64_t track_id, int sector) {
    if (sector >= 0 && sector < NUM_SECTORS) {
        auto& vec = tracks_by_sector_[sector];
        auto it = std::find(vec.begin(), vec.end(), track_id);
        if (it != vec.end()) {
            vec.erase(it);
        }
    }
}

int TrackPool::get_sector_for_track(const Track& track) const {
    double az_deg = track.azimuth_deg;
    if (az_deg < 0) az_deg += 360.0;
    if (az_deg >= 360.0) az_deg -= 360.0;
    
    int sector = static_cast<int>(az_deg / 11.25);
    if (sector >= NUM_SECTORS) sector = NUM_SECTORS - 1;
    return sector;
}

TrackPool::Stats TrackPool::get_stats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Stats stats;
    stats.total_tracks = tracks_.size();
    
    for (const auto& [id, track] : tracks_) {
        if (track.state == TrackState::ACTIVE) {
            stats.active_tracks++;
        }
        if (track.is_confirmed()) {
            stats.confirmed_tracks++;
        }
        if (track.state == TrackState::COASTING) {
            stats.coasting_tracks++;
        }
    }
    
    for (int i = 0; i < NUM_SECTORS; ++i) {
        stats.tracks_by_sector[i] = tracks_by_sector_[i].size();
    }
    
    return stats;
}

} // namespace radar
} // namespace vrl
