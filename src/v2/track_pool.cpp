// src/v2/track_pool.cpp
#include "vrl/radar/v2/track_pool.hpp"
#include "vrl/radar/utils/logger.h"
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {
namespace v2 {

// ============================================================================
// СИНГЛТОН
// ============================================================================

TrackPool& TrackPool::instance() {
    static TrackPool instance;
    return instance;
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ
// ============================================================================

void TrackPool::init(size_t max_tracks) {
    if (initialized_) {
        VRL_LOG_WARN(modules::CORE, "TrackPool already initialized");
        return;
    }
    
    if (max_tracks == 0) {
        VRL_LOG_ERROR(modules::CORE, "max_tracks must be > 0");
        return;
    }
    
    max_tracks_ = max_tracks;
    tracks_.resize(max_tracks_);
    
    for (size_t i = 0; i < max_tracks_; ++i) {
        tracks_[i].id = static_cast<uint64_t>(i + 1);
        tracks_[i].reset();
    }
    
    next_slot_ = 0;
    initialized_ = true;
    
    VRL_LOG_INFO(modules::CORE, "TrackPool initialized with " + 
                 std::to_string(max_tracks_) + " slots");
}

// ============================================================================
// ПРОВЕРКА ID
// ============================================================================

bool TrackPool::is_valid_id(uint64_t id) const {
    if (id == 0) return false;
    size_t slot = static_cast<size_t>(id - 1);
    return slot < max_tracks_;
}

// ============================================================================
// СОЗДАНИЕ ТРЕКА
// ============================================================================

Track* TrackPool::create_track() {
    if (!initialized_) {
        VRL_LOG_ERROR(modules::CORE, "TrackPool not initialized");
        return nullptr;
    }
    
    // Берём следующий слот (циклический буфер)
    size_t slot = next_slot_ % max_tracks_;
    next_slot_ = (next_slot_ + 1) % max_tracks_;
    
    Track& track = tracks_[slot];
    track.reset();
    track.id = static_cast<uint64_t>(slot + 1);
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    std::cout << "[TrackPool] create_track: id=" << track.id 
              << ", slot=" << slot << std::endl;
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Track created, id=" + 
                  std::to_string(track.id) + ", slot=" + std::to_string(slot));
    
    return &track;
}

// ============================================================================
// ПОЛУЧЕНИЕ ТРЕКА ПО ID
// ============================================================================

Track* TrackPool::get_track(uint64_t id) {
    if (!is_valid_id(id)) {
        return nullptr;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    Track& track = tracks_[slot];
    
    // Проверяем, что ID соответствует слоту
    if (track.id != id) {
        return nullptr;
    }
    
    return &track;
}

const Track* TrackPool::get_track(uint64_t id) const {
    if (!is_valid_id(id)) {
        return nullptr;
    }
    
    size_t slot = static_cast<size_t>(id - 1);
    const Track& track = tracks_[slot];
    
    if (track.id != id) {
        return nullptr;
    }
    
    return &track;
}

// ============================================================================
// ПОЛУЧЕНИЕ ВСЕХ ТРЕКОВ
// ============================================================================

std::vector<Track*> TrackPool::get_all_tracks() {
    std::vector<Track*> result;
    result.reserve(max_tracks_);
    for (size_t i = 0; i < max_tracks_; ++i) {
        result.push_back(&tracks_[i]);
    }
    return result;
}

std::vector<const Track*> TrackPool::get_all_tracks() const {
    std::vector<const Track*> result;
    result.reserve(max_tracks_);
    for (size_t i = 0; i < max_tracks_; ++i) {
        result.push_back(&tracks_[i]);
    }
    return result;
}

// ============================================================================
// ОЧИСТКА
// ============================================================================

void TrackPool::clear() {
#ifdef CMAKE_BUILD_TYPE_DEBUG
    std::cout << "[TrackPool] clear: called" << std::endl;
#endif
    
    for (size_t i = 0; i < max_tracks_; ++i) {
        tracks_[i].reset();
        tracks_[i].id = static_cast<uint64_t>(i + 1);
    }
    
    next_slot_ = 0;
    
    VRL_LOG_INFO(modules::CORE, "TrackPool cleared");
}

} // namespace v2
} // namespace radar
} // namespace vrl
