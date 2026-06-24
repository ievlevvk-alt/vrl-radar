// src/v2/grid_index.cpp
#include "vrl/radar/v2/grid_index.hpp"
#include <cmath>
#include <algorithm>

namespace vrl {
namespace radar {
namespace v2 {

GridIndex::GridIndex(const GridConfig& config)
    : config_(config)
    , cell_size_m_(config.cell_size_km * 1000.0)
    , max_range_m_(config.max_range_km * 1000.0) {}

// ============================================================================
// ВНУТРЕННИЕ МЕТОДЫ
// ============================================================================

GridCellKey GridIndex::get_cell_key(double x, double y) const {
    int gx = static_cast<int>(std::floor(x / cell_size_m_));
    int gy = static_cast<int>(std::floor(y / cell_size_m_));
    return {gx, gy};
}

bool GridIndex::is_in_range(const GridCellKey& key) const {
    // Центр ячейки
    double cx = (static_cast<double>(key.x) + 0.5) * cell_size_m_;
    double cy = (static_cast<double>(key.y) + 0.5) * cell_size_m_;
    double range = std::sqrt(cx * cx + cy * cy);
    return range <= max_range_m_;
}

int GridIndex::get_rings_for_range(double range_m) const {
    double range_km = range_m / 1000.0;
    if (range_km < config_.far_threshold_km) {
        return config_.rings_near;
    }
    return config_.rings_far;
}

std::vector<GridCellKey> GridIndex::get_neighbor_cells(
    const GridCellKey& center, int rings) const {
    
    std::vector<GridCellKey> result;
    result.reserve((2 * rings + 1) * (2 * rings + 1));
    
    for (int dx = -rings; dx <= rings; ++dx) {
        for (int dy = -rings; dy <= rings; ++dy) {
            GridCellKey key{center.x + dx, center.y + dy};
            if (is_in_range(key)) {
                result.push_back(key);
            }
        }
    }
    
    return result;
}

// ============================================================================
// УПРАВЛЕНИЕ ТРЕКАМИ
// ============================================================================

void GridIndex::add_track(uint64_t track_id, double x, double y) {
    GridCellKey key = get_cell_key(x, y);
    
    if (!is_in_range(key)) {
        return;  // за пределами дальности
    }
    
    grid_[key].push_back(track_id);
    track_to_cell_[track_id] = key;
}

void GridIndex::update_track(uint64_t track_id, double x, double y) {
    auto it = track_to_cell_.find(track_id);
    if (it == track_to_cell_.end()) {
        add_track(track_id, x, y);
        return;
    }
    
    GridCellKey old_key = it->second;
    GridCellKey new_key = get_cell_key(x, y);
    
    if (old_key == new_key) {
        return;  // не изменилась
    }
    
    // Удаляем из старой ячейки
    auto& vec = grid_[old_key];
    auto vec_it = std::find(vec.begin(), vec.end(), track_id);
    if (vec_it != vec.end()) {
        vec.erase(vec_it);
    }
    if (vec.empty()) {
        grid_.erase(old_key);
    }
    
    // Добавляем в новую
    track_to_cell_[track_id] = new_key;
    grid_[new_key].push_back(track_id);
}

void GridIndex::remove_track(uint64_t track_id) {
    auto it = track_to_cell_.find(track_id);
    if (it == track_to_cell_.end()) {
        return;
    }
    
    GridCellKey key = it->second;
    auto& vec = grid_[key];
    auto vec_it = std::find(vec.begin(), vec.end(), track_id);
    if (vec_it != vec.end()) {
        vec.erase(vec_it);
    }
    if (vec.empty()) {
        grid_.erase(key);
    }
    
    track_to_cell_.erase(it);
}

bool GridIndex::has_track(uint64_t track_id) const {
    return track_to_cell_.find(track_id) != track_to_cell_.end();
}

// ============================================================================
// ПОИСК
// ============================================================================

std::vector<uint64_t> GridIndex::get_nearby_tracks(double x, double y) const {
    GridCellKey center = get_cell_key(x, y);
    double range_m = std::sqrt(x * x + y * y);
    int rings = get_rings_for_range(range_m);
    return get_nearby_tracks(x, y, rings);
}

std::vector<uint64_t> GridIndex::get_nearby_tracks(double x, double y, int rings) const {
    GridCellKey center = get_cell_key(x, y);
    auto cells = get_neighbor_cells(center, rings);
    
    std::vector<uint64_t> result;
    
    for (const auto& key : cells) {
        auto it = grid_.find(key);
        if (it != grid_.end()) {
            result.insert(result.end(), it->second.begin(), it->second.end());
        }
    }
    
    // Удаляем дубликаты
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    
    return result;
}

std::vector<uint64_t> GridIndex::get_all_tracks() const {
    std::vector<uint64_t> result;
    result.reserve(track_to_cell_.size());
    for (const auto& [id, key] : track_to_cell_) {
        result.push_back(id);
    }
    return result;
}

// ============================================================================
// ОЧИСТКА И СТАТИСТИКА
// ============================================================================

void GridIndex::clear() {
    grid_.clear();
    track_to_cell_.clear();
}

GridIndex::Stats GridIndex::get_stats() const {
    Stats stats;
    stats.total_tracks = track_to_cell_.size();
    stats.non_empty_cells = grid_.size();
    return stats;
}

} // namespace v2
} // namespace radar
} // namespace vrl
