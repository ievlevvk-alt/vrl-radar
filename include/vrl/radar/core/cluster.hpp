// include/vrl/radar/core/cluster.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>
#include <iostream>  // <-- ДОБАВЛЯЕМ для отладочной печати

namespace vrl {
namespace radar {

class Cluster {
public:
    Cluster() = default;
    
    // === Управление ID ===
    uint64_t get_id() const { return id_; }
    void set_id(uint64_t id) { id_ = id; }
    
    // === Управление отладкой ===
    void enable_debug(bool enable) { debug_enabled_ = enable; }
    bool is_debug_enabled() const { return debug_enabled_; }
    
    // === Управление точками ===
    void add_point(size_t point_index);
    void remove_points(const std::vector<size_t>& positions);
    void clear();
    
    // === Доступ к данным ===
    size_t size() const { return indices_.size(); }
    bool is_empty() const { return indices_.empty(); }
    const std::vector<size_t>& get_indices() const { return indices_; }
    size_t get_point_index(size_t position) const { return indices_[position]; }
    
    // === Геометрия ===
    uint16_t get_min_range() const { return min_range_; }
    uint16_t get_max_range() const { return max_range_; }
    int get_azimuth_span() const { return azimuth_span_; }
    bool has_rbs() const { return has_rbs_; }
    bool has_uvd() const { return has_uvd_; }
    bool is_mixed() const { return has_rbs_ && has_uvd_; }
    
    // === Управление состоянием ===
    void close();
    bool is_closed() const { return closed_; }
    uint16_t get_last_azimuth() const { return last_azimuth_; }

    // === Связи с треками ===
    std::vector<uint64_t> candidate_track_ids;    


private:
    void recalculate_statistics();
    
    std::vector<size_t> indices_;
    
    uint16_t min_range_{65535};
    uint16_t max_range_{0};
    int azimuth_span_{0};
    uint16_t last_azimuth_{0};
    bool has_rbs_{false};
    bool has_uvd_{false};
    bool closed_{false};
    
    // НОВЫЕ ПОЛЯ
    uint64_t id_{0};
    bool debug_enabled_{false};

};

} // namespace radar
} // namespace vrl
