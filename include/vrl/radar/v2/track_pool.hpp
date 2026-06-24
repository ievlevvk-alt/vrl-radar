// include/vrl/radar/v2/track_pool.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>

namespace vrl {
namespace radar {
namespace v2 {

struct Track {
    uint64_t id{0};
    
    // Позиция и движение
    double x{0.0}, y{0.0};
    double vx{0.0}, vy{0.0};
    uint16_t azimuth_maia{0};
    uint16_t range_bins{0};
    
    // Коды и данные
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    // Состояние
    uint8_t state{0};           // 0=NEW, 1=ACTIVE, 2=COASTING, 3=DROPPED
    uint32_t hit_count{0};
    uint32_t coast_count{0};
    
    // Время последнего обновления
    uint64_t last_update_time{0};
    
    // Статус и уверенность
    uint32_t status{0};
    bool code_confidence{false};
    bool altitude_confidence{false};
    
    // === НОВОЕ: флаг обновления в текущем обороте ===
    bool updated_in_current_sector{false};
    
    // === НОВОЕ: индекс плота ===
    uint64_t plot_index{0};
    
    // Связи с кластерами
    std::vector<uint64_t> candidate_cluster_ids;
    
    void reset() {
        x = 0.0; y = 0.0;
        vx = 0.0; vy = 0.0;
        azimuth_maia = 0;
        range_bins = 0;
        mode3a_code = 0;
        uvd_data20 = 0;
        altitude = 0;
        spi = false;
        state = 0;
        hit_count = 0;
        coast_count = 0;
        last_update_time = 0;
        status = 0;
        code_confidence = false;
        altitude_confidence = false;
        updated_in_current_sector = false;
        plot_index = 0;
        candidate_cluster_ids.clear();
    }
};

class TrackPool {
public:
    static TrackPool& instance();
    
    void init(size_t max_tracks = 8192);
    bool is_initialized() const { return initialized_; }
    
    Track* create_track();
    Track* get_track(uint64_t id);
    const Track* get_track(uint64_t id) const;
    
    size_t size() const { return tracks_.size(); }
    
    std::vector<Track*> get_all_tracks();
    std::vector<const Track*> get_all_tracks() const;
    
    void clear();

private:
    TrackPool() = default;
    ~TrackPool() = default;
    TrackPool(const TrackPool&) = delete;
    TrackPool& operator=(const TrackPool&) = delete;
    
    bool is_valid_id(uint64_t id) const;
    
    std::vector<Track> tracks_;
    size_t max_tracks_{8192};
    size_t next_slot_{0};
    bool initialized_{false};
};

} // namespace v2
} // namespace radar
} // namespace vrl
