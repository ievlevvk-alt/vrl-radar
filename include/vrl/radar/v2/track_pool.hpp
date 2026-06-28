// include/vrl/radar/v2/track_pool.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include <memory>
#include "kalman_filter.hpp"

namespace vrl {
namespace radar {
namespace v2 {

struct Track {
    uint64_t id{0};
    
    double x{0.0}, y{0.0};
    double vx{0.0}, vy{0.0};
    uint16_t azimuth_maia{0};
    uint16_t range_bins{0};
    
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    uint8_t state{0};
    uint32_t hit_count{0};
    uint32_t coast_count{0};
    
    uint64_t last_update_time{0};
    
    uint32_t status{0};
    bool code_confidence{false};
    bool altitude_confidence{false};
    
    bool updated_in_current_sector{false};
    
    uint64_t plot_index{0};
    
    std::vector<uint64_t> candidate_cluster_ids;
    
    std::unique_ptr<KalmanFilter> filter;
    
    Track() = default;
    
    // Конструктор копирования
    Track(const Track& other)
        : id(other.id)
        , x(other.x), y(other.y)
        , vx(other.vx), vy(other.vy)
        , azimuth_maia(other.azimuth_maia)
        , range_bins(other.range_bins)
        , mode3a_code(other.mode3a_code)
        , uvd_data20(other.uvd_data20)
        , altitude(other.altitude)
        , spi(other.spi)
        , state(other.state)
        , hit_count(other.hit_count)
        , coast_count(other.coast_count)
        , last_update_time(other.last_update_time)
        , status(other.status)
        , code_confidence(other.code_confidence)
        , altitude_confidence(other.altitude_confidence)
        , updated_in_current_sector(other.updated_in_current_sector)
        , plot_index(other.plot_index)
        , candidate_cluster_ids(other.candidate_cluster_ids)
    {
        if (other.filter && other.filter->is_initialized()) {
            filter = other.filter->clone();
        }
    }
    
    // Оператор присваивания
    Track& operator=(const Track& other) {
        if (this != &other) {
            id = other.id;
            x = other.x; y = other.y;
            vx = other.vx; vy = other.vy;
            azimuth_maia = other.azimuth_maia;
            range_bins = other.range_bins;
            mode3a_code = other.mode3a_code;
            uvd_data20 = other.uvd_data20;
            altitude = other.altitude;
            spi = other.spi;
            state = other.state;
            hit_count = other.hit_count;
            coast_count = other.coast_count;
            last_update_time = other.last_update_time;
            status = other.status;
            code_confidence = other.code_confidence;
            altitude_confidence = other.altitude_confidence;
            updated_in_current_sector = other.updated_in_current_sector;
            plot_index = other.plot_index;
            candidate_cluster_ids = other.candidate_cluster_ids;
            
            if (other.filter && other.filter->is_initialized()) {
                filter = other.filter->clone();
            } else {
                filter.reset();
            }
        }
        return *this;
    }
    
    // Конструктор перемещения
    Track(Track&& other) noexcept = default;
    
    // Оператор присваивания перемещением
    Track& operator=(Track&& other) noexcept = default;
    
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
        if (filter) {
            filter->reset();
        }
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
