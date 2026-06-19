// include/vrl/radar/processing/tracker.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "i_tracker_filter.h"
#include "kalman_filter.h"
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <cstddef>

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
    /**
     * @brief Конструктор с конфигурацией
     * @param config конфигурация трекера
     */
    explicit TrackManager(const TrackerConfig& config = TrackerConfig());
    
    /**
     * @brief Конструктор с пользовательским фильтром
     * @param config конфигурация трекера
     * @param filter уникальный указатель на фильтр
     */
    explicit TrackManager(const TrackerConfig& config, 
                          std::unique_ptr<ITrackerFilter> filter);
    
    ~TrackManager() = default;
    
    /**
     * @brief Обработать цели на текущем обороте
     * @param targets вектор отчетов о целях
     * @param revolution номер оборота
     */
    void process_targets(const std::vector<TargetReport>& targets, uint32_t revolution);
    
    /**
     * @brief Получить активные треки
     * @return вектор треков (отсортирован по уверенности)
     */
    std::vector<Track> get_active_tracks() const;
    
    /**
     * @brief Получить подтвержденные треки
     * @return вектор подтвержденных треков
     */
    std::vector<Track> get_confirmed_tracks() const;
    
    /**
     * @brief Сбросить все треки
     */
    void reset();
    
    /**
     * @brief Включить/выключить отладочный режим
     */
    void set_debug(bool enable) { config_.debug_mode = enable; }
    
    /**
     * @brief Получить конфигурацию
     */
    const TrackerConfig& get_config() const { return config_; }
    
    /**
     * @brief Заменить фильтр на новый
     * @param filter новый фильтр
     */
    void set_filter(std::unique_ptr<ITrackerFilter> filter);
    
    /**
     * @brief Получить текущий фильтр
     */
    ITrackerFilter* get_filter() const { return default_filter_.get(); }
    
private:
    struct TrackWithFilter {
        Track track;
        std::unique_ptr<ITrackerFilter> filter;
        
        TrackWithFilter() = default;
        TrackWithFilter(const Track& t, std::unique_ptr<ITrackerFilter> f)
            : track(t), filter(std::move(f)) {}
    };
    
    void update_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void create_new_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void manage_track_states(uint32_t revolution);
    double calculate_distance(const TargetReport& target, const Track& track) const;
    bool is_code_match(const TargetReport& target, const Track& track) const;
    double calculate_azimuth_diff(double az1, double az2) const;
    
    /**
     * @brief Создать новый фильтр для трека
     */
    std::unique_ptr<ITrackerFilter> create_filter() const;
    
    TrackerConfig config_;
    uint64_t next_id_{1};
    std::map<uint64_t, TrackWithFilter> tracks_;
    
    // Фильтр по умолчанию (используется для создания новых)
    std::unique_ptr<ITrackerFilter> default_filter_;
};

} // namespace radar
} // namespace vrl
