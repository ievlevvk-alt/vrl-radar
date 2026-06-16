// file: include/radar/track_manager.h
#pragma once

#include "replies.h"
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <cmath>
#include <cstdint>

namespace radar {

// Состояние трека
enum class TrackState {
    NEW,        // Новый трек (требует подтверждения)
    ACTIVE,     // Активный трек
    COASTING,   // Потерян, но еще не сброшен
    DROPPED     // Сброшен
};

// Трек цели
struct Track {
    uint64_t id{0};
    TrackState state{TrackState::NEW};
    
    // Позиция
    double x{0.0};
    double y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    // Скорость (в метрах на оборот)
    double vx{0.0};
    double vy{0.0};
    double ground_speed{0.0};
    double course_deg{0.0};
    
    // Данные цели
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    // Статистика (в оборотах)
    uint32_t first_revolution{0};
    uint32_t last_revolution{0};
    uint32_t last_update_revolution{0};
    uint32_t coast_count{0};
    uint32_t hit_count{0};
    
    // Качество
    double confidence{0.0};
    double position_error{0.0};
    
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
    
    uint32_t get_revolutions_alive() const {
        return last_revolution - first_revolution;
    }
};

// Параметры трекера
struct TrackerConfig {
    int min_hits_to_confirm{3};
    int max_coast_count{5};
    double max_gate_distance{150.0};
    double max_gate_azimuth{30.0};
    double process_noise{0.1};
    double measurement_noise{1.0};
    bool enable_uvd_tracking{true};
    bool enable_rbs_tracking{true};
    bool debug_mode{false};
};

// Калмановский фильтр для оборотов
class RevolutionKalmanFilter {
public:
    RevolutionKalmanFilter();
    RevolutionKalmanFilter(double process_noise, double measurement_noise);
    
    void init(double x, double y, uint32_t revolution);
    void predict(uint32_t delta_revolutions);
    void update(double x, double y, uint32_t revolution);
    
    double get_x() const { return x_; }
    double get_y() const { return y_; }
    double get_vx() const { return vx_; }
    double get_vy() const { return vy_; }
    double get_speed() const { return std::sqrt(vx_*vx_ + vy_*vy_); }
    double get_course() const { return std::atan2(vx_, vy_) * 180.0 / M_PI; }
    
    std::pair<double, double> predict_position(uint32_t delta_revolutions) const;
    bool is_initialized() const { return initialized_; }
    
private:
    double x_{0.0}, y_{0.0};
    double vx_{0.0}, vy_{0.0};
    double P_[4][4];
    double Q_;
    double R_;
    uint32_t last_revolution_{0};
    bool initialized_{false};
    
    void update_matrices();
};

// Менеджер треков
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
        RevolutionKalmanFilter filter;
        
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