// include/vrl/radar/processing/tracker.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "../core/track.hpp"        // <-- Добавляем
#include "../core/track_pool.hpp"   // <-- Добавляем
#include "../core/filter_types.hpp"
#include "i_tracker_filter.h"
#include "kalman_filter.h"
#include "extended_kalman_filter.h"
#include "unscented_kalman_filter.h"
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <cstddef>
#include <array>
#include <string>

namespace vrl {
namespace radar {

// Forward declaration
class Cluster;

// ============================================================================
// КЛАСС ДЛЯ ИСТОРИИ
// ============================================================================

template<typename T, size_t MaxSize>
class CircularHistory {
public:
    static constexpr size_t MAX_SIZE = MaxSize;
    
    CircularHistory() = default;
    
    bool push(const T& item) {
        if (size_ < MAX_SIZE) {
            data_[size_++] = item;
            return true;
        } else {
            data_[head_] = item;
            head_ = (head_ + 1) % MAX_SIZE;
            return true;
        }
    }
    
    bool push(T&& item) {
        if (size_ < MAX_SIZE) {
            data_[size_++] = std::move(item);
            return true;
        } else {
            data_[head_] = std::move(item);
            head_ = (head_ + 1) % MAX_SIZE;
            return true;
        }
    }
    
    std::vector<T> get_all() const {
        std::vector<T> result;
        result.reserve(size_);
        if (size_ == 0) return result;
        for (size_t i = 0; i < size_; ++i) {
            size_t idx = (head_ + i) % MAX_SIZE;
            result.push_back(data_[idx]);
        }
        return result;
    }
    
    const T* back() const {
        if (size_ == 0) return nullptr;
        size_t idx = (head_ + size_ - 1) % MAX_SIZE;
        return &data_[idx];
    }
    
    const T* front() const {
        if (size_ == 0) return nullptr;
        return &data_[head_];
    }
    
    const T* get(size_t index) const {
        if (index >= size_) return nullptr;
        size_t idx = (head_ + index) % MAX_SIZE;
        return &data_[idx];
    }
    
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    constexpr size_t max_size() const { return MAX_SIZE; }
    void clear() { size_ = 0; head_ = 0; }
    bool is_full() const { return size_ == MAX_SIZE; }
    
    // Итераторы
    class Iterator {
    public:
        Iterator(const CircularHistory* history, size_t pos)
            : history_(history), pos_(pos) {}
        Iterator& operator++() {
            if (pos_ < history_->size_) pos_++;
            return *this;
        }
        const T& operator*() const {
            size_t idx = (history_->head_ + pos_) % history_->MAX_SIZE;
            return history_->data_[idx];
        }
        bool operator!=(const Iterator& other) const {
            return pos_ != other.pos_ || history_ != other.history_;
        }
    private:
        const CircularHistory* history_;
        size_t pos_;
    };
    
    Iterator begin() const { return Iterator(this, 0); }
    Iterator end() const { return Iterator(this, size_); }
    
private:
    std::array<T, MAX_SIZE> data_{};
    size_t head_{0};
    size_t size_{0};
};

// ============================================================================
// TRACK MANAGER
// ============================================================================

class TrackManager {
public:
    explicit TrackManager(const TrackerConfig& config = TrackerConfig());
    explicit TrackManager(const TrackerConfig& config, 
                          std::unique_ptr<ITrackerFilter> filter);
    ~TrackManager() = default;
    
    // --- НОВЫЕ МЕТОДЫ ДЛЯ РАБОТЫ С КЛАСТЕРАМИ ---
    
    /**
     * @brief Обработать закрытые кластеры из сектора
     * @param sector_index индекс сектора
     * @param cluster_ids ID кластеров для обработки
     */
    void process_clusters_in_sector(int sector_index, 
                                    const std::vector<uint64_t>& cluster_ids);
    
    /**
     * @brief Обработать широкие кластеры
     * @param cluster_ids ID широких кластеров
     */
    void process_wide_clusters(const std::vector<uint64_t>& cluster_ids);
    
    std::vector<Track*> get_tracks_in_sector(int sector_index);
    
    // --- СУЩЕСТВУЮЩИЕ МЕТОДЫ ---
    
    void process_targets(const std::vector<TargetReport>& targets, uint32_t revolution);
    std::vector<Track> get_active_tracks() const;
    std::vector<Track> get_confirmed_tracks() const;
    void reset();
    void set_debug(bool enable) { config_.debug_mode = enable; }
    const TrackerConfig& get_config() const { return config_; }
    void set_filter(std::unique_ptr<ITrackerFilter> filter);
    ITrackerFilter* get_filter() const { return default_filter_.get(); }
    
    void set_filter_type(FilterType type);
    FilterType get_filter_type() const { return filter_type_; }
    void set_max_history_size(size_t max_size) { max_history_size_ = max_size; }
    size_t get_max_history_size() const { return max_history_size_; }
    
    std::string get_filter_name() const;

private:
    struct TrackWithFilter {
        Track track;
        std::unique_ptr<ITrackerFilter> filter;
        
        TrackWithFilter() = default;
        TrackWithFilter(const Track& t, std::unique_ptr<ITrackerFilter> f)
            : track(t), filter(std::move(f)) {}
    };
    
    // --- Вспомогательные методы ---
    Track* find_best_track(const Cluster& cluster, 
                           const std::vector<Track*>& tracks);
    
    void update_track_with_cluster(Track& track, const Cluster& cluster);
    void create_track_from_cluster(const Cluster& cluster, int sector);
    std::vector<TargetReport> split_wide_cluster(const Cluster& cluster);
    bool is_valid_cluster(const Cluster& cluster) const;
    bool is_valid_report(const TargetReport& report) const;
    
    TargetReport build_report_from_cluster(const Cluster& cluster);
    double calculate_distance(const TargetReport& target, const Track& track) const;
    double calculate_distance(const Cluster& cluster, const Track& track) const;
    bool is_code_match(const TargetReport& target, const Track& track) const;
    bool is_code_match(const Cluster& cluster, const Track& track) const;
    double calculate_azimuth_diff(double az1, double az2) const;
    
    std::unique_ptr<ITrackerFilter> create_filter(FilterType type) const;
    std::unique_ptr<ITrackerFilter> create_default_filter() const;
    
    TrackerConfig config_;
    uint64_t next_id_{1};
    std::map<uint64_t, TrackWithFilter> tracks_;
    
    std::unique_ptr<ITrackerFilter> default_filter_;
    FilterType filter_type_{FilterType::KALMAN};
    size_t max_history_size_{Track::DEFAULT_MAX_HISTORY};
    
    TrackPool& track_pool_;
};

} // namespace radar
} // namespace vrl
