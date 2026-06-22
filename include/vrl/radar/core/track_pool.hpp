// include/vrl/radar/core/track_pool.hpp
#pragma once

#include "track.hpp"          // <-- Вместо tracker.h
#include "types.h"
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace vrl {
namespace radar {

// Forward declaration
class TargetReport;

/**
 * @brief Пул для хранения треков
 * 
 * Хранит треки и обеспечивает быстрый доступ по секторам.
 * Треки распределяются по секторам на основе прогнозируемой позиции.
 */
class TrackPool {
public:
    static constexpr int NUM_SECTORS = 32;
    
    static TrackPool& instance();
    
    // --- Создание трека ---
    uint64_t create_track(const TargetReport& report, int sector, FilterType filter_type = FilterType::KALMAN);

    // --- Получение треков ---
    Track* get_track(uint64_t id);
    const Track* get_track(uint64_t id) const;
    
    /**
     * @brief Получить треки, прогнозируемые в секторе
     * @param sector_index индекс сектора
     * @return вектор указателей на треки
     */
    std::vector<Track*> get_tracks_in_sector(int sector_index);
    
    /**
     * @brief Получить все активные треки
     */
    std::vector<Track*> get_all_tracks();
    
    // --- Обновление трека ---
    void update_track(uint64_t id, const TargetReport& report, int sector);
    
    // --- Управление ---
    void remove_track(uint64_t id);
    void clear();
    size_t size() const { return tracks_.size(); }
    
    // --- Статистика ---
    struct Stats {
        size_t total_tracks{0};
        size_t active_tracks{0};
        size_t confirmed_tracks{0};
        size_t coasting_tracks{0};
        size_t tracks_by_sector[NUM_SECTORS]{};
    };
    
    Stats get_stats() const;

private:
    TrackPool() = default;
    ~TrackPool() = default;
    TrackPool(const TrackPool&) = delete;
    TrackPool& operator=(const TrackPool&) = delete;
    
    // Вспомогательные методы
    void add_to_sector(uint64_t track_id, int sector);
    void remove_from_sector(uint64_t track_id, int sector);
    int get_sector_for_track(const Track& track) const;
    
    // Хранилище треков
    std::unordered_map<uint64_t, Track> tracks_;
    
    // Треки по секторам (по прогнозируемой позиции)
    std::array<std::vector<uint64_t>, NUM_SECTORS> tracks_by_sector_;
    
    // Для каждого трека храним текущий сектор
    std::unordered_map<uint64_t, int> track_sectors_;
    
    mutable std::mutex mutex_;
    uint64_t next_id_{1};
};

} // namespace radar
} // namespace vrl
