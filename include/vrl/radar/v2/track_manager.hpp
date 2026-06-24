// include/vrl/radar/v2/track_manager.hpp
#pragma once

#include "grid_config.hpp"
#include "grid_index.hpp"
#include "plot.hpp"
#include "plot_pool.hpp"
#include "track_pool.hpp"
#include "../core/cluster.hpp"
#include "../core/cluster_pool.hpp"
#include "../core/point_buffer.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <utility>

namespace vrl {
namespace radar {
namespace v2 {

class TrackManager {
public:
    TrackManager();
    ~TrackManager() = default;
    
    // === Инициализация ===
    void init(const GridConfig& config);
    bool is_initialized() const { return initialized_; }
    
    // === Основной метод ===
    void process_azimuth(uint16_t azimuth_maia);
    
    // === Доступ к данным ===
    Track* get_track(uint64_t id);
    const Track* get_track(uint64_t id) const;
    
    std::vector<Plot> get_plots() const;
    
    // === Доступ к обновлённым трекам ===
    const std::vector<uint64_t>& get_updated_tracks() const;
    void clear_updated_tracks();
    const Plot* get_plot(uint64_t track_id) const;
    
    // === Управление ===
    void reset();
    
    // === Сохранение/загрузка ===
    uint64_t get_global_maia_counter() const { return global_maia_counter_; }
    void set_global_maia_counter(uint64_t counter) { global_maia_counter_ = counter; }
    
    // === Статистика ===
    struct Stats {
        size_t total_tracks{0};
        size_t active_tracks{0};
        size_t coasting_tracks{0};
        size_t confirmed_tracks{0};
    };
    Stats get_stats() const;

    // ========================================================================
    // === ПУБЛИЧНЫЕ МЕТОДЫ ДЛЯ ТЕСТИРОВАНИЯ ===
    // ========================================================================
    
    // Управление секторами
    void add_track_to_sector(uint64_t track_id, int sector);
    void remove_track_from_sector(uint64_t track_id, int sector);
    void update_track_sector(uint64_t track_id, int new_sector);
    bool is_track_in_sector(uint64_t track_id, int sector) const;
    int get_sector_from_azimuth(uint16_t azimuth_maia) const;
    int get_delayed_sector(int current_sector) const;
    
    // Прогноз позиции
    std::pair<double, double> predict_position(const Track& track, int64_t delta_maia) const;
    
    // Эллиптический строб
    bool is_in_elliptical_gate(const Track& track, const Cluster& cluster) const;
    
    // Обработка COASTED треков
    void process_coasted_tracks(int sector_index);
    
    // Очистка флагов
    void clear_updated_flags_for_sector(int sector);
    
    // Для доступа к внутреннему состоянию в тестах
    std::vector<std::pair<uint64_t, int>>& get_tracks_to_clear_flag() {
        return tracks_to_clear_flag_;
    }

private:
    // === Основные методы обработки ===
    void process_closed_clusters();
    void process_wide_clusters();
    void process_delayed_sector(int sector_index);
    void process_cluster(uint64_t cluster_id);
    
    // === Управление треками ===
    void create_track_from_cluster(uint64_t cluster_id);
    void update_track_with_plot(uint64_t track_id, const Plot& plot);
    void remove_track(uint64_t track_id);
    
    // Работа с кластерами
    std::pair<double, double> get_cluster_center(const Cluster& cluster) const;
    Plot::SourceType get_cluster_source_type(const Cluster& cluster) const;
    Plot create_plot_from_cluster(const Cluster& cluster) const;
    bool is_candidate(const Track& track, const Cluster& cluster) const;
    double calculate_distance(const Track& track, const Cluster& cluster) const;
    int analyze_cluster_for_targets(const Cluster& cluster) const;
    
    // Очистка связей
    void remove_cluster_from_track_candidates(uint64_t track_id, uint64_t cluster_id);
    
    // Вспомогательные методы
    int get_azimuth_from_xy(double x, double y) const;
    
    // === Компоненты ===
    TrackPool& track_pool_;
    GridIndex grid_index_;
    ClusterPool& cluster_pool_;
    
    // === Состояние ===
    uint64_t global_maia_counter_{0};
    uint16_t current_azimuth_{0};
    uint16_t previous_azimuth_{0};
    int current_sector_{0};
    int last_processed_sector_{-1};
    
    // Списки треков по секторам
    std::array<std::vector<uint64_t>, 32> tracks_by_sector_;
    
    // Список обновлённых треков для внешнего потребителя
    std::vector<uint64_t> updated_track_ids_;
    
    // Список для очистки флага updated_in_current_sector
    std::vector<std::pair<uint64_t, int>> tracks_to_clear_flag_;
    
    // Конфигурация
    GridConfig config_;
    bool initialized_{false};
    
    // Дружественный класс для тестов
    friend class TrackManagerTest;
};

} // namespace v2
} // namespace radar
} // namespace vrl
