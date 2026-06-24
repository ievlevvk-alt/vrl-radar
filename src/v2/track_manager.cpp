// src/v2/track_manager.cpp
#include "vrl/radar/v2/track_manager.hpp"
#include "vrl/radar/v2/plot_pool.hpp"
#include "vrl/radar/utils/logger.h"
#include <cmath>
#include <algorithm>

using namespace vrl::radar::utils;
using ::vrl::radar::Cluster;

namespace vrl {
namespace radar {
namespace v2 {

static constexpr int NUM_SECTORS = 32;
static constexpr int SECTOR_SIZE = 4096 / NUM_SECTORS;

// ============================================================================
// КОНСТРУКТОР И ИНИЦИАЛИЗАЦИЯ
// ============================================================================

TrackManager::TrackManager()
    : track_pool_(TrackPool::instance())
    , grid_index_(GridConfig())
    , cluster_pool_(ClusterPool::instance()) {}

void TrackManager::init(const GridConfig& config) {
    if (initialized_) {
        VRL_LOG_WARN(modules::TRACKER, "TrackManager already initialized");
        return;
    }
    
    config_ = config;
    track_pool_.init(8192);
    PlotPool::instance().init(8192);
    grid_index_ = GridIndex(config_);
    
    for (auto& vec : tracks_by_sector_) {
        vec.clear();
    }
    
    updated_track_ids_.clear();
    tracks_to_clear_flag_.clear();
    
    current_sector_ = 0;
    last_processed_sector_ = -1;
    global_maia_counter_ = 0;
    previous_azimuth_ = 0;
    
    initialized_ = true;
    
    VRL_LOG_INFO(modules::TRACKER, "TrackManager initialized");
}

// ============================================================================
// ОСНОВНОЙ МЕТОД
// ============================================================================

void TrackManager::process_azimuth(uint16_t azimuth_maia) {
    if (!initialized_) {
        VRL_LOG_ERROR(modules::TRACKER, "TrackManager not initialized");
        return;
    }
    
    if (azimuth_maia < previous_azimuth_) {
        global_maia_counter_ += 4096;
    }
    previous_azimuth_ = azimuth_maia;
    current_azimuth_ = azimuth_maia;
    
    int new_sector = get_sector_from_azimuth(azimuth_maia);
    
    if (last_processed_sector_ == -1) {
        // Первый вызов
        int delayed_sector = get_delayed_sector(new_sector);
        int coast_sector = (new_sector - 6 + NUM_SECTORS) % NUM_SECTORS;
        
        clear_updated_flags_for_sector((new_sector - 4 + NUM_SECTORS) % NUM_SECTORS);
        process_coasted_tracks(coast_sector);
        process_delayed_sector(delayed_sector);
        
        last_processed_sector_ = new_sector;
    } else if (new_sector != last_processed_sector_) {
        int diff = new_sector - last_processed_sector_;
        if (diff < 0) diff += NUM_SECTORS;
        
        for (int step = 1; step <= diff; ++step) {
            int sector = (last_processed_sector_ + step) % NUM_SECTORS;
            int delayed_sector = get_delayed_sector(sector);
            int coast_sector = (sector - 6 + NUM_SECTORS) % NUM_SECTORS;
            
            clear_updated_flags_for_sector((sector - 4 + NUM_SECTORS) % NUM_SECTORS);
            process_coasted_tracks(coast_sector);
            process_delayed_sector(delayed_sector);
        }
        
        last_processed_sector_ = new_sector;
    }
    
    current_sector_ = new_sector;
    
    process_closed_clusters();
    process_wide_clusters();
}

int TrackManager::get_azimuth_from_xy(double x, double y) const {
    double az_rad = std::atan2(x, y);
    if (az_rad < 0) az_rad += 2.0 * M_PI;
    return static_cast<int>(az_rad * 4096.0 / (2.0 * M_PI));
}

void TrackManager::process_coasted_tracks(int sector_index) {
    if (sector_index < 0 || sector_index >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid sector index: " + std::to_string(sector_index));
        return;
    }
    
    const auto& track_ids = tracks_by_sector_[sector_index];
    if (track_ids.empty()) return;
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing coasted tracks in sector " +
                  std::to_string(sector_index) + ", count=" + std::to_string(track_ids.size()));
    
    std::vector<uint64_t> to_remove;
    
    for (uint64_t track_id : track_ids) {
        Track* track = track_pool_.get_track(track_id);
        if (!track || track->state == 3) continue;
        
        // Если трек уже обновлён в этом обороте — пропускаем
        if (track->updated_in_current_sector) continue;
        
        // Увеличиваем счётчик пропусков
        track->coast_count++;
        
        // Проверяем, не превышен ли лимит
        if (track->coast_count >= static_cast<uint32_t>(config_.max_coast_revolutions)) {
            VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(track_id) +
                          " dropped (coast_count=" + std::to_string(track->coast_count) +
                          ", max=" + std::to_string(config_.max_coast_revolutions) + ")");
            to_remove.push_back(track_id);
            continue;
        }
        
        // Прогнозируем позицию на один оборот вперёд
        auto [pred_x, pred_y] = predict_position(*track, 4096);
        
        // Обновляем GridIndex с прогнозируемой позицией
        grid_index_.update_track(track->id, pred_x, pred_y);
        
        // Вычисляем прогнозируемый азимут и обновляем сектор
        int pred_az_maia = get_azimuth_from_xy(pred_x, pred_y);
        int new_sector = get_sector_from_azimuth(static_cast<uint16_t>(pred_az_maia));
        update_track_sector(track->id, new_sector);
        
        // Если трек был ACTIVE, переводим в COASTING
        if (track->state == 1) {
            track->state = 2;  // COASTING
            VRL_LOG_TRACE(modules::TRACKER, "Track " + std::to_string(track_id) +
                          " coasting, count=" + std::to_string(track->coast_count));
        }
    }
    
    // Удаляем устаревшие треки
    for (uint64_t track_id : to_remove) {
        remove_track(track_id);
    }
}

// ============================================================================
// СЕКТОРА
// ============================================================================

int TrackManager::get_sector_from_azimuth(uint16_t azimuth_maia) const {
    int sector = azimuth_maia / SECTOR_SIZE;
    if (sector >= NUM_SECTORS) sector = NUM_SECTORS - 1;
    return sector;
}

int TrackManager::get_delayed_sector(int current_sector) const {
    int delayed = current_sector - 2;
    if (delayed < 0) delayed += NUM_SECTORS;
    return delayed;
}

void TrackManager::add_track_to_sector(uint64_t track_id, int sector) {
    if (sector < 0 || sector >= NUM_SECTORS) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid sector: " + std::to_string(sector));
        return;
    }
    
    auto& vec = tracks_by_sector_[sector];
    if (std::find(vec.begin(), vec.end(), track_id) == vec.end()) {
        vec.push_back(track_id);
    }
}

void TrackManager::remove_track_from_sector(uint64_t track_id, int sector) {
    if (sector < 0 || sector >= NUM_SECTORS) return;
    
    auto& vec = tracks_by_sector_[sector];
    auto it = std::find(vec.begin(), vec.end(), track_id);
    if (it != vec.end()) {
        vec.erase(it);
    }
}

void TrackManager::update_track_sector(uint64_t track_id, int new_sector) {
    for (int i = 0; i < NUM_SECTORS; ++i) {
        remove_track_from_sector(track_id, i);
    }
    add_track_to_sector(track_id, new_sector);
}

bool TrackManager::is_track_in_sector(uint64_t track_id, int sector) const {
    if (sector < 0 || sector >= NUM_SECTORS) return false;
    const auto& vec = tracks_by_sector_[sector];
    return std::find(vec.begin(), vec.end(), track_id) != vec.end();
}

// ============================================================================
// ПРОГНОЗ
// ============================================================================

std::pair<double, double> TrackManager::predict_position(
    const Track& track,
    int64_t delta_maia
) const {
    double delta_seconds = static_cast<double>(delta_maia) / 4096.0 * config_.revolution_time_s;
    
    return {
        track.x + track.vx * delta_seconds,
        track.y + track.vy * delta_seconds
    };
}

// ============================================================================
// СТРОБ (ЭЛЛИПТИЧЕСКИЙ GATE)
// ============================================================================

bool TrackManager::is_in_elliptical_gate(const Track& track, const Cluster& cluster) const {
    // Прогнозируем позицию трека на азимут кластера
    auto [pred_x, pred_y] = predict_position(track, cluster.get_last_azimuth());
    
    // Центр кластера
    auto [cx, cy] = get_cluster_center(cluster);
    
    // Разница в метрах
    double dx = pred_x - cx;
    double dy = pred_y - cy;
    double distance_m = std::sqrt(dx*dx + dy*dy);
    
    // Дальность до цели
    double range_km = std::sqrt(cx*cx + cy*cy) / 1000.0;
    
    // Выбираем пороги в зависимости от дальности
    double range_gate_bins;
    double azimuth_gate_maia;
    
    if (range_km < 50.0) {
        range_gate_bins = config_.range_gate_bins_near;
        azimuth_gate_maia = config_.azimuth_gate_maia_near;
    } else if (range_km < 150.0) {
        range_gate_bins = config_.range_gate_bins_mid;
        azimuth_gate_maia = config_.azimuth_gate_maia_mid;
    } else {
        range_gate_bins = config_.range_gate_bins_far;
        azimuth_gate_maia = config_.azimuth_gate_maia_far;
    }
    
    // Расширяем строб при coasting
    if (track.coast_count > 0) {
        double expansion = 1.0 + track.coast_count * config_.coast_gate_expansion;
        range_gate_bins *= expansion;
        azimuth_gate_maia *= expansion;
    }
    
    // Проверка попадания в строб
    double range_bin_size = cluster.has_rbs() ? 30.0 : 60.0;
    double delta_range_bins = distance_m / range_bin_size;
    
    double delta_az_maia = std::abs(track.azimuth_maia - cluster.get_last_azimuth());
    if (delta_az_maia > 2048) delta_az_maia = 4096 - delta_az_maia;
    
    return (delta_range_bins <= range_gate_bins) &&
           (delta_az_maia <= azimuth_gate_maia);
}

// ============================================================================
// РАБОТА С КЛАСТЕРАМИ
// ============================================================================

std::pair<double, double> TrackManager::get_cluster_center(const Cluster& cluster) const {
    const auto& indices = cluster.get_indices();
    auto& buffer = PointBuffer::instance();
    
    if (indices.empty()) {
        return {0.0, 0.0};
    }
    
    double sum_x = 0.0, sum_y = 0.0;
    int count = 0;
    
    for (size_t idx : indices) {
        const auto& point = buffer.get_point(idx);
        double az_rad = point.azimuth * 2.0 * M_PI / 4096.0;
        double range_m = point.range * (point.is_rbs ? 30.0 : 60.0);
        
        sum_x += range_m * std::sin(az_rad);
        sum_y += range_m * std::cos(az_rad);
        count++;
    }
    
    if (count == 0) return {0.0, 0.0};
    return {sum_x / count, sum_y / count};
}

Plot::SourceType TrackManager::get_cluster_source_type(const Cluster& cluster) const {
    if (cluster.has_rbs() && !cluster.has_uvd()) {
        return Plot::SourceType::RBS;
    } else if (!cluster.has_rbs() && cluster.has_uvd()) {
        return Plot::SourceType::UVD;
    } else {
        return Plot::SourceType::MIXED;
    }
}

int TrackManager::analyze_cluster_for_targets(const Cluster& cluster) const {
    // TODO: заглушка, позже реализовать анализ
    // Пока всегда возвращаем 1
    (void)cluster;
    return 1;
}

Plot TrackManager::create_plot_from_cluster(const Cluster& cluster) const {
    Plot plot;
    
    auto [cx, cy] = get_cluster_center(cluster);
    plot.x = cx;
    plot.y = cy;
    plot.source_type = get_cluster_source_type(cluster);
    
    const auto& indices = cluster.get_indices();
    auto& buffer = PointBuffer::instance();
    
    double sum_az = 0.0, sum_range = 0.0;
    int count = 0;
    
    for (size_t idx : indices) {
        const auto& point = buffer.get_point(idx);
        sum_az += point.azimuth;
        sum_range += point.range;
        count++;
    }
    
    if (count > 0) {
        plot.azimuth_maia = static_cast<uint16_t>(sum_az / count);
        plot.range_bins = static_cast<uint16_t>(sum_range / count);
        plot.source_cluster_id = cluster.get_id();
        
        if (!indices.empty()) {
            const auto& point = buffer.get_point(indices[0]);
            if (point.is_rbs) {
                plot.mode3a_code = point.code12;
                plot.spi = point.spi;
            } else {
                plot.uvd_data20 = point.data20;
            }
        }
        
        plot.confidence = 1.0;
    }
    
    return plot;
}

double TrackManager::calculate_distance(const Track& track, const Cluster& cluster) const {
    auto [cx, cy] = get_cluster_center(cluster);
    double dx = track.x - cx;
    double dy = track.y - cy;
    return std::sqrt(dx*dx + dy*dy);
}

bool TrackManager::is_candidate(const Track& track, const Cluster& cluster) const {
    double distance_km = calculate_distance(track, cluster) / 1000.0;
    if (distance_km > config_.max_candidate_distance_km) {
        return false;
    }
    
    Plot::SourceType cluster_type = get_cluster_source_type(cluster);
    bool track_has_rbs = (track.mode3a_code != 0 || track.spi);
    bool track_has_uvd = (track.uvd_data20 != 0);
    
    if (cluster_type == Plot::SourceType::RBS && !track_has_rbs) {
        return false;
    }
    if (cluster_type == Plot::SourceType::UVD && !track_has_uvd) {
        return false;
    }
    
    return true;
}

// ============================================================================
// УПРАВЛЕНИЕ ТРЕКАМИ
// ============================================================================

void TrackManager::create_track_from_cluster(uint64_t cluster_id) {
    Cluster* cluster = cluster_pool_.get_cluster(cluster_id);
    if (!cluster || cluster->is_empty()) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid cluster " + std::to_string(cluster_id));
        return;
    }
    
    Plot plot = create_plot_from_cluster(*cluster);
    if (!plot.is_valid()) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid plot from cluster " + std::to_string(cluster_id));
        return;
    }
    
    uint64_t plot_index = PlotPool::instance().add_plot(plot);
    if (plot_index == 0) {
        VRL_LOG_ERROR(modules::TRACKER, "Failed to add plot to PlotPool");
        return;
    }
    
    Track* track = track_pool_.create_track();
    if (!track) {
        VRL_LOG_ERROR(modules::TRACKER, "Failed to create track");
        return;
    }
    
    track->x = plot.x;
    track->y = plot.y;
    track->azimuth_maia = plot.azimuth_maia;
    track->range_bins = plot.range_bins;
    track->mode3a_code = plot.mode3a_code;
    track->uvd_data20 = plot.uvd_data20;
    track->altitude = plot.altitude;
    track->spi = plot.spi;
    track->state = 0;  // NEW
    track->hit_count = 1;
    track->coast_count = 0;
    track->last_update_time = global_maia_counter_ + plot.azimuth_maia;
    track->plot_index = plot_index;
    
    grid_index_.add_track(track->id, track->x, track->y);
    int sector = get_sector_from_azimuth(track->azimuth_maia);
    add_track_to_sector(track->id, sector);
    
    updated_track_ids_.push_back(track->id);
    
    VRL_LOG_DEBUG(modules::TRACKER, "New track created: id=" +
                  std::to_string(track->id) + ", plot_index=" + std::to_string(plot_index));
}

void TrackManager::update_track_with_plot(uint64_t track_id, const Plot& plot) {
    Track* track = track_pool_.get_track(track_id);
    if (!track) {
        VRL_LOG_WARN(modules::TRACKER, "Track " + std::to_string(track_id) + " not found");
        return;
    }
    
    uint64_t plot_index = PlotPool::instance().add_plot(plot);
    if (plot_index == 0) {
        VRL_LOG_ERROR(modules::TRACKER, "Failed to add plot to PlotPool");
        return;
    }
    
    // Обновляем позицию
    track->x = plot.x;
    track->y = plot.y;
    track->azimuth_maia = plot.azimuth_maia;
    track->range_bins = plot.range_bins;
    
    // Обновляем коды
    if (plot.source_type == Plot::SourceType::RBS) {
        track->mode3a_code = plot.mode3a_code;
        track->spi = plot.spi;
    } else if (plot.source_type == Plot::SourceType::UVD) {
        track->uvd_data20 = plot.uvd_data20;
        track->altitude = plot.altitude;
    }
    
    // Обновляем состояние
    track->hit_count++;
    track->coast_count = 0;
    track->last_update_time = global_maia_counter_ + plot.azimuth_maia;
    track->plot_index = plot_index;
    track->updated_in_current_sector = true;
    track->candidate_cluster_ids.clear();
    
    if (track->hit_count >= 3 && track->state == 0) {
        track->state = 1;  // ACTIVE
        VRL_LOG_DEBUG(modules::TRACKER, "Track " + std::to_string(track_id) + " confirmed");
    }
    
    // === ИЗМЕНЕНИЕ: для обновлённого трека используем ПРОГНОЗИРУЕМЫЙ азимут ===
    // Прогнозируем позицию на текущий момент (текущий азимут)
    auto [pred_x, pred_y] = predict_position(*track, current_azimuth_);
    
    // Обновляем GridIndex с прогнозируемой позицией
    grid_index_.update_track(track->id, pred_x, pred_y);
    
    // Сектор тоже определяем по прогнозируемому азимуту
    // Для этого нужно получить прогнозируемый азимут
    // Пока используем текущий азимут для сектора (он близок к прогнозируемому)
    int new_sector = get_sector_from_azimuth(track->azimuth_maia);
    update_track_sector(track->id, new_sector);
    
    updated_track_ids_.push_back(track->id);
    
    VRL_LOG_DEBUG(modules::TRACKER, "Track updated: id=" +
                  std::to_string(track->id) + ", plot_index=" + std::to_string(plot_index) +
                  ", pred_x=" + std::to_string(pred_x) + ", pred_y=" + std::to_string(pred_y));
}

void TrackManager::remove_track(uint64_t track_id) {
    Track* track = track_pool_.get_track(track_id);
    if (!track) return;
    
    track->state = 3;  // DROPPED
    grid_index_.remove_track(track_id);
    
    for (int i = 0; i < NUM_SECTORS; ++i) {
        remove_track_from_sector(track_id, i);
    }
    
    VRL_LOG_DEBUG(modules::TRACKER, "Track removed: id=" + std::to_string(track_id));
}

// ============================================================================
// ОЧИСТКА ФЛАГОВ
// ============================================================================

void TrackManager::clear_updated_flags_for_sector(int sector) {
    auto it = tracks_to_clear_flag_.begin();
    while (it != tracks_to_clear_flag_.end()) {
        if (it->second == sector) {
            Track* track = track_pool_.get_track(it->first);
            if (track) {
                track->updated_in_current_sector = false;
            }
            it = tracks_to_clear_flag_.erase(it);
        } else {
            ++it;
        }
    }
}

void TrackManager::remove_cluster_from_track_candidates(uint64_t track_id, uint64_t cluster_id) {
    Track* track = track_pool_.get_track(track_id);
    if (!track) return;
    
    auto it = std::find(track->candidate_cluster_ids.begin(),
                        track->candidate_cluster_ids.end(),
                        cluster_id);
    if (it != track->candidate_cluster_ids.end()) {
        track->candidate_cluster_ids.erase(it);
    }
}

// ============================================================================
// ОБРАБОТКА ЗАКРЫТЫХ КЛАСТЕРОВ
// ============================================================================

void TrackManager::process_closed_clusters() {
    auto closed_ids = cluster_pool_.get_closed_clusters();
    if (closed_ids.empty()) return;
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(closed_ids.size()) +
                  " closed clusters");
    
    for (uint64_t cluster_id : closed_ids) {
        Cluster* cluster = cluster_pool_.get_cluster(cluster_id);
        if (!cluster || cluster->is_empty()) continue;
        
        auto [cx, cy] = get_cluster_center(*cluster);
        if (cx == 0.0 && cy == 0.0) continue;
        
        auto candidate_ids = grid_index_.get_nearby_tracks(cx, cy);
        
        std::vector<uint64_t> valid_candidates;
        for (uint64_t track_id : candidate_ids) {
            Track* track = track_pool_.get_track(track_id);
            if (!track || track->state == 3) continue;
            if (track->updated_in_current_sector) continue;
            if (is_candidate(*track, *cluster)) {
                valid_candidates.push_back(track_id);
            }
        }
        
        if (valid_candidates.empty()) {
            VRL_LOG_TRACE(modules::TRACKER, "No candidates for cluster " +
                          std::to_string(cluster_id) + ", processing");
            process_cluster(cluster_id);
        } else {
            for (uint64_t track_id : valid_candidates) {
                cluster->candidate_track_ids.push_back(track_id);
                Track* track = track_pool_.get_track(track_id);
                if (track) {
                    track->candidate_cluster_ids.push_back(cluster_id);
                }
            }
            
            int sector = get_sector_from_azimuth(cluster->get_last_azimuth());
            cluster_pool_.add_to_delayed(cluster_id, sector);
            
            VRL_LOG_TRACE(modules::TRACKER, "Cluster " + std::to_string(cluster_id) +
                          " delayed to sector " + std::to_string(sector) +
                          " with " + std::to_string(valid_candidates.size()) + " candidates");
        }
    }
}

// ============================================================================
// ОБРАБОТКА ЗАДЕРЖАННЫХ КЛАСТЕРОВ (ОСНОВНАЯ ЛОГИКА)
// ============================================================================

void TrackManager::process_delayed_sector(int sector_index) {
    auto delayed_ids = cluster_pool_.take_delayed_clusters(sector_index);
    if (delayed_ids.empty()) return;
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing " + std::to_string(delayed_ids.size()) +
                  " delayed clusters in sector " + std::to_string(sector_index));
    
    for (uint64_t cluster_id : delayed_ids) {
        Cluster* cluster = cluster_pool_.get_cluster(cluster_id);
        if (!cluster || cluster->is_empty()) {
            VRL_LOG_WARN(modules::TRACKER, "Invalid delayed cluster " + std::to_string(cluster_id));
            continue;
        }
        
        // Анализ кластера на число целей (заглушка → 1)
        int num_targets = analyze_cluster_for_targets(*cluster);
        (void)num_targets;  // пока не используем
        
        // === ЖЁСТКАЯ ПРОВЕРКА ПО СТРОБУ ===
        std::vector<uint64_t> valid_candidates;
        for (uint64_t track_id : cluster->candidate_track_ids) {
            Track* track = track_pool_.get_track(track_id);
            if (!track || track->state == 3) continue;
            if (track->updated_in_current_sector) continue;
            
            if (is_in_elliptical_gate(*track, *cluster)) {
                valid_candidates.push_back(track_id);
            } else {
                remove_cluster_from_track_candidates(track_id, cluster_id);
            }
        }
        
        // === ПРИНЯТИЕ РЕШЕНИЯ ===
        if (valid_candidates.empty()) {
            VRL_LOG_TRACE(modules::TRACKER, "No valid candidates for cluster " +
                          std::to_string(cluster_id) + ", creating new track");
            process_cluster(cluster_id);
        } else {
            // Пока берём первый кандидат
            uint64_t best_track_id = valid_candidates[0];
            Plot plot = create_plot_from_cluster(*cluster);
            update_track_with_plot(best_track_id, plot);
            
            // Запоминаем для очистки флага через 4 сектора
            int clear_sector = (sector_index + 4) % NUM_SECTORS;
            tracks_to_clear_flag_.push_back({best_track_id, clear_sector});
            
            VRL_LOG_TRACE(modules::TRACKER, "Cluster " + std::to_string(cluster_id) +
                          " updated track " + std::to_string(best_track_id));
        }
        
        // === ОЧИСТКА СВЯЗЕЙ ===
        for (uint64_t track_id : cluster->candidate_track_ids) {
            Track* track = track_pool_.get_track(track_id);
            if (track) {
                auto it = std::find(track->candidate_cluster_ids.begin(),
                                    track->candidate_cluster_ids.end(),
                                    cluster_id);
                if (it != track->candidate_cluster_ids.end()) {
                    track->candidate_cluster_ids.erase(it);
                }
            }
        }
        cluster->candidate_track_ids.clear();
    }
}

// ============================================================================
// ОБРАБОТКА ШИРОКИХ КЛАСТЕРОВ
// ============================================================================

void TrackManager::process_wide_clusters() {
    // TODO: реализовать позже
    VRL_LOG_TRACE(modules::TRACKER, "process_wide_clusters");
}

// ============================================================================
// ОБРАБОТКА КЛАСТЕРА (СОЗДАНИЕ НОВОГО ТРЕКА)
// ============================================================================

void TrackManager::process_cluster(uint64_t cluster_id) {
    Cluster* cluster = cluster_pool_.get_cluster(cluster_id);
    if (!cluster || cluster->is_empty()) {
        VRL_LOG_WARN(modules::TRACKER, "Invalid cluster for processing: " +
                     std::to_string(cluster_id));
        return;
    }
    
    VRL_LOG_DEBUG(modules::TRACKER, "Processing cluster " + std::to_string(cluster_id) +
                  " (size=" + std::to_string(cluster->size()) + ")");
    
    // TODO: анализ на несколько целей
    // Пока просто создаём один трек
    create_track_from_cluster(cluster_id);
}

// ============================================================================
// ДОСТУП К ДАННЫМ
// ============================================================================

Track* TrackManager::get_track(uint64_t id) {
    return track_pool_.get_track(id);
}

const Track* TrackManager::get_track(uint64_t id) const {
    return track_pool_.get_track(id);
}

std::vector<Plot> TrackManager::get_plots() const {
    std::vector<Plot> plots;
    auto all_tracks = track_pool_.get_all_tracks();
    
    for (const Track* track : all_tracks) {
        if (track && track->state != 3) {
            Plot plot;
            plot.x = track->x;
            plot.y = track->y;
            plot.azimuth_maia = track->azimuth_maia;
            plot.range_bins = track->range_bins;
            plot.mode3a_code = track->mode3a_code;
            plot.uvd_data20 = track->uvd_data20;
            plot.altitude = track->altitude;
            plot.spi = track->spi;
            plot.confidence = (track->state == 1) ? 1.0 : 0.5;
            plots.push_back(plot);
        }
    }
    
    return plots;
}

const std::vector<uint64_t>& TrackManager::get_updated_tracks() const {
    return updated_track_ids_;
}

void TrackManager::clear_updated_tracks() {
    updated_track_ids_.clear();
}

const Plot* TrackManager::get_plot(uint64_t track_id) const {
    const Track* track = track_pool_.get_track(track_id);
    if (!track) return nullptr;
    if (track->state == 2 || track->state == 3) return nullptr;
    if (track->plot_index == 0) return nullptr;
    return PlotPool::instance().get_plot(track->plot_index);
}

// ============================================================================
// УПРАВЛЕНИЕ
// ============================================================================

void TrackManager::reset() {
    track_pool_.clear();
    PlotPool::instance().clear();
    grid_index_.clear();
    
    for (auto& vec : tracks_by_sector_) {
        vec.clear();
    }
    
    updated_track_ids_.clear();
    tracks_to_clear_flag_.clear();
    
    current_sector_ = 0;
    last_processed_sector_ = -1;
    global_maia_counter_ = 0;
    previous_azimuth_ = 0;
    
    VRL_LOG_INFO(modules::TRACKER, "TrackManager reset");
}

TrackManager::Stats TrackManager::get_stats() const {
    Stats stats;
    stats.total_tracks = track_pool_.size();
    
    auto all_tracks = track_pool_.get_all_tracks();
    for (const Track* track : all_tracks) {
        if (track->state == 1) {
            stats.active_tracks++;
            if (track->hit_count >= 3) stats.confirmed_tracks++;
        } else if (track->state == 2) {
            stats.coasting_tracks++;
        }
    }
    
    return stats;
}

} // namespace v2
} // namespace radar
} // namespace vrl
