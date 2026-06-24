// include/vrl/radar/v2/grid_index.hpp
#pragma once

#include "grid_config.hpp"
#include <vector>
#include <cstdint>
#include <cstddef>          // <-- ДОБАВЛЕНО для size_t
#include <unordered_map>

namespace vrl {
namespace radar {
namespace v2 {

/**
 * @brief Пространственный индекс на основе квадратной сетки
 * 
 * Позволяет быстро находить треки в окрестности заданной точки.
 */
class GridIndex {
public:
    explicit GridIndex(const GridConfig& config);
    ~GridIndex() = default;
    
    // === Управление треками ===
    
    /**
     * @brief Добавить трек в индекс
     */
    void add_track(uint64_t track_id, double x, double y);
    
    /**
     * @brief Обновить позицию трека
     */
    void update_track(uint64_t track_id, double x, double y);
    
    /**
     * @brief Удалить трек из индекса
     */
    void remove_track(uint64_t track_id);
    
    /**
     * @brief Проверить, есть ли трек в индексе
     */
    bool has_track(uint64_t track_id) const;
    
    // === Поиск ===
    
    /**
     * @brief Получить ID треков в окрестности точки
     * @param x координата X (метры)
     * @param y координата Y (метры)
     * @return вектор ID треков
     */
    std::vector<uint64_t> get_nearby_tracks(double x, double y) const;
    
    /**
     * @brief Получить ID треков в окрестности точки с указанным числом колец
     */
    std::vector<uint64_t> get_nearby_tracks(double x, double y, int rings) const;
    
    /**
     * @brief Получить все ID треков в индексе
     */
    std::vector<uint64_t> get_all_tracks() const;
    
    // === Очистка ===
    void clear();
    
    // === Статистика ===
    struct Stats {
        size_t total_tracks{0};
        size_t non_empty_cells{0};
    };
    Stats get_stats() const;

private:
    struct CellKey {
        int x;
        int y;
        
        bool operator==(const CellKey& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    struct CellKeyHash {
        // ИСПРАВЛЕНО: operator() должен быть public (по умолчанию public в struct)
        size_t operator()(const CellKey& key) const {
            return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
        }
    };
    
    CellKey get_cell_key(double x, double y) const;
    bool is_in_range(const CellKey& key) const;
    int get_rings_for_range(double range_m) const;
    std::vector<CellKey> get_neighbor_cells(const CellKey& center, int rings) const;
    
    GridConfig config_;
    double cell_size_m_;
    double max_range_m_;
    
    std::unordered_map<CellKey, std::vector<uint64_t>, CellKeyHash> grid_;
    std::unordered_map<uint64_t, CellKey> track_to_cell_;
};

} // namespace v2
} // namespace radar
} // namespace vrl
