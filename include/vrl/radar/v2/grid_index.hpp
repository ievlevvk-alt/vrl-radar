// include/vrl/radar/v2/grid_index.hpp
#pragma once

#include "grid_config.hpp"
#include <vector>
#include <cstdint>
#include <cstddef>
#include <unordered_map>

namespace vrl {
namespace radar {
namespace v2 {

/**
 * @brief Ключ ячейки сетки
 */
struct GridCellKey {
    int x;
    int y;
    
    bool operator==(const GridCellKey& other) const {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief Хеш для ключа ячейки
 */
struct GridCellKeyHash {
    size_t operator()(const GridCellKey& key) const {
        return std::hash<int>()(key.x) ^ (std::hash<int>()(key.y) << 1);
    }
};

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
    
    // === Внутренние методы (публичные для тестирования) ===
    
    /**
     * @brief Получить ключ ячейки для координат
     */
    GridCellKey get_cell_key(double x, double y) const;
    
    /**
     * @brief Проверить, находится ли ячейка в пределах дальности
     */
    bool is_in_range(const GridCellKey& key) const;
    
    /**
     * @brief Получить количество колец для заданной дальности
     */
    int get_rings_for_range(double range_m) const;
    
    /**
     * @brief Получить соседние ячейки
     */
    std::vector<GridCellKey> get_neighbor_cells(const GridCellKey& center, int rings) const;
    
    // === Очистка ===
    void clear();
    
    // === Статистика ===
    struct Stats {
        size_t total_tracks{0};
        size_t non_empty_cells{0};
    };
    Stats get_stats() const;

private:
    using CellMap = std::unordered_map<GridCellKey, std::vector<uint64_t>, GridCellKeyHash>;
    using TrackMap = std::unordered_map<uint64_t, GridCellKey>;
    
    GridConfig config_;
    double cell_size_m_;
    double max_range_m_;
    
    CellMap grid_;
    TrackMap track_to_cell_;
};

} // namespace v2
} // namespace radar
} // namespace vrl
