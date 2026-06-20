// include/vrl/radar/core/cluster.hpp
#pragma once

#include <vector>
#include <cstdint>
#include <algorithm>

namespace vrl {
namespace radar {

/**
 * @brief Кластер — группа точек, объединенных по пространственному признаку
 * 
 * Хранит индексы точек в PointBuffer в порядке добавления.
 * Поддерживает удаление точек и пересчет статистики.
 */
class Cluster {
public:
    Cluster() = default;
    
    // === Управление точками ===
    
    /**
     * @brief Добавить точку в кластер
     * @param point_index индекс точки в PointBuffer
     */
    void add_point(size_t point_index);
    
    /**
     * @brief Удалить точки по позициям в кластере
     * @param positions список позиций в indices_ (0, 1, 2, ...)
     */
    void remove_points(const std::vector<size_t>& positions);
    
    /**
     * @brief Очистить кластер (удалить все точки)
     */
    void clear();
    
    // === Доступ к данным ===
    
    /**
     * @brief Количество точек в кластере
     */
    size_t size() const { return indices_.size(); }
    
    /**
     * @brief Проверить, пуст ли кластер
     */
    bool is_empty() const { return indices_.empty(); }
    
    /**
     * @brief Получить все индексы точек
     */
    const std::vector<size_t>& get_indices() const { return indices_; }
    
    /**
     * @brief Получить индекс точки по позиции в кластере
     */
    size_t get_point_index(size_t position) const { return indices_[position]; }
    
    // === Геометрия ===
    
    /**
     * @brief Минимальная дальность в кластере (в бинах)
     */
    uint16_t get_min_range() const { return min_range_; }
    
    /**
     * @brief Максимальная дальность в кластере (в бинах)
     */
    uint16_t get_max_range() const { return max_range_; }
    
    /**
     * @brief Азимутальный размах кластера (в МАИ)
     * Учитывает переход через Север
     */
    int get_azimuth_span() const { return azimuth_span_; }
    
    /**
     * @brief Проверить, есть ли RBS точки
     */
    bool has_rbs() const { return has_rbs_; }
    
    /**
     * @brief Проверить, есть ли UVD точки
     */
    bool has_uvd() const { return has_uvd_; }
    
    /**
     * @brief Проверить, смешанный ли кластер (есть и RBS, и UVD)
     */
    bool is_mixed() const { return has_rbs_ && has_uvd_; }
    
    // === Управление состоянием ===
    
    /**
     * @brief Закрыть кластер (больше нельзя добавлять точки)
     */
    void close() { closed_ = true; }
    
    /**
     * @brief Проверить, закрыт ли кластер
     */
    bool is_closed() const { return closed_; }
    
    /**
     * @brief Получить последний азимут в кластере
     */
    uint16_t get_last_azimuth() const { return last_azimuth_; }

private:
    /**
     * @brief Пересчитать статистику кластера
     */
    void recalculate_statistics();
    
    std::vector<size_t> indices_;   // Индексы точек в PointBuffer (в порядке добавления)
    
    // Кэшированная статистика
    uint16_t min_range_{65535};
    uint16_t max_range_{0};
    int azimuth_span_{0};
    uint16_t last_azimuth_{0};
    bool has_rbs_{false};
    bool has_uvd_{false};
    bool closed_{false};
};

} // namespace radar
} // namespace vrl
