// include/vrl/radar/v2/plot_pool.hpp
#pragma once

#include "plot.hpp"
#include <vector>
#include <cstdint>
#include <cstddef>

namespace vrl {
namespace radar {
namespace v2 {

/**
 * @brief Пул плотов (кольцевой буфер)
 * 
 * Аналог PointBuffer, ClusterPool, TrackPool.
 * Размер = размеру TrackPool.
 */
class PlotPool {
public:
    static PlotPool& instance();
    
    void init(size_t max_plots = 8192);
    bool is_initialized() const { return initialized_; }
    
    /**
     * @brief Добавить плот в буфер
     * @param plot плот для добавления
     * @return индекс плота (0 = ошибка)
     */
    uint64_t add_plot(const Plot& plot);
    
    /**
     * @brief Получить плот по индексу
     * @param index индекс плота (1-based)
     * @return указатель на плот, или nullptr
     */
    const Plot* get_plot(uint64_t index) const;
    
    /**
     * @brief Получить размер пула
     */
    size_t size() const { return plots_.size(); }
    
    /**
     * @brief Очистить пул
     */
    void clear();

private:
    PlotPool() = default;
    ~PlotPool() = default;
    PlotPool(const PlotPool&) = delete;
    PlotPool& operator=(const PlotPool&) = delete;
    
    bool is_valid_index(uint64_t index) const;
    
    std::vector<Plot> plots_;
    size_t max_plots_{8192};
    size_t head_{0};
    bool initialized_{false};
};

} // namespace v2
} // namespace radar
} // namespace vrl
