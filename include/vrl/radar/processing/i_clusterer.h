// include/vrl/radar/processing/i_clusterer.h
#pragma once

#include "../core/replies.h"
#include "../core/types.h"
#include <vector>
#include <memory>
#include <string>

namespace vrl {
namespace radar {

// Forward declaration
struct TargetCluster;

/**
 * @brief Результат кластеризации
 */
struct ClusterResult {
    std::vector<TargetCluster> clusters;
    double processing_time_ms{0.0};
    int total_replies_processed{0};
    int clusters_formed{0};
    std::string method_used;
};

/**
 * @brief Интерфейс для алгоритмов кластеризации
 * 
 * Определяет контракт для всех реализаций кластеризации.
 * Позволяет легко подменять алгоритм (DBSCAN, OPTICS, иерархический и т.д.)
 */
class IClusterer {
public:
    virtual ~IClusterer() = default;
    
    /**
     * @brief Обработать сканирование
     * @param scan данные сканирования
     */
    virtual void process_scan(const ScanReplies& scan) = 0;
    
    /**
     * @brief Получить завершенные кластеры
     * @return вектор завершенных кластеров
     */
    virtual std::vector<TargetCluster> get_completed_clusters() = 0;
    
    /**
     * @brief Получить активные кластеры (для отладки)
     * @return const ссылка на активные кластеры
     */
    virtual const std::vector<TargetCluster>& get_active_clusters() const = 0;
    
    /**
     * @brief Сбросить все кластеры
     */
    virtual void reset() = 0;
    
    /**
     * @brief Завершить все активные кластеры
     * @return вектор всех завершенных кластеров
     */
    virtual std::vector<TargetCluster> finish_all() = 0;
    
    /**
     * @brief Получить имя алгоритма для логирования
     */
    virtual std::string get_name() const = 0;
    
    /**
     * @brief Получить статистику работы
     */
    virtual void get_stats(size_t& active, size_t& completed) const = 0;
    
    /**
     * @brief Создать клон кластеризатора
     */
    virtual std::unique_ptr<IClusterer> clone() const = 0;
    
    /**
     * @brief Установить параметры кластеризации
     */
    virtual void set_param(const std::string& key, double value) = 0;
    virtual void set_param(const std::string& key, int value) = 0;
};

} // namespace radar
} // namespace vrl
