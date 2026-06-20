// include/vrl/radar/core/cluster_pool.hpp
#pragma once

#include "cluster.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace vrl {
namespace radar {

/**
 * @brief Пул кластеров — синглтон для хранения всех кластеров
 * 
 * Хранит все активные и закрытые кластеры.
 * Обеспечивает доступ к кластерам по индексу.
 */
class ClusterPool {
public:
    static ClusterPool& instance();
    
    /**
     * @brief Создать новый кластер и добавить в пул
     * @return ссылка на созданный кластер
     */
    Cluster& create_cluster();
    
    /**
     * @brief Получить кластер по индексу
     * @param index индекс кластера в пуле
     * @return ссылка на кластер
     */
    Cluster& get_cluster(size_t index);
    
    /**
     * @brief Получить константную ссылку на кластер
     */
    const Cluster& get_cluster(size_t index) const;
    
    /**
     * @brief Получить все кластеры
     */
    const std::vector<std::unique_ptr<Cluster>>& get_all_clusters() const;
    
    /**
     * @brief Получить все активные (незакрытые) кластеры
     */
    std::vector<Cluster*> get_active_clusters();
    
    /**
     * @brief Получить все закрытые кластеры
     */
    std::vector<Cluster*> get_closed_clusters();
    
    /**
     * @brief Удалить кластер из пула
     * @param index индекс кластера
     */
    void remove_cluster(size_t index);
    
    /**
     * @brief Очистить пул (удалить все кластеры)
     */
    void clear();
    
    /**
     * @brief Количество кластеров в пуле
     */
    size_t size() const { return clusters_.size(); }
    
    /**
     * @brief Проверить, пуст ли пул
     */
    bool is_empty() const { return clusters_.empty(); }

private:
    ClusterPool() = default;
    ~ClusterPool() = default;
    ClusterPool(const ClusterPool&) = delete;
    ClusterPool& operator=(const ClusterPool&) = delete;
    
    std::vector<std::unique_ptr<Cluster>> clusters_;
    mutable std::mutex mutex_;
};

} // namespace radar
} // namespace vrl
