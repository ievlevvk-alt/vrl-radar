// include/vrl/radar/core/cluster_pool.hpp
#pragma once

#include "cluster.hpp"
#include <vector>
#include <array>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <chrono>
#include <cstdint>

namespace vrl {
namespace radar {

/**
 * @brief Пул кластеров с уникальными ID
 * 
 * Каждый кластер получает уникальный ID при создании, который сохраняется
 * на всём протяжении жизни кластера. Это позволяет безопасно ссылаться
 * на кластер без проблем с индексами.
 */
class ClusterPool {
public:
    static constexpr int NUM_SECTORS = 32;
    static constexpr int SECTOR_SIZE = 4096 / NUM_SECTORS;
    
    enum class ClusterState : uint8_t {
        ACTIVE,      // Можно добавлять точки
        WIDE,        // Широкий (но всё ещё активный!)
        CLOSED,      // Закрыт, ждёт обработки по секторам
        DELAYED,     // Задержан (обработан, ждёт удаления)
        PROCESSED    // Обработан, можно удалить
    };
    
    static ClusterPool& instance();
    
    // --- Создание ---
    uint64_t create_cluster();      // Возвращает уникальный ID
    
    // --- Закрытие ---
    void close_cluster(uint64_t id, int sector);  // ACTIVE → CLOSED
    void mark_as_wide(uint64_t id);               // ACTIVE → WIDE (остаётся в active!)
    
    // --- Получение для обработки ---
    std::vector<uint64_t> take_closed_clusters(int sector);   // CLOSED → DELAYED
    std::vector<uint64_t> take_wide_clusters();               // WIDE → PROCESSED
    
    // --- Удаление ---
    void remove_cluster(uint64_t id);
    
    // --- Очистка задержанных ---
    size_t cleanup_delayed_clusters(uint32_t current_revolution, double max_delay = 0.5);
    
    // --- Получение ---
    Cluster* get_cluster(uint64_t id);
    const Cluster* get_cluster(uint64_t id) const;
    std::vector<Cluster*> get_all_clusters() const;
    std::vector<uint64_t> get_all_ids() const;
    
    // --- Проверка ---
    bool has_closed_clusters(int sector) const;
    bool has_wide_clusters() const;
    bool has_delayed_clusters() const;
    bool exists(uint64_t id) const;
    
    // --- Подсчёт ---
    size_t count_active_clusters() const;
    size_t count_closed_clusters() const;
    size_t count_delayed_clusters() const;
    size_t count_wide_clusters() const;
    size_t size() const;
    bool is_empty() const;
    
    // --- Управление ---
    void clear();
    
    // --- Статистика ---
    struct Stats {
        size_t total_clusters{0};
        size_t active_clusters{0};
        size_t closed_clusters{0};
        size_t wide_clusters{0};
        size_t delayed_clusters{0};
        size_t clusters_by_sector[NUM_SECTORS]{};
    };
    
    Stats get_stats() const;

private:
    ClusterPool() = default;
    ~ClusterPool() = default;
    ClusterPool(const ClusterPool&) = delete;
    ClusterPool& operator=(const ClusterPool&) = delete;
    
    struct Metadata {
        ClusterState state{ClusterState::ACTIVE};
        int sector_index{-1};
        uint16_t close_azimuth{0};
        uint32_t close_revolution{0};
        uint32_t delayed_since{0};
    };
    
    void remove_from_active(uint64_t id);
    void remove_from_closed(uint64_t id);
    void remove_from_wide(uint64_t id);
    void remove_from_delayed(uint64_t id);
    bool is_valid_id(uint64_t id) const;
    
    // Хранилище кластеров с уникальными ID
    std::unordered_map<uint64_t, std::unique_ptr<Cluster>> clusters_;
    
    // Списки ID
    std::vector<uint64_t> active_ids_;
    std::array<std::vector<uint64_t>, NUM_SECTORS> closed_by_sector_;
    std::vector<uint64_t> wide_ids_;
    std::vector<uint64_t> delayed_ids_;
    
    // Метаданные по ID
    std::unordered_map<uint64_t, Metadata> metadata_;
    
    mutable std::mutex mutex_;
    uint64_t next_id_{1};
};

} // namespace radar
} // namespace vrl
