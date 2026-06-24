// include/vrl/radar/core/cluster_pool.hpp
#pragma once

#include "cluster.hpp"
#include <vector>
#include <array>
#include <cstdint>
#include <iostream>

namespace vrl {
namespace radar {

class ClusterPool {
public:
    static constexpr int NUM_SECTORS = 32;
    static constexpr int SECTOR_SIZE = 4096 / NUM_SECTORS;
    
    enum class ClusterState : uint8_t {
        ACTIVE,
        WIDE,
        CLOSED,
        DELAYED,
        PROCESSED
    };
    
    static ClusterPool& instance();
    
    // === Инициализация ===
    void init(size_t max_clusters = 65535);
    bool is_initialized() const { return initialized_; }
    
    // === Управление отладкой ===
    void enable_debug(bool enable) { debug_enabled_ = enable; }
    bool is_debug_enabled() const { return debug_enabled_; }
    
    // === Создание ===
    uint64_t create_cluster();
    
    // === Закрытие ===
    void close_cluster(uint64_t id, int sector);
    
    // === Широкие кластеры ===
    void add_to_wide(uint64_t id);
    std::vector<uint64_t> take_wide_clusters();
    
    // === Получение для обработки ===
    std::vector<uint64_t> take_closed_clusters(int sector);
    
    // === Очистка задержанных ===
    size_t cleanup_delayed_clusters(uint32_t current_revolution, double max_delay = 0.5);
    
    // === Получение ===
    Cluster* get_cluster(uint64_t id);
    const Cluster* get_cluster(uint64_t id) const;
    std::vector<Cluster*> get_all_clusters() const;
    std::vector<Cluster*> get_active_clusters() const;      // <-- НОВЫЙ МЕТОД
    std::vector<uint64_t> get_all_ids() const;
    
    // === Проверка ===
    bool has_closed_clusters(int sector) const;
    bool has_wide_clusters() const;
    bool has_delayed_clusters() const;
    bool exists(uint64_t id) const;
    
    // === Подсчёт ===
    size_t count_active_clusters() const;
    size_t count_closed_clusters() const;
    size_t count_delayed_clusters() const;
    size_t count_wide_clusters() const;
    size_t size() const;
    bool is_empty() const;
    
    // === Управление ===
    void clear();
    
    // === Статистика ===
    struct Stats {
        size_t total_clusters{0};
        size_t active_clusters{0};
        size_t closed_clusters{0};
        size_t wide_clusters{0};
        size_t delayed_clusters{0};
        size_t clusters_by_sector[NUM_SECTORS]{};
    };
    
    Stats get_stats() const;

    void add_to_active(uint64_t id);

    void merge_clusters(uint64_t keep_id, uint64_t remove_id);    

    // === Управление задержанными ===
    void add_to_delayed(uint64_t id, int sector); 

    // === Получение ===
    std::vector<uint64_t> get_closed_clusters() const;  // <-- ДОБАВИТЬ    

    // === Управление задержанными ===
    std::vector<uint64_t> take_delayed_clusters(int sector);    


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
    
    // === ХРАНИЛИЩЕ ===
    std::vector<Cluster> clusters_;
    std::vector<Metadata> metadata_;
    
    // Списки ID
    std::vector<uint64_t> active_ids_;
    std::array<std::vector<uint64_t>, NUM_SECTORS> closed_by_sector_;
    std::vector<uint64_t> wide_ids_;
    std::vector<uint64_t> delayed_ids_;
    
    size_t max_clusters_{65535};
    size_t next_slot_{0};
    bool initialized_{false};
    bool debug_enabled_{true};
};

} // namespace radar
} // namespace vrl
