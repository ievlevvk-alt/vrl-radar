// include/vrl/radar/processing/legacy_clusterer.h
#pragma once

#include "i_clusterer.h"
#include "cluster.h"  // <-- ДОБАВЛЯЕМ
#include <vector>
#include <map>
#include <set>

namespace vrl {
namespace radar {

/**
 * @brief Существующий алгоритм кластеризации (адаптированный под интерфейс)
 * 
 * Реализует текущий алгоритм ClusterTracker как IClusterer.
 * Сохраняет обратную совместимость.
 */
class LegacyClusterer : public IClusterer {
public:
    /**
     * @brief Конструктор
     * @param max_gap_azimuth максимальный разрыв по азимуту для завершения кластера
     * @param range_window окно по дальности для объединения
     */
    LegacyClusterer(int max_gap_azimuth = 8, int range_window = 30);
    
    // === Реализация IClusterer ===
    
    void process_scan(const ScanReplies& scan) override;
    std::vector<TargetCluster> get_completed_clusters() override;
    const std::vector<TargetCluster>& get_active_clusters() const override;
    void reset() override;
    std::vector<TargetCluster> finish_all() override;
    
    std::string get_name() const override { return "LegacyClusterer"; }
    void get_stats(size_t& active, size_t& completed) const override;
    std::unique_ptr<IClusterer> clone() const override;
    
    void set_param(const std::string& key, double value) override;
    void set_param(const std::string& key, int value) override;
    
    // === Дополнительные методы ===
    
    void set_max_gap_azimuth(int gap) { max_gap_azimuth_ = gap; }
    void set_range_window(int window) { range_window_ = window; }
    
private:
    void update_existing_clusters(const ScanReplies& scan);
    void try_create_new_clusters(const ScanReplies& scan);
    void complete_expired_clusters(uint16_t current_azimuth);
    
    std::vector<TargetCluster> active_clusters_;
    std::vector<TargetCluster> completed_clusters_;
    std::vector<TargetCluster> finalized_clusters_;
    
    int max_gap_azimuth_;
    int range_window_;
    
    // Статистика
    size_t total_scans_processed_{0};
    size_t total_clusters_formed_{0};
    size_t total_clusters_completed_{0};
};

} // namespace radar
} // namespace vrl
