// include/vrl/radar/processing/dbscan_clusterer.h
#pragma once

#include "i_clusterer.h"
#include "cluster.h"
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <unordered_set>
#include <limits>

namespace vrl {
namespace radar {

/**
 * @brief DBSCAN кластеризатор для радиолокационных данных
 * 
 * Реализует алгоритм DBSCAN для группировки ответов в кластеры.
 * Использует эллиптическую метрику с учетом дальности.
 * 
 * Параметры:
 * - eps_range: максимальное расстояние по дальности в метрах
 * - eps_azimuth_deg: максимальное угловое расстояние в градусах
 * - min_pts: минимальное количество точек для формирования кластера
 */
class DBSCANClusterer : public IClusterer {
public:
    /**
     * @brief Конструктор
     * @param eps_range погрешность по дальности в метрах (обычно 50-150м)
     * @param eps_azimuth_deg погрешность по азимуту в градусах (обычно 0.5-2°)
     * @param min_pts минимальное количество точек для кластера
     * @param range_bin_m размер бина дальности в метрах
     */
    DBSCANClusterer(double eps_range = 150.0, double eps_azimuth_deg = 1.0, 
                    int min_pts = 3, double range_bin_m = 30.0);
    
    ~DBSCANClusterer() override = default;
    
    // === Реализация IClusterer ===
    
    void process_scan(const ScanReplies& scan) override;
    std::vector<TargetCluster> get_completed_clusters() override;
    const std::vector<TargetCluster>& get_active_clusters() const override;
    void reset() override;
    std::vector<TargetCluster> finish_all() override;
    
    std::string get_name() const override { return "DBSCANClusterer"; }
    void get_stats(size_t& active, size_t& completed) const override;
    std::unique_ptr<IClusterer> clone() const override;
    
    void set_param(const std::string& key, double value) override;
    void set_param(const std::string& key, int value) override;
    
    // === Дополнительные методы ===
    
    void set_eps_range(double eps) { eps_range_ = eps; }
    void set_eps_azimuth_deg(double eps_deg) { eps_azimuth_deg_ = eps_deg; }
    void set_min_pts(int min_pts) { min_pts_ = min_pts; }
    void set_range_bin(double range_bin_m) { range_bin_m_ = range_bin_m; }
    
    double get_eps_range() const { return eps_range_; }
    double get_eps_azimuth_deg() const { return eps_azimuth_deg_; }
    int get_min_pts() const { return min_pts_; }
    double get_range_bin() const { return range_bin_m_; }

private:
    /**
     * @brief Структура точки для DBSCAN
     */
    struct Point {
        double x;           // Координата X в метрах
        double y;           // Координата Y в метрах
        double range_m;     // Дальность в метрах
        uint16_t azimuth;   // Азимут в бинах
        uint16_t range;     // Дальность в бинах
        int cluster_id;     // ID кластера (-1 = шум, 0 = неклассифицирована)
        bool visited;       // Флаг посещения
        size_t scan_index;  // Индекс скана
        
        const RBSReply* rbs_reply;
        const UVDReply* uvd_reply;
        bool is_rbs;
        
        Point() : x(0), y(0), range_m(0), azimuth(0), range(0), 
                  cluster_id(-1), visited(false), scan_index(0),
                  rbs_reply(nullptr), uvd_reply(nullptr), is_rbs(true) {}
    };
    
    /**
     * @brief Вычислить расстояние между двумя точками
     * @return расстояние в метрах или INF если точки не соседи
     */
    double distance(const Point& a, const Point& b) const;
    
    /**
     * @brief Найти соседей точки
     */
    std::vector<size_t> find_neighbors(size_t point_idx);
    
    /**
     * @brief Выполнить DBSCAN кластеризацию
     */
    void run_dbscan();
    
    /**
     * @brief Создать кластер из набора точек
     */
    TargetCluster create_cluster_from_points(const std::vector<size_t>& point_indices);
    
    /**
     * @brief Обновить активные кластеры
     */
    void update_active_clusters();
    
    /**
     * @brief Завершить кластеры, которые уже не активны
     */
    void complete_expired_clusters();
    
    // Параметры DBSCAN
    double eps_range_;          // Погрешность по дальности в метрах
    double eps_azimuth_deg_;    // Погрешность по азимуту в градусах
    double eps_azimuth_rad_;    // Погрешность по азимуту в радианах
    int min_pts_;               // Минимальное количество точек для кластера
    double range_bin_m_;        // Размер бина дальности в метрах
    
    // Данные
    std::vector<Point> points_;
    std::vector<TargetCluster> active_clusters_;
    std::vector<TargetCluster> completed_clusters_;
    
    // Статистика
    size_t total_scans_processed_{0};
    size_t total_clusters_formed_{0};
    size_t total_clusters_completed_{0};
    size_t total_noise_points_{0};
    
    // Текущий азимут
    uint16_t current_azimuth_{0};
    
    // Параметры завершения кластера
    int max_gap_azimuth_{8};
    int range_window_{30};
    
    // Константы
    static constexpr double DEG_TO_RAD = M_PI / 180.0;
    static constexpr int AZIMUTH_BINS = 4096;
};

} // namespace radar
} // namespace vrl
