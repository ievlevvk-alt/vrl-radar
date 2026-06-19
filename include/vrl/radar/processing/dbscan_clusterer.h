// include/vrl/radar/processing/dbscan_clusterer.h
#pragma once

#include "i_clusterer.h"
#include "../core/replies.h"
#include "../core/config.h"
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <optional>
#include <algorithm>
#include <cmath>
#include <limits>

namespace vrl {
namespace radar {

/**
 * @brief DBSCAN кластеризатор с азимутально-дальностной логикой
 * 
 * Особенности:
 * - Работает в нативных единицах (МАИ и бины дальности)
 * - Учитывает поступление азимутов по возрастанию
 * - Автоматически объединяет перекрывающиеся кластеры
 * - Разделяет RBS и UVD ответы
 * - Корректно обрабатывает переход через Север
 */
class DBSCANClusterer : public IClusterer {
public:
    /**
     * @brief Конструктор
     * @param config конфигурация радара (для beamwidth)
     * @param max_range_gap максимальный разрыв по дальности в бинах
     * @param min_points минимальное количество точек для кластера
     * @param azimuth_gap_coefficient коэффициент для расчета max_azimuth_gap
     */
    explicit DBSCANClusterer(const RadarConfig& config,
                             int max_range_gap = 3,
                             int min_points = 2,
                             double azimuth_gap_coefficient = 1.2);
    
    // Конструктор по умолчанию
    DBSCANClusterer() 
        : DBSCANClusterer(RadarConfig{}, 3, 2, 1.2) {}
    
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
    
    // === Дополнительные методы для настройки ===
    void set_max_azimuth_gap(int gap) { max_azimuth_gap_ = gap; }
    void set_max_range_gap(int gap) { max_range_gap_ = gap; }
    void set_min_points(int pts) { min_points_ = pts; }
    void set_azimuth_gap_coefficient(double coef) { azimuth_gap_coefficient_ = coef; }
    void set_debug(bool enable) { debug_ = enable; }
    
    int get_max_azimuth_gap() const { return max_azimuth_gap_; }
    int get_max_range_gap() const { return max_range_gap_; }
    int get_min_points() const { return min_points_; }
    double get_azimuth_gap_coefficient() const { return azimuth_gap_coefficient_; }

private:
    // ========================================================================
    // ВНУТРЕННИЕ СТРУКТУРЫ
    // ========================================================================
    
    /**
     * @brief Точка (единичный ответ)
     */
    struct Point {
        uint16_t azimuth{0};      // Азимут в МАИ (0-4095)
        uint16_t range{0};        // Дальность в бинах
        uint16_t amplitude{0};    // Амплитуда
        bool is_rbs{true};        // true - RBS, false - UVD
        
        // Ссылки на оригинальные ответы (для сохранения флагов)
        const RBSReply* rbs_reply{nullptr};
        const UVDReply* uvd_reply{nullptr};
        
        // Флаги из ответов
        bool spi{false};
        uint16_t code12{0};
        uint32_t data20{0};
        
        // Конструкторы
        static Point from_rbs(const RBSReply& reply) {
            Point p;
            p.azimuth = reply.azimuth;
            p.range = reply.range;
            p.is_rbs = true;
            p.rbs_reply = &reply;
            p.spi = reply.spi;
            p.code12 = reply.code12;
            // Амплитуда - средняя из фреймовых импульсов
            p.amplitude = (reply.ether_amplitudes[0] + reply.ether_amplitudes[14]) / 2;
            return p;
        }
        
        static Point from_uvd(const UVDReply& reply) {
            Point p;
            p.azimuth = reply.azimuth;
            p.range = reply.range;
            p.is_rbs = false;
            p.uvd_reply = &reply;
            p.data20 = reply.data20;
            // Амплитуда - средняя из всех импульсов
            uint32_t sum = 0;
            for (uint8_t amp : reply.ether_amplitudes) {
                sum += amp;
            }
            p.amplitude = static_cast<uint16_t>(sum / reply.ether_amplitudes.size());
            return p;
        }
        
        bool operator<(const Point& other) const {
            return azimuth < other.azimuth || (azimuth == other.azimuth && range < other.range);
        }
    };
    
    /**
     * @brief Кластер (группа точек)
     */
    struct RadarCluster {
        // Азимутальный диапазон (с учетом перехода через Север)
        // Храним как набор азимутов для корректного определения диапазона
        std::set<uint16_t> azimuths;          // Все азимуты в кластере
        uint16_t last_azimuth{0};             // Последний добавленный азимут
        uint16_t min_azimuth{4096};           // Минимальный азимут (для отладки)
        uint16_t max_azimuth{0};              // Максимальный азимут (для отладки)
        
        // Дальностный диапазон
        uint16_t min_range{65535};
        uint16_t max_range{0};
        
        // Точки
        std::vector<Point> points;
        
        // Состояние
        bool is_closed{false};                // Закрыт для новых точек
        uint32_t last_revolution{0};          // Последний оборот обновления
        uint32_t created_revolution{0};       // Оборот создания
        
        // Тип кластера (определяется по первой точке)
        enum class Type { RBS, UVD, EMPTY } type{Type::EMPTY};
        
        // Флаг перехода через Север
        bool crosses_north() const {
            if (azimuths.size() < 2) return false;
            // Проверяем: есть ли азимуты близкие к 0 и близкие к 4095
            auto it = azimuths.begin();
            uint16_t first = *it;
            auto last_it = azimuths.end();
            --last_it;
            uint16_t last = *last_it;
            // Если минимальный азимут близок к 0, а максимальный близок к 4095
            return (first < 10 && last > 4085);
        }
        
        /**
         * @brief Вычислить расстояние между азимутами
         */
        static int16_t azimuth_distance(uint16_t a, uint16_t b) {
            int16_t diff = static_cast<int16_t>(a - b);
            if (diff < 0) diff += 4096;
            return diff;
        }
        
        /**
         * @brief Проверить, можно ли добавить точку в кластер
         */
        bool can_add_point(const Point& point, 
                          int max_az_gap, 
                          int max_r_gap) const {
            if (is_closed) return false;
            if (points.empty()) return true;  // Первая точка всегда добавляется
            
            // Проверка типа (RBS и UVD не смешиваем)
            if (point.is_rbs && type != Type::RBS) return false;
            if (!point.is_rbs && type != Type::UVD) return false;
            
            // Проверка азимутального разрыва
            int16_t az_gap = azimuth_distance(point.azimuth, last_azimuth);
            if (az_gap > max_az_gap) {
                return false;  // Слишком большой разрыв
            }
            
            // Проверка дальности
            if (point.range < min_range - max_r_gap ||
                point.range > max_range + max_r_gap) {
                return false;
            }
            
            return true;
        }
        
        /**
         * @brief Добавить точку в кластер
         */
        void add_point(const Point& point) {
            points.push_back(point);
            azimuths.insert(point.azimuth);
            
            // Обновляем дальность
            min_range = std::min(min_range, point.range);
            max_range = std::max(max_range, point.range);
            last_azimuth = point.azimuth;
            
            // Обновляем min/max азимут для отладки
            min_azimuth = std::min(min_azimuth, point.azimuth);
            max_azimuth = std::max(max_azimuth, point.azimuth);
            
            // Устанавливаем тип
            if (type == Type::EMPTY) {
                type = point.is_rbs ? Type::RBS : Type::UVD;
            }
        }
        
        /**
         * @brief Проверить пересечение азимутальных множеств
         */
        bool azimuths_overlap(const RadarCluster& other) const {
            // Если одно из множеств пустое
            if (azimuths.empty() || other.azimuths.empty()) return false;
            
            // Проверяем пересечение множеств
            for (uint16_t az : azimuths) {
                if (other.azimuths.find(az) != other.azimuths.end()) {
                    return true;
                }
            }
            
            // Проверяем, что азимуты близки (разрыв меньше max_az_gap)
            for (uint16_t az1 : azimuths) {
                for (uint16_t az2 : other.azimuths) {
                    int16_t gap = azimuth_distance(az1, az2);
                    if (gap < 20) {  // Примерный порог для объединения
                        return true;
                    }
                }
            }
            
            return false;
        }
        
        /**
         * @brief Проверить пересечение с другим кластером
         */
        bool overlaps(const RadarCluster& other, int max_r_gap) const {
            // Разные типы не пересекаются
            if (type != other.type) return false;
            if (type == Type::EMPTY || other.type == Type::EMPTY) return false;
            
            // Проверка дальности
            if (min_range > other.max_range + max_r_gap ||
                other.min_range > max_range + max_r_gap) {
                return false;
            }
            
            // Проверка азимута (с учетом перехода через Север)
            return azimuths_overlap(other);
        }
        
        /**
         * @brief Объединить с другим кластером
         */
        void merge(const RadarCluster& other) {
            if (type == Type::EMPTY) {
                type = other.type;
            }
            
            // Объединяем азимуты
            azimuths.insert(other.azimuths.begin(), other.azimuths.end());
            
            // Обновляем дальность
            min_range = std::min(min_range, other.min_range);
            max_range = std::max(max_range, other.max_range);
            last_azimuth = other.last_azimuth;
            
            // Обновляем min/max азимут для отладки
            min_azimuth = std::min(min_azimuth, other.min_azimuth);
            max_azimuth = std::max(max_azimuth, other.max_azimuth);
            
            // Объединяем точки
            points.insert(points.end(), other.points.begin(), other.points.end());
            
            // Сортируем точки по азимуту, затем по дальности
            std::sort(points.begin(), points.end(),
                [](const Point& a, const Point& b) {
                    if (a.azimuth != b.azimuth) return a.azimuth < b.azimuth;
                    return a.range < b.range;
                });
        }
        
        /**
         * @brief Проверить, достаточно ли точек для кластера
         */
        bool has_min_points(int min_pts) const {
            return points.size() >= static_cast<size_t>(min_pts);
        }
        
        /**
         * @brief Получить азимутальный размах кластера
         */
        int azimuth_span() const {
            if (azimuths.size() < 2) return 0;
            
            // Если кластер пересекает Север, размах = (4096 - min) + max
            if (crosses_north()) {
                auto it = azimuths.begin();
                uint16_t first = *it;
                auto last_it = azimuths.end();
                --last_it;
                uint16_t last = *last_it;
                return (4096 - first) + last;
            }
            
            return max_azimuth - min_azimuth;
        }
        
        /**
         * @brief Преобразовать в TargetCluster
         */
        TargetCluster to_target_cluster(uint32_t revolution) const;
    };
    
    // ========================================================================
    // МЕТОДЫ
    // ========================================================================
    
    /**
     * @brief Обработать ответы на одном азимуте
     */
    void process_azimuth(const std::vector<RBSReply>& rbs_replies,
                         const std::vector<UVDReply>& uvd_replies,
                         uint16_t azimuth,
                         uint32_t revolution);
    
    /**
     * @brief Добавить точку в существующие кластеры или создать новый
     */
    bool try_add_to_clusters(const Point& point, uint32_t revolution);
    
    /**
     * @brief Объединить перекрывающиеся кластеры
     */
    void merge_overlapping_clusters();
    
    /**
     * @brief Закрыть кластеры с большим азимутальным разрывом
     */
    void close_expired_clusters(uint16_t current_azimuth, uint32_t revolution);
    
    /**
     * @brief Завершить закрытые кластеры
     */
    void finalize_closed_clusters();
    
    /**
     * @brief Отладочный вывод кластеров
     */
    void debug_print_clusters(const std::string& prefix = "");
    
    // ========================================================================
    // ДАННЫЕ
    // ========================================================================
    
    // Параметры
    int max_azimuth_gap_{8};          // Максимальный разрыв по азимуту
    int max_range_gap_{3};            // Максимальный разрыв по дальности
    int min_points_{2};               // Минимальное количество точек
    double azimuth_gap_coefficient_{1.2};  // Коэффициент для расчета gap
    bool debug_{false};               // Отладочный вывод
    
    // Данные
    std::vector<RadarCluster> active_clusters_;      // Активные кластеры
    std::vector<TargetCluster> completed_clusters_;  // Завершенные кластеры
    
    // Статистика
    size_t total_scans_processed_{0};
    size_t total_clusters_formed_{0};
    size_t total_clusters_completed_{0};
    size_t total_points_processed_{0};
    
    // Текущее состояние
    uint32_t current_revolution_{0};
    uint16_t current_azimuth_{0};
    
    // Конфигурация
    RadarConfig config_;
    
    // Константы
    static constexpr int AZIMUTH_BINS = 4096;
};

} // namespace radar
} // namespace vrl
