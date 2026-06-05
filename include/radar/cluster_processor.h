// file: include/radar/cluster_processor.h
#pragma once

#include "cluster_tracker.h"
#include "replies.h"
#include "garbling_solver.h"
#include <vector>
#include <map>
#include <set>
#include <optional>
#include <cmath>
#include <memory>  // Добавлено для std::unique_ptr

namespace radar {

// Класс для обработки кластера и выделения целей
class ClusterProcessor {
public:
    ClusterProcessor(const RadarConfig& config);
    
    // Основной метод обработки кластера
    std::vector<TargetReport> process_cluster(const TargetCluster& cluster);
    
    // Настройки
    void set_range_tolerance(uint16_t bins) { range_tolerance_ = bins; }
    void set_min_hits(int hits) { min_hits_ = hits; }
    void set_confidence_threshold(uint8_t thresh) { confidence_threshold_ = thresh; }
    
    // Включить/выключить разделение перекрытий
    void enable_garbling_splitting(bool enable) { split_garbled_ = enable; }
    
    // Установить решатель для перекрытий
    void set_garbling_solver(std::unique_ptr<GarblingSolver> solver) {
        garbling_solver_ = std::move(solver);
    }
    
    // Включить отладку
    void set_debug(bool enable) { debug_ = enable; }
    
private:
    // Структура для группировки ответов по дальности
    struct RangeGroup {
        uint16_t nominal_range;           // номинальная дальность группы
        std::vector<const RBSReply*> rbs_replies;
        std::vector<const UVDReply*> uvd_replies;
        std::set<uint16_t> azimuths;       // на каких азимутах видели
        
        void add_rbs(const RBSReply* reply) {
            rbs_replies.push_back(reply);
            azimuths.insert(reply->azimuth);
        }
        
        void add_uvd(const UVDReply* reply) {
            uvd_replies.push_back(reply);
            azimuths.insert(reply->azimuth);
        }
        
        size_t total_replies() const {
            return rbs_replies.size() + uvd_replies.size();
        }
        
        uint16_t azimuth_span() const {
            if (azimuths.empty()) return 0;
            uint16_t min_az = *azimuths.begin();
            uint16_t max_az = *azimuths.rbegin();
            int16_t span = max_az - min_az;
            if (span < 0) span += 4096;
            return static_cast<uint16_t>(span);
        }
    };
    
    // Группировка ответов по дальности
    std::vector<RangeGroup> group_by_range(const TargetCluster& cluster);
    
    // Обработка RBS ответов в группе
    std::optional<TargetReport> process_rbs_group(const RangeGroup& group);
    
    // Обработка УВД ответов в группе
    std::optional<TargetReport> process_uvd_group(const RangeGroup& group);
    
    // Обработка перекрытой группы
    std::vector<TargetReport> process_garbled_group(const RangeGroup& group);
    
    // Вычисление усреднённого азимута (с учётом цикличности)
    double average_azimuth(const std::vector<uint16_t>& azimuths);
    
    // Декодирование УВД информации из 20 бит
    void decode_uvd_info(uint32_t data20, TargetReport& report);

    // Проверка SLS для определения боковых лепестков
    bool check_sidelobe(const RBSReply& reply) const;
    bool check_sidelobe(const UVDReply& reply) const;
    
    RadarConfig config_;
    uint16_t range_tolerance_{5};        // допуск по дальности для группировки (дискреты)
    int min_hits_{2};                      // минимальное число попаданий для цели
    uint8_t confidence_threshold_{50};     // порог достоверности
    
    bool split_garbled_{true};
    bool debug_{false};
    std::unique_ptr<GarblingSolver> garbling_solver_;
};

} // namespace radar