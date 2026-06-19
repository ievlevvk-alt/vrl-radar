// include/vrl/radar/processing/range_grouper.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include <vector>
#include <map>
#include <set>

// Forward declaration
namespace vrl {
namespace radar {
struct TargetCluster;
}
}

namespace vrl {
namespace radar {

/**
 * @brief Группирует ответы по дальности
 * 
 * Отвечает за объединение ответов в группы по близости дальности
 */
class RangeGrouper {
public:
    /**
     * @brief Группа ответов с близкой дальностью
     */
    struct RangeGroup {
        uint16_t nominal_range{0};
        std::vector<const RBSReply*> rbs_replies;
        std::vector<const UVDReply*> uvd_replies;
        std::set<uint16_t> azimuths;
        
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
        
        bool has_rbs() const { return !rbs_replies.empty(); }
        bool has_uvd() const { return !uvd_replies.empty(); }
        bool has_overlap() const { return rbs_replies.size() > 1; }
    };
    
    /**
     * @brief Конструктор
     * @param range_tolerance_bins допуск по дальности в бинах
     */
    explicit RangeGrouper(uint16_t range_tolerance_bins = 5);
    
    /**
     * @brief Сгруппировать ответы из кластера по дальности
     * @param cluster кластер с ответами
     * @return вектор групп
     */
    std::vector<RangeGroup> group(const TargetCluster& cluster);
    
    /**
     * @brief Установить допуск по дальности
     * @param tolerance_bins допуск в бинах
     */
    void set_tolerance(uint16_t tolerance_bins) { range_tolerance_bins_ = tolerance_bins; }
    
    /**
     * @brief Получить текущий допуск
     */
    uint16_t get_tolerance() const { return range_tolerance_bins_; }
    
private:
    uint16_t range_tolerance_bins_;
};

} // namespace radar
} // namespace vrl
