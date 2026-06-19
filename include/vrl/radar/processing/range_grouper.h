// include/vrl/radar/processing/range_grouper.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include <vector>
#include <map>
#include <set>
#include <cstdint>
#include <algorithm>
#include <memory>

namespace vrl {
namespace radar {

// Forward declaration
struct TargetCluster;

/**
 * @brief Группирует ответы по дальности с использованием сортировки O(n log n)
 * 
 * Отвечает за объединение ответов в группы по близости дальности.
 * Использует сортировку O(n log n) вместо O(n²) линейного поиска.
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
        bool has_overlap() const { return rbs_replies.size() > 1 || uvd_replies.size() > 1; }
        
        void reserve(size_t rbs_count, size_t uvd_count) {
            rbs_replies.reserve(rbs_count);
            uvd_replies.reserve(uvd_count);
        }
    };
    
    /**
     * @brief Конструктор
     * @param range_tolerance_bins допуск по дальности в бинах
     */
    explicit RangeGrouper(uint16_t range_tolerance_bins = 5);
    
    /**
     * @brief Сгруппировать ответы из кластера по дальности (оптимизированная версия)
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
    /**
     * @brief Вспомогательная структура для сортировки ответов
     */
    struct ReplyEntry {
        uint16_t range;
        uint16_t azimuth;
        union {
            const RBSReply* rbs;
            const UVDReply* uvd;
        };
        enum class Type : uint8_t { RBS, UVD } type;
        
        static ReplyEntry make_rbs(const RBSReply* reply) {
            ReplyEntry entry;
            entry.range = reply->range;
            entry.azimuth = reply->azimuth;
            entry.rbs = reply;
            entry.type = Type::RBS;
            return entry;
        }
        
        static ReplyEntry make_uvd(const UVDReply* reply) {
            ReplyEntry entry;
            entry.range = reply->range;
            entry.azimuth = reply->azimuth;
            entry.uvd = reply;
            entry.type = Type::UVD;
            return entry;
        }
    };
    
    /**
     * @brief Группировка отсортированных записей
     */
    void group_sorted_entries(
        const std::vector<ReplyEntry>& entries,
        std::vector<RangeGroup>& groups,
        bool is_rbs
    );
    
    uint16_t range_tolerance_bins_;
};

} // namespace radar
} // namespace vrl
