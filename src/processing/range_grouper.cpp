// src/processing/range_grouper.cpp
#include "vrl/radar/processing/range_grouper.h"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>
#include <vector>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

RangeGrouper::RangeGrouper(uint16_t range_tolerance_bins)
    : range_tolerance_bins_(range_tolerance_bins) {
    VRL_LOG_DEBUG(modules::CLUSTER, "RangeGrouper initialized: tolerance=" + 
                  std::to_string(range_tolerance_bins) + " bins");
}

std::vector<RangeGrouper::RangeGroup> RangeGrouper::group(const TargetCluster& cluster) {
    VRL_LOG_TRACE(modules::CLUSTER, "Grouping " + std::to_string(cluster.scans.size()) + " scans by range");
    
    if (cluster.scans.empty()) {
        return {};
    }
    
    // ============================================================================
    // ШАГ 1: Собираем все ответы в единый список с информацией о типе
    // ============================================================================
    
    struct ReplyEntry {
        uint16_t range;
        uint16_t azimuth;
        union {
            const RBSReply* rbs;
            const UVDReply* uvd;
        };
        enum class Type { RBS, UVD } type;
        bool is_rbs;
        
        static ReplyEntry make_rbs(const RBSReply* reply) {
            ReplyEntry entry;
            entry.range = reply->range;
            entry.azimuth = reply->azimuth;
            entry.rbs = reply;
            entry.type = Type::RBS;
            entry.is_rbs = true;
            return entry;
        }
        
        static ReplyEntry make_uvd(const UVDReply* reply) {
            ReplyEntry entry;
            entry.range = reply->range;
            entry.azimuth = reply->azimuth;
            entry.uvd = reply;
            entry.type = Type::UVD;
            entry.is_rbs = false;
            return entry;
        }
    };
    
    std::vector<ReplyEntry> all_replies;
    all_replies.reserve(cluster.scans.size() * 10);  // Приблизительная оценка
    
    // Собираем RBS ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            all_replies.push_back(ReplyEntry::make_rbs(&reply));
        }
    }
    
    // Собираем UVD ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            all_replies.push_back(ReplyEntry::make_uvd(&reply));
        }
    }
    
    if (all_replies.empty()) {
        return {};
    }
    
    // ============================================================================
    // ШАГ 2: Сортируем по дальности (O(n log n))
    // ============================================================================
    
    std::sort(all_replies.begin(), all_replies.end(),
        [](const ReplyEntry& a, const ReplyEntry& b) {
            return a.range < b.range;
        });
    
    // ============================================================================
    // ШАГ 3: Группируем с использованием двух указателей (O(n))
    // ============================================================================
    
    std::vector<RangeGroup> groups;
    size_t i = 0;
    
    while (i < all_replies.size()) {
        // Начинаем новую группу с текущего элемента
        RangeGroup group;
        group.nominal_range = all_replies[i].range;
        
        // Добавляем первый элемент
        if (all_replies[i].is_rbs) {
            group.add_rbs(all_replies[i].rbs);
        } else {
            group.add_uvd(all_replies[i].uvd);
        }
        
        // Ищем все элементы, попадающие в допуск
        size_t j = i + 1;
        while (j < all_replies.size()) {
            int16_t range_diff = std::abs(static_cast<int16_t>(all_replies[j].range - group.nominal_range));
            
            if (range_diff <= range_tolerance_bins_) {
                // Добавляем в текущую группу
                if (all_replies[j].is_rbs) {
                    group.add_rbs(all_replies[j].rbs);
                } else {
                    group.add_uvd(all_replies[j].uvd);
                }
                j++;
            } else {
                // Слишком далеко по дальности - выходим
                break;
            }
        }
        
        groups.push_back(std::move(group));
        i = j;  // Переходим к следующей группе
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created " + std::to_string(groups.size()) + 
                  " range groups from " + std::to_string(all_replies.size()) + " replies");
    
    return groups;
}

} // namespace radar
} // namespace vrl
