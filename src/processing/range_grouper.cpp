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

void RangeGrouper::group_sorted_entries(
    const std::vector<ReplyEntry>& entries,
    std::vector<RangeGroup>& groups,
    bool is_rbs) {
    
    if (entries.empty()) return;
    
    // Проходим по отсортированным записям
    size_t i = 0;
    while (i < entries.size()) {
        RangeGroup group;
        group.nominal_range = entries[i].range;
        
        // Предварительное резервирование (приблизительно)
        size_t remaining = entries.size() - i;
        size_t estimated = std::min(remaining, size_t(10));
        if (is_rbs) {
            group.reserve(estimated, 0);
        } else {
            group.reserve(0, estimated);
        }
        
        // Добавляем первый элемент
        if (is_rbs) {
            group.add_rbs(entries[i].rbs);
        } else {
            group.add_uvd(entries[i].uvd);
        }
        
        // Добавляем все элементы, попадающие в допуск
        size_t j = i + 1;
        while (j < entries.size()) {
            int16_t range_diff = static_cast<int16_t>(entries[j].range) - 
                                static_cast<int16_t>(group.nominal_range);
            if (range_diff < 0) range_diff = -range_diff;
            
            if (range_diff <= range_tolerance_bins_) {
                if (is_rbs) {
                    group.add_rbs(entries[j].rbs);
                } else {
                    group.add_uvd(entries[j].uvd);
                }
                j++;
            } else {
                break;
            }
        }
        
        groups.push_back(std::move(group));
        i = j;
    }
}

std::vector<RangeGrouper::RangeGroup> RangeGrouper::group(const TargetCluster& cluster) {
    VRL_LOG_TRACE(modules::CLUSTER, "Grouping " + std::to_string(cluster.scans.size()) + " scans by range");
    
    if (cluster.scans.empty()) {
        return {};
    }
    
    // ============================================================================
    // ШАГ 1: Подсчет количества ответов для резервирования
    // ============================================================================
    
    size_t total_rbs = 0;
    size_t total_uvd = 0;
    
    for (const auto& scan : cluster.scans) {
        total_rbs += scan.rbs_replies.size();
        total_uvd += scan.uvd_replies.size();
    }
    
    if (total_rbs == 0 && total_uvd == 0) {
        VRL_LOG_TRACE(modules::CLUSTER, "No replies to group");
        return {};
    }
    
    // ============================================================================
    // ШАГ 2: Сбор ответов в единый вектор с типом
    // ============================================================================
    
    std::vector<ReplyEntry> entries;
    entries.reserve(total_rbs + total_uvd);
    
    // Добавляем RBS ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            entries.push_back(ReplyEntry::make_rbs(&reply));
        }
    }
    
    // Добавляем UVD ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            entries.push_back(ReplyEntry::make_uvd(&reply));
        }
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Collected " + std::to_string(entries.size()) + 
                  " replies (" + std::to_string(total_rbs) + " RBS, " + 
                  std::to_string(total_uvd) + " UVD)");
    
    // ============================================================================
    // ШАГ 3: Сортировка по дальности O(n log n)
    // ============================================================================
    
    std::sort(entries.begin(), entries.end(),
        [](const ReplyEntry& a, const ReplyEntry& b) {
            return a.range < b.range;
        });
    
    // ============================================================================
    // ШАГ 4: Группировка
    // ============================================================================
    
    std::vector<RangeGroup> result;
    
    // Разделяем RBS и UVD для более эффективной группировки
    std::vector<ReplyEntry> rbs_entries;
    std::vector<ReplyEntry> uvd_entries;
    
    rbs_entries.reserve(total_rbs);
    uvd_entries.reserve(total_uvd);
    
    for (const auto& entry : entries) {
        if (entry.type == ReplyEntry::Type::RBS) {
            rbs_entries.push_back(entry);
        } else {
            uvd_entries.push_back(entry);
        }
    }
    
    // Группируем RBS отдельно
    group_sorted_entries(rbs_entries, result, true);
    
    // Группируем UVD отдельно
    group_sorted_entries(uvd_entries, result, false);
    
    // ============================================================================
    // ШАГ 5: Сортировка групп по дальности для консистентности
    // ============================================================================
    
    std::sort(result.begin(), result.end(),
        [](const RangeGroup& a, const RangeGroup& b) {
            return a.nominal_range < b.nominal_range;
        });
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created " + std::to_string(result.size()) + 
                  " groups from " + std::to_string(entries.size()) + " replies");
    
    return result;
}

} // namespace radar
} // namespace vrl
