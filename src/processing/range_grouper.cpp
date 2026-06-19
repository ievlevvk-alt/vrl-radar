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
    // ШАГ 1: Собираем RBS ответы в отдельный список
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
    
    std::vector<ReplyEntry> rbs_replies;
    std::vector<ReplyEntry> uvd_replies;
    
    // Приблизительная оценка для резервирования памяти
    size_t estimated_rbs = 0;
    size_t estimated_uvd = 0;
    for (const auto& scan : cluster.scans) {
        estimated_rbs += scan.rbs_replies.size();
        estimated_uvd += scan.uvd_replies.size();
    }
    rbs_replies.reserve(estimated_rbs);
    uvd_replies.reserve(estimated_uvd);
    
    // Собираем RBS ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            rbs_replies.push_back(ReplyEntry::make_rbs(&reply));
        }
    }
    
    // Собираем UVD ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            uvd_replies.push_back(ReplyEntry::make_uvd(&reply));
        }
    }
    
    // ============================================================================
    // ШАГ 2: Группируем RBS ответы отдельно
    // ============================================================================
    
    std::vector<RangeGroup> all_groups;
    
    if (!rbs_replies.empty()) {
        // Сортируем RBS по дальности
        std::sort(rbs_replies.begin(), rbs_replies.end(),
            [](const ReplyEntry& a, const ReplyEntry& b) {
                return a.range < b.range;
            });
        
        // Группируем RBS
        size_t i = 0;
        while (i < rbs_replies.size()) {
            RangeGroup group;
            group.nominal_range = rbs_replies[i].range;
            group.add_rbs(rbs_replies[i].rbs);
            
            size_t j = i + 1;
            while (j < rbs_replies.size()) {
                int16_t range_diff = std::abs(static_cast<int16_t>(
                    rbs_replies[j].range - group.nominal_range));
                
                if (range_diff <= range_tolerance_bins_) {
                    group.add_rbs(rbs_replies[j].rbs);
                    j++;
                } else {
                    break;
                }
            }
            
            all_groups.push_back(std::move(group));
            i = j;
        }
        
        VRL_LOG_TRACE(modules::CLUSTER, "Created " + 
                      std::to_string(all_groups.size()) + " RBS groups from " + 
                      std::to_string(rbs_replies.size()) + " replies");
    }
    
    // ============================================================================
    // ШАГ 3: Группируем UVD ответы отдельно
    // ============================================================================
    
    if (!uvd_replies.empty()) {
        // Сортируем UVD по дальности
        std::sort(uvd_replies.begin(), uvd_replies.end(),
            [](const ReplyEntry& a, const ReplyEntry& b) {
                return a.range < b.range;
            });
        
        // Группируем UVD
        size_t i = 0;
        size_t uvd_groups_start = all_groups.size();
        
        while (i < uvd_replies.size()) {
            RangeGroup group;
            group.nominal_range = uvd_replies[i].range;
            group.add_uvd(uvd_replies[i].uvd);
            
            size_t j = i + 1;
            while (j < uvd_replies.size()) {
                int16_t range_diff = std::abs(static_cast<int16_t>(
                    uvd_replies[j].range - group.nominal_range));
                
                if (range_diff <= range_tolerance_bins_) {
                    group.add_uvd(uvd_replies[j].uvd);
                    j++;
                } else {
                    break;
                }
            }
            
            all_groups.push_back(std::move(group));
            i = j;
        }
        
        VRL_LOG_TRACE(modules::CLUSTER, "Created " + 
                      std::to_string(all_groups.size() - uvd_groups_start) + 
                      " UVD groups from " + std::to_string(uvd_replies.size()) + " replies");
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Total: " + std::to_string(all_groups.size()) + 
                  " groups from " + std::to_string(rbs_replies.size() + uvd_replies.size()) + 
                  " replies");
    
    return all_groups;
}

} // namespace radar
} // namespace vrl
