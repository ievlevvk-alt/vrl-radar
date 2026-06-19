// src/processing/range_grouper.cpp
#include "vrl/radar/processing/range_grouper.h"
#include "vrl/radar/processing/cluster.h"  // <-- ДОБАВЛЯЕМ ДЛЯ TargetCluster
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>

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
    
    std::vector<RangeGroup> groups;
    std::map<uint16_t, RangeGroup> range_map;
    
    // Группируем RBS ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            bool added = false;
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_bins_) {
                    group.add_rbs(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_rbs(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    // Группируем UVD ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            bool added = false;
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_bins_) {
                    group.add_uvd(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_uvd(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    // Переносим группы в вектор
    for (auto& [_, group] : range_map) {
        groups.push_back(std::move(group));
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Created " + std::to_string(groups.size()) + " range groups");
    return groups;
}

} // namespace radar
} // namespace vrl
