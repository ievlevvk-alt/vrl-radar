// src/processing/rbs_processor.cpp
#include "vrl/radar/processing/rbs_processor.h"
#include "vrl/radar/utils/utils.h"
#include "vrl/radar/utils/logger.h"
#include <map>
#include <cmath>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

RBSProcessor::RBSProcessor(const RadarConfig& config)
    : config_(config), reply_processor_(config) {
    VRL_LOG_DEBUG(modules::CLUSTER, "RBSProcessor initialized");
}

double RBSProcessor::average_azimuth(const std::vector<uint16_t>& azimuths) {
    if (azimuths.empty()) return 0.0;
    
    const int AZIMUTH_HALF = 2048;
    bool has_wraparound = false;
    for (size_t i = 1; i < azimuths.size(); ++i) {
        if (std::abs(static_cast<int16_t>(azimuths[i] - azimuths[i-1])) > AZIMUTH_HALF) {
            has_wraparound = true;
            break;
        }
    }
    
    double az_per_bin = 360.0 / 4096.0;
    
    if (!has_wraparound) {
        double sum = 0.0;
        for (auto az : azimuths) sum += az;
        return sum / azimuths.size();
    } else {
        double sum_sin = 0.0, sum_cos = 0.0;
        for (auto az : azimuths) {
            double rad = az * az_per_bin * M_PI / 180.0;
            sum_sin += std::sin(rad);
            sum_cos += std::cos(rad);
        }
        double avg_rad = std::atan2(sum_sin, sum_cos);
        double avg_deg = avg_rad * 180.0 / M_PI;
        if (avg_deg < 0) avg_deg += 360.0;
        return avg_deg / az_per_bin;
    }
}

uint16_t RBSProcessor::select_best_code(const RangeGrouper::RangeGroup& group, double& best_confidence) {
    std::map<uint16_t, int> code_counts;
    std::map<uint16_t, const RBSReply*> code_to_reply;
    std::map<uint16_t, bool> code_spi;
    std::map<uint16_t, double> code_confidence;
    
    for (const auto* reply : group.rbs_replies) {
        code_counts[reply->code12]++;
        code_to_reply[reply->code12] = reply;
        if (reply->spi) {
            code_spi[reply->code12] = true;
        }
        
        ReplyFeatures features = reply_processor_.analyze_rbs(*reply);
        code_confidence[reply->code12] = std::max(code_confidence[reply->code12], features.confidence);
    }
    
    uint16_t best_code = 0;
    int max_count = 0;
    best_confidence = 0;
    
    for (const auto& [code, count] : code_counts) {
        double confidence = code_confidence[code];
        if (confidence > best_confidence || 
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_code = code;
        }
    }
    
    return best_code;
}

bool RBSProcessor::is_sidelobe(const RBSReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

std::optional<TargetReport> RBSProcessor::process_group(const RangeGrouper::RangeGroup& group) {
    if (group.rbs_replies.empty()) {
        return std::nullopt;
    }
    
    if (static_cast<int>(group.rbs_replies.size()) < min_hits_) {
        VRL_LOG_TRACE(modules::CLUSTER, "RBS group skipped: insufficient replies (" + 
                      std::to_string(group.rbs_replies.size()) + " < " + 
                      std::to_string(min_hits_) + ")");
        return std::nullopt;
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing RBS group: " + 
                  std::to_string(group.rbs_replies.size()) + " replies at range " + 
                  std::to_string(group.nominal_range));
    
    TargetReport report = TargetReport::make_rbs();
    report.type = TargetReport::SourceType::RBS;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    // Собираем азимуты
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.rbs_replies) {
        azimuths.push_back(reply->azimuth);
        // ИСПРАВЛЕНО: используем новый типобезопасный метод
        report.add_source(reply);
    }
    
    // Вычисляем позицию
    double az_per_bin = 360.0 / 4096.0;
    report.azimuth_deg = average_azimuth(azimuths) * az_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_rbs;
    polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    // Выбираем лучший код
    double best_confidence;
    uint16_t best_code = select_best_code(group, best_confidence);
    
    if (best_code == 0 || best_confidence < min_confidence_) {
        VRL_LOG_WARN(modules::CLUSTER, "RBS group rejected: code=0" + 
                     std::to_string(best_code) + ", conf=" + 
                     std::to_string(best_confidence));
        return std::nullopt;
    }
    
    // Заполняем отчет
    report.rbs.mode3a_code = best_code;
    report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
    report.is_garbled = (best_confidence < garbled_threshold_);
    
    // Проверяем SPI
    for (const auto* reply : group.rbs_replies) {
        if (reply->spi) {
            report.rbs.spi = true;
            break;
        }
    }
    
    // Проверяем SLS
    for (const auto* reply : group.rbs_replies) {
        if (is_sidelobe(*reply)) {
            report.is_sls_blanked = true;
            break;
        }
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "RBS target: code=0" + std::to_string(best_code) + 
                  ", conf=" + std::to_string(best_confidence) +
                  ", sources=" + std::to_string(report.sources.size()));
    
    return report;
}

} // namespace radar
} // namespace vrl
