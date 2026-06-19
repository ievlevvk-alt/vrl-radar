// src/processing/uvd_processor.cpp
#include "vrl/radar/processing/uvd_processor.h"
#include "vrl/radar/utils/utils.h"
#include "vrl/radar/utils/logger.h"
#include <map>
#include <cmath>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

UVDProcessor::UVDProcessor(const RadarConfig& config)
    : config_(config), reply_processor_(config) {
    VRL_LOG_DEBUG(modules::CLUSTER, "UVDProcessor initialized");
}

double UVDProcessor::average_azimuth(const std::vector<uint16_t>& azimuths) {
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

uint32_t UVDProcessor::select_best_data(const RangeGrouper::RangeGroup& group, double& best_confidence) {
    std::map<uint32_t, int> data_counts;
    std::map<uint32_t, const UVDReply*> data_to_reply;
    std::map<uint32_t, double> data_confidence;
    
    for (const auto* reply : group.uvd_replies) {
        data_counts[reply->data20]++;
        data_to_reply[reply->data20] = reply;
        
        ReplyFeatures features = reply_processor_.analyze_uvd(*reply);
        data_confidence[reply->data20] = std::max(data_confidence[reply->data20], features.confidence);
    }
    
    uint32_t best_data = 0;
    int max_count = 0;
    best_confidence = 0;
    
    for (const auto& [data, count] : data_counts) {
        double confidence = data_confidence[data];
        if (confidence > best_confidence ||
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_data = data;
        }
    }
    
    return best_data;
}

bool UVDProcessor::is_sidelobe(const UVDReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

void UVDProcessor::decode_uvd_info(uint32_t data20, TargetReport& report) {
    report.uvd.raw_data20 = data20;
    
    uint32_t octal_part = data20 & 0x7FFFF;
    for (int i = 4; i >= 0; --i) {
        report.uvd.octal_id[i] = (octal_part >> (i * 3)) & 0x7;
    }
    
    report.uvd.altitude = (data20 >> 3) & 0x7FF;
    report.uvd.fuel = (data20 >> 14) & 0x0F;
    report.uvd.pressure_ref = (data20 >> 18) & 0x01;
}

std::optional<TargetReport> UVDProcessor::process_group(const RangeGrouper::RangeGroup& group) {
    if (group.uvd_replies.empty()) {
        return std::nullopt;
    }
    
    if (static_cast<int>(group.uvd_replies.size()) < min_hits_) {
        VRL_LOG_TRACE(modules::CLUSTER, "UVD group skipped: insufficient replies (" + 
                      std::to_string(group.uvd_replies.size()) + " < " + 
                      std::to_string(min_hits_) + ")");
        return std::nullopt;
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "Processing UVD group: " + 
                  std::to_string(group.uvd_replies.size()) + " replies at range " + 
                  std::to_string(group.nominal_range));
    
    TargetReport report = TargetReport::make_uvd();
    report.type = TargetReport::SourceType::UVD;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    // Собираем азимуты
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.uvd_replies) {
        azimuths.push_back(reply->azimuth);
        report.sources.push_back(reply);
    }
    
    // Вычисляем позицию
    double az_per_bin = 360.0 / 4096.0;
    report.azimuth_deg = average_azimuth(azimuths) * az_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_uvd;
    polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    // Выбираем лучшие данные
    double best_confidence;
    uint32_t best_data = select_best_data(group, best_confidence);
    
    if (best_data == 0 || best_confidence < min_confidence_) {
        VRL_LOG_WARN(modules::CLUSTER, "UVD group rejected: data=0x" + 
                     std::to_string(best_data) + ", conf=" + 
                     std::to_string(best_confidence));
        return std::nullopt;
    }
    
    // Декодируем информацию
    decode_uvd_info(best_data, report);
    report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
    report.is_garbled = (best_confidence < garbled_threshold_);
    
    // Проверяем ошибки
    for (const auto* reply : group.uvd_replies) {
        if (reply->error_mask != 0) {
            report.is_garbled = true;
            break;
        }
    }
    
    // Проверяем SLS
    for (const auto* reply : group.uvd_replies) {
        if (is_sidelobe(*reply)) {
            report.is_sls_blanked = true;
            break;
        }
    }
    
    VRL_LOG_TRACE(modules::CLUSTER, "UVD target: data=0x" + std::to_string(best_data) + 
                  ", conf=" + std::to_string(best_confidence));
    
    return report;
}

} // namespace radar
} // namespace vrl
