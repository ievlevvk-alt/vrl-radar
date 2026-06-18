// src/utils/utils.cpp
#include "vrl/radar/utils/utils.h"
#include "vrl/radar/utils/logger.h"
#include <algorithm>
#include <cmath>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {
namespace utils {

size_t bit_position(int bit_idx) {
    static constexpr uint8_t bit_to_ether[12] = {1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13};
    if (bit_idx >= 0 && bit_idx < 12) {
        return bit_to_ether[bit_idx];
    }
    return 0;
}

bool is_potential_overlap(
    uint16_t az1, uint16_t r1,
    uint16_t az2, uint16_t r2,
    const RadarConfig& cfg) {
    
    int16_t az_diff = std::abs(static_cast<int16_t>(az1 - az2));
    az_diff = std::min(az_diff, static_cast<int16_t>(4096 - az_diff));
    
    uint16_t range_diff = std::abs(static_cast<int16_t>(r1 - r2));
    
    bool overlap = az_diff <= cfg.max_azimuth_diff_for_overlap &&
                   range_diff <= cfg.max_range_diff_for_overlap;
    
    if (overlap) {
        VRL_LOG_TRACE(modules::UTILS, "Overlap detected: az_diff=" + 
                      std::to_string(az_diff) + ", range_diff=" + 
                      std::to_string(range_diff));
    }
    
    return overlap;
}

std::array<uint8_t, 12> compute_rbs_confidence(const RBSReply& reply) {
    std::array<uint8_t, 12> conf{};
    
    uint16_t ref = (reply.f1() + reply.f2()) / 2;
    if (ref == 0) return conf;
    
    uint16_t threshold = ref / 2;
    
    for (size_t i = 0; i < 12; ++i) {
        if (reply.bit(i) >= threshold) {
            conf[i] = static_cast<uint8_t>(
                std::min(100, (reply.bit(i) * 100) / ref)
            );
        } else {
            conf[i] = 0;
        }
    }
    
    return conf;
}

bool validate_rbs(const RBSReply& reply, const RadarConfig& cfg) {
    bool valid = reply.f1() >= cfg.min_amplitude && reply.f2() >= cfg.min_amplitude;
    
    if (!valid) {
        VRL_LOG_TRACE(modules::UTILS, "RBS validation failed: f1=" + 
                      std::to_string(reply.f1()) + ", f2=" + 
                      std::to_string(reply.f2()) + ", min=" + 
                      std::to_string(cfg.min_amplitude));
    }
    
    return valid;
}

bool validate_uvd(const UVDReply& reply, const RadarConfig& cfg) {
    (void)cfg;
    bool valid = !(reply.data20 == 0 && reply.error_mask == 0xFFFFF);
    
    if (!valid) {
        VRL_LOG_TRACE(modules::UTILS, "UVD validation failed: data=0x" + 
                      std::to_string(reply.data20) + ", mask=0x" + 
                      std::to_string(reply.error_mask));
    }
    
    return valid;
}

bool is_sidelobe(const RBSReply& reply, double threshold_db) {
    if (reply.ether_amplitudes_sls[0] == 0) return false;
    
    double ratio = static_cast<double>(reply.ether_amplitudes_sls[0]) / 
                   static_cast<double>(reply.ether_amplitudes[0]);
    
    double ratio_db = 20.0 * std::log10(ratio);
    bool sidelobe = ratio_db > threshold_db;
    
    if (sidelobe) {
        VRL_LOG_TRACE(modules::UTILS, "RBS sidelobe detected: ratio=" + 
                      std::to_string(ratio_db) + "dB > " + std::to_string(threshold_db) + "dB");
    }
    
    return sidelobe;
}

bool is_sidelobe(const UVDReply& reply, double threshold_db) {
    if (reply.ether_amplitudes_sls[0] == 0) return false;
    
    double ratio = static_cast<double>(reply.ether_amplitudes_sls[0]) / 
                   static_cast<double>(reply.ether_amplitudes[0]);
    
    double ratio_db = 20.0 * std::log10(ratio);
    bool sidelobe = ratio_db > threshold_db;
    
    if (sidelobe) {
        VRL_LOG_TRACE(modules::UTILS, "UVD sidelobe detected: ratio=" + 
                      std::to_string(ratio_db) + "dB > " + std::to_string(threshold_db) + "dB");
    }
    
    return sidelobe;
}

} // namespace utils
} // namespace radar
} // namespace vrl
