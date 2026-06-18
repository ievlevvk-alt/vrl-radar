// include/vrl/radar/processing/reply_processor.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include <optional>
#include <vector>

namespace vrl {
namespace radar {

struct ReplyFeatures {
    double snr_estimate{0.0};
    double pulse_stability{0.0};
    bool has_framing{false};
    bool has_spi{false};
    int bit_errors{0};
    double confidence{0.0};
};

class ReplyProcessor {
public:
    explicit ReplyProcessor(const RadarConfig& config);
    
    ReplyFeatures analyze_rbs(const RBSReply& reply);
    ReplyFeatures analyze_uvd(const UVDReply& reply);
    
    void normalize_amplitudes(RBSReply& reply);
    void normalize_amplitudes(UVDReply& reply);
    
    uint16_t decode_rbs_with_errors(const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps);
    uint32_t decode_uvd_with_errors(const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps);
    
    double estimate_snr(const RBSReply& reply);
    double estimate_snr(const UVDReply& reply);
    
private:
    RadarConfig config_;
    double calculate_pulse_ratio(const RBSReply& reply);
    double calculate_pulse_ratio(const UVDReply& reply);
    int count_bit_errors(const RBSReply& reply);
};

} // namespace radar
} // namespace vrl
