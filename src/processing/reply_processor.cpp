// src/processing/reply_processor.cpp
#include "vrl/radar/processing/reply_processor.h"
#include "vrl/radar/utils/utils.h"
#include <cmath>
#include <algorithm>

namespace vrl {
namespace radar {

ReplyProcessor::ReplyProcessor(const RadarConfig& config) : config_(config) {}

ReplyFeatures ReplyProcessor::analyze_rbs(const RBSReply& reply) {
    ReplyFeatures features;
    
    // КОНСТАНТЫ ДЛЯ РАСЧЕТА УВЕРЕННОСТИ
    const double WEIGHT_FRAMING = 0.3;
    const double WEIGHT_SNR = 0.3;
    const double WEIGHT_STABILITY = 0.2;
    const double WEIGHT_ERRORS = 0.2;
    const double MAX_SNR_DB = 30.0;
    
    features.snr_estimate = estimate_snr(reply);
    features.has_framing = (reply.f1() > config_.min_amplitude && 
                            reply.f2() > config_.min_amplitude);
    features.has_spi = (reply.spi_pulse() > config_.min_amplitude);
    features.pulse_stability = calculate_pulse_ratio(reply);
    features.bit_errors = count_bit_errors(reply);
    
    features.confidence = (features.has_framing ? WEIGHT_FRAMING : 0) +
                          (features.snr_estimate > 15 ? WEIGHT_SNR : 
                           features.snr_estimate / MAX_SNR_DB * WEIGHT_SNR) +
                          (features.pulse_stability > 0.7 ? WEIGHT_STABILITY : 
                           features.pulse_stability / 3.5 * WEIGHT_STABILITY) +
                          (features.bit_errors == 0 ? WEIGHT_ERRORS : 0);
    
    return features;
}

ReplyFeatures ReplyProcessor::analyze_uvd(const UVDReply& reply) {
    ReplyFeatures features;
    
    // КОНСТАНТЫ ДЛЯ РАСЧЕТА УВЕРЕННОСТИ UVD
    const double WEIGHT_SNR = 0.5;
    const double WEIGHT_ERRORS = 0.3;
    const double WEIGHT_STABILITY = 0.2;
    const double MAX_SNR_DB = 30.0;
    
    features.snr_estimate = estimate_snr(reply);
    features.pulse_stability = calculate_pulse_ratio(reply);
    features.bit_errors = (reply.error_mask != 0) ? __builtin_popcount(reply.error_mask) : 0;
    
    features.confidence = (features.snr_estimate > 15 ? WEIGHT_SNR : 
                           features.snr_estimate / MAX_SNR_DB * WEIGHT_SNR) +
                          (features.bit_errors == 0 ? WEIGHT_ERRORS : 0) +
                          (features.pulse_stability > 0.7 ? WEIGHT_STABILITY : 0);
    
    return features;
}

double ReplyProcessor::estimate_snr(const RBSReply& reply) {
    const double MAX_SNR_DB = 30.0;
    double signal = 0;
    double noise = 0;
    int signal_count = 0;
    int noise_count = 0;
    
    for (size_t i = 0; i < reply.ether_amplitudes.size(); ++i) {
        if (reply.ether_amplitudes[i] > config_.min_amplitude) {
            signal += reply.ether_amplitudes[i];
            signal_count++;
        } else {
            noise += reply.ether_amplitudes[i];
            noise_count++;
        }
    }
    
    if (noise_count == 0) return MAX_SNR_DB;
    
    double avg_signal = signal / signal_count;
    double avg_noise = noise / noise_count;
    
    if (avg_noise == 0) return MAX_SNR_DB;
    
    double snr_linear = avg_signal / avg_noise;
    return 20.0 * std::log10(snr_linear);
}

double ReplyProcessor::estimate_snr(const UVDReply& reply) {
    const double MAX_SNR_DB = 30.0;
    double signal = 0;
    double noise = 0;
    int signal_count = 0;
    int noise_count = 0;
    
    for (size_t i = 0; i < reply.ether_amplitudes.size(); ++i) {
        if (reply.ether_amplitudes[i] > config_.min_amplitude) {
            signal += reply.ether_amplitudes[i];
            signal_count++;
        } else {
            noise += reply.ether_amplitudes[i];
            noise_count++;
        }
    }
    
    if (noise_count == 0) return MAX_SNR_DB;
    
    double avg_signal = signal / signal_count;
    double avg_noise = noise / noise_count;
    
    if (avg_noise == 0) return MAX_SNR_DB;
    
    double snr_linear = avg_signal / avg_noise;
    return 20.0 * std::log10(snr_linear);
}

double ReplyProcessor::calculate_pulse_ratio(const RBSReply& reply) {
    double framing_avg = (reply.f1() + reply.f2()) / 2.0;
    if (framing_avg < 1) return 0;
    
    double data_sum = 0;
    int data_count = 0;
    
    for (size_t i = 1; i <= 13; ++i) {
        if (i != 7) {
            data_sum += reply.ether_amplitudes[i];
            data_count++;
        }
    }
    
    double data_avg = data_sum / data_count;
    return data_avg / framing_avg;
}

double ReplyProcessor::calculate_pulse_ratio(const UVDReply& reply) {
    double correlation = 0;
    int count = 0;
    
    for (int i = 0; i < 20; ++i) {
        uint8_t left1 = reply.left_pulse(i, 0);
        uint8_t right1 = reply.right_pulse(i, 0);
        uint8_t left2 = reply.left_pulse(i, 1);
        uint8_t right2 = reply.right_pulse(i, 1);
        
        bool bit1 = left1 > right1;
        bool bit2 = left2 > right2;
        
        if (bit1 == bit2) correlation++;
        count++;
    }
    
    return correlation / count;
}

int ReplyProcessor::count_bit_errors(const RBSReply& reply) {
    int errors = 0;
    int high_threshold = config_.min_amplitude * 3;
    int low_threshold = config_.min_amplitude;
    
    for (size_t i = 0; i < 12; ++i) {
        uint8_t amp = reply.bit(i);
        if (amp > low_threshold && amp < high_threshold) {
            errors++;
        }
    }
    
    return errors;
}

uint16_t ReplyProcessor::decode_rbs_with_errors(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps) {
    
    uint16_t code = 0;
    int ref = (amps[0] + amps[14]) / 2;
    
    if (ref == 0) return 0;
    
    int threshold = ref / 2;
    
    for (int i = 0; i < 12; ++i) {
        size_t pos = utils::bit_position(i);
        if (amps[pos] > threshold) {
            code |= (1 << i);
        }
    }
    
    return code;
}

uint32_t ReplyProcessor::decode_uvd_with_errors(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps) {
    
    uint32_t data = 0;
    uint8_t threshold = config_.min_amplitude * 2;
    
    for (int i = 0; i < 20; ++i) {
        uint8_t left1 = amps[i * 2];
        uint8_t right1 = amps[i * 2 + 1];
        uint8_t left2 = amps[40 + i * 2];
        uint8_t right2 = amps[40 + i * 2 + 1];
        
        bool bit1 = (left1 > threshold && right1 <= threshold);
        bool bit1_one = (left1 <= threshold && right1 > threshold);
        bool bit2 = (left2 > threshold && right2 <= threshold);
        bool bit2_one = (left2 <= threshold && right2 > threshold);
        
        if ((bit1 && bit2) || (bit1 && !bit2 && !bit2_one) || (bit2 && !bit1 && !bit1_one)) {
            // Бит = 0
        } else if ((bit1_one && bit2_one) || (bit1_one && !bit2 && !bit2_one) || (bit2_one && !bit1 && !bit1_one)) {
            data |= (1 << i);
        }
    }
    
    return data;
}

void ReplyProcessor::normalize_amplitudes(RBSReply& reply) {
    int ref = (reply.f1() + reply.f2()) / 2;
    if (ref == 0) return;
    
    double scale = 200.0 / ref;
    
    for (auto& amp : reply.ether_amplitudes) {
        amp = static_cast<uint8_t>(std::min(255.0, amp * scale));
    }
    for (auto& amp : reply.ether_amplitudes_sls) {
        amp = static_cast<uint8_t>(std::min(255.0, amp * scale));
    }
}

void ReplyProcessor::normalize_amplitudes(UVDReply& reply) {
    int max_amp = 0;
    for (auto amp : reply.ether_amplitudes) {
        if (amp > max_amp) max_amp = amp;
    }
    
    if (max_amp == 0) return;
    
    double scale = 200.0 / max_amp;
    
    for (auto& amp : reply.ether_amplitudes) {
        amp = static_cast<uint8_t>(std::min(255.0, amp * scale));
    }
    for (auto& amp : reply.ether_amplitudes_sls) {
        amp = static_cast<uint8_t>(std::min(255.0, amp * scale));
    }
}

} // namespace radar
} // namespace vrl
