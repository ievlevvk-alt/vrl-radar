// src/simulation/simulator.cpp
#include "vrl/radar/simulation/simulator.h"
#include "vrl/radar/utils/logger.h"
#include <cmath>
#include <algorithm>
#include <random>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

ReplySimulator::ReplySimulator(const SimulatorConfig& config)
    : config_(config), rng_(std::random_device{}()) {
    VRL_LOG_DEBUG(modules::SIMULATOR, "ReplySimulator initialized: RBS SNR=" + 
                  std::to_string(config.rbs.snr_db) + ", UVD SNR=" + 
                  std::to_string(config.uvd.snr_db) + ", SLS=" + 
                  (config.sls.enabled ? "enabled" : "disabled"));
}

void ReplySimulator::add_noise_to_amplitudes(
    std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps,
    double snr_db) {
    
    if (snr_db <= 0) return;
    
    const double BASE_SIGNAL_POWER = 128.0;
    double signal_power = BASE_SIGNAL_POWER;
    double noise_power = signal_power / std::pow(10.0, snr_db / 10.0);
    double noise_std = std::sqrt(noise_power);
    
    std::normal_distribution<double> noise_dist(0.0, noise_std);
    
    int noise_added = 0;
    for (auto& amp : amps) {
        if (amp > 0) {
            double noisy = amp + noise_dist(rng_);
            amp = static_cast<uint8_t>(std::clamp(noisy, 0.0, 255.0));
            noise_added++;
        }
    }
    
    VRL_LOG_TRACE(modules::SIMULATOR, "Added noise to " + std::to_string(noise_added) + 
                  " positions (SNR=" + std::to_string(snr_db) + "dB)");
}

void ReplySimulator::add_noise_to_amplitudes(
    std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
    double snr_db) {
    
    if (snr_db <= 0) return;
    
    const double BASE_SIGNAL_POWER = 128.0;
    double signal_power = BASE_SIGNAL_POWER;
    double noise_power = signal_power / std::pow(10.0, snr_db / 10.0);
    double noise_std = std::sqrt(noise_power);
    
    std::normal_distribution<double> noise_dist(0.0, noise_std);
    
    int noise_added = 0;
    for (auto& amp : amps) {
        if (amp > 0) {
            double noisy = amp + noise_dist(rng_);
            amp = static_cast<uint8_t>(std::clamp(noisy, 0.0, 255.0));
            noise_added++;
        }
    }
    
    VRL_LOG_TRACE(modules::SIMULATOR, "Added noise to " + std::to_string(noise_added) + 
                  " positions (SNR=" + std::to_string(snr_db) + "dB)");
}

void ReplySimulator::generate_sls_channel_rbs(RBSReply& reply) {
    if (!config_.sls.enabled) {
        reply.ether_amplitudes_sls.fill(0);
        return;
    }
    
    std::bernoulli_distribution sidelobe_dist(config_.sls.sidelobe_probability);
    bool is_sidelobe = sidelobe_dist(rng_);
    
    VRL_LOG_TRACE(modules::SIMULATOR, "SLS: " + std::string(is_sidelobe ? "sidelobe" : "mainlobe"));
    
    for (size_t i = 0; i < RBSReply::ETHER_POSITIONS; ++i) {
        if (is_sidelobe) {
            double attenuation = std::pow(10.0, config_.sls.sls_attenuation_db / 20.0);
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * attenuation, 0.0, 255.0)
            );
        } else {
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * config_.sls.main_to_sls_ratio, 0.0, 255.0)
            );
        }
    }
}

void ReplySimulator::generate_sls_channel_uvd(UVDReply& reply) {
    if (!config_.sls.enabled) {
        reply.ether_amplitudes_sls.fill(0);
        return;
    }
    
    std::bernoulli_distribution sidelobe_dist(config_.sls.sidelobe_probability);
    bool is_sidelobe = sidelobe_dist(rng_);
    
    for (size_t i = 0; i < UVDReply::ETHER_POSITIONS; ++i) {
        if (is_sidelobe) {
            double attenuation = std::pow(10.0, config_.sls.sls_attenuation_db / 20.0);
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * attenuation, 0.0, 255.0)
            );
        } else {
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * config_.sls.main_to_sls_ratio, 0.0, 255.0)
            );
        }
    }
}

RBSReply ReplySimulator::generate_rbs(
    uint16_t azimuth,
    uint16_t range,
    uint16_t code12,
    bool spi) {
    
    VRL_LOG_TRACE(modules::SIMULATOR, "Generating RBS: az=" + std::to_string(azimuth) + 
                  ", range=" + std::to_string(range) + ", code=0" + 
                  std::to_string(code12) + ", spi=" + (spi ? "true" : "false"));
    
    // Используем пул для RBS
    auto pooled = ReplyPools::instance().acquire_rbs();
    RBSReply& reply = *pooled;
    
    reply.azimuth = azimuth;
    reply.range = range;
    reply.code12 = code12 & 0x0FFF;
    reply.spi = spi;
    
    uint8_t base_amp = 200;
    reply.ether_amplitudes.fill(0);
    
    reply.ether_amplitudes[0] = base_amp;
    
    for (int i = 0; i < 6; ++i) {
        if ((code12 >> i) & 1) {
            reply.ether_amplitudes[1 + i] = base_amp;
        }
    }
    
    reply.ether_amplitudes[7] = 0;
    
    for (int i = 0; i < 6; ++i) {
        if ((code12 >> (6 + i)) & 1) {
            reply.ether_amplitudes[8 + i] = base_amp;
        }
    }
    
    reply.ether_amplitudes[14] = base_amp;
    reply.ether_amplitudes[15] = 0;
    reply.ether_amplitudes[16] = 0;
    
    if (spi) {
        reply.ether_amplitudes[17] = base_amp;
    }
    
    add_noise_to_amplitudes(reply.ether_amplitudes, config_.rbs.snr_db);
    
    if (config_.rbs.amp_variation > 0) {
        std::normal_distribution<double> amp_dist(1.0, config_.rbs.amp_variation);
        const double AMP_VARIATION_MIN = 0.5;
        const double AMP_VARIATION_MAX = 1.5;
        int varied = 0;
        for (auto& amp : reply.ether_amplitudes) {
            if (amp > 0) {
                double factor = std::clamp(amp_dist(rng_), AMP_VARIATION_MIN, AMP_VARIATION_MAX);
                amp = static_cast<uint8_t>(std::clamp(amp * factor, 0.0, 255.0));
                varied++;
            }
        }
        VRL_LOG_TRACE(modules::SIMULATOR, "Varied " + std::to_string(varied) + " amplitudes");
    }
    
    generate_sls_channel_rbs(reply);
    reply.is_valid = true;
    
    double az_per_bin = 360.0 / 4096.0;
    double az_rad = azimuth * az_per_bin * M_PI / 180.0;
    double range_m = range * config_.radar.range_bin_rbs;
    reply.x = range_m * sin(az_rad);
    reply.y = range_m * cos(az_rad);
    
    // Возвращаем копию, т.к. функция ожидает RBSReply по значению
    return reply;
}

UVDReply ReplySimulator::generate_uvd(
    uint16_t azimuth,
    uint16_t range,
    uint32_t data20) {
    
    VRL_LOG_TRACE(modules::SIMULATOR, "Generating UVD: az=" + std::to_string(azimuth) + 
                  ", range=" + std::to_string(range) + ", data=0x" + std::to_string(data20));
    
    // Используем пул для UVD
    auto pooled = ReplyPools::instance().acquire_uvd();
    UVDReply& reply = *pooled;
    
    reply.azimuth = azimuth;
    reply.range = range;
    reply.data20 = data20 & 0x0FFFFF;
    
    uint8_t base_amp = 200;
    reply.ether_amplitudes.fill(0);
    
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 20; ++i) {
            bool bit = (data20 >> i) & 1;
            size_t offset = repeat * 40 + i * 2;
            
            if (!bit) {
                reply.ether_amplitudes[offset] = base_amp;
                reply.ether_amplitudes[offset + 1] = 0;
            } else {
                reply.ether_amplitudes[offset] = 0;
                reply.ether_amplitudes[offset + 1] = base_amp;
            }
        }
    }
    
    add_noise_to_amplitudes(reply.ether_amplitudes, config_.uvd.snr_db);
    generate_sls_channel_uvd(reply);
    
    const uint8_t UVD_ERROR_THRESHOLD = 50;
    reply.error_mask = 0;
    int errors = 0;
    
    for (int i = 0; i < 20; ++i) {
        uint8_t left1 = reply.ether_amplitudes[i * 2];
        uint8_t right1 = reply.ether_amplitudes[i * 2 + 1];
        uint8_t left2 = reply.ether_amplitudes[40 + i * 2];
        uint8_t right2 = reply.ether_amplitudes[40 + i * 2 + 1];
        
        bool repeat1_one = (left1 <= UVD_ERROR_THRESHOLD && right1 > UVD_ERROR_THRESHOLD);
        bool repeat2_one = (left2 <= UVD_ERROR_THRESHOLD && right2 > UVD_ERROR_THRESHOLD);
        bool repeat1_zero = (left1 > UVD_ERROR_THRESHOLD && right1 <= UVD_ERROR_THRESHOLD);
        bool repeat2_zero = (left2 > UVD_ERROR_THRESHOLD && right2 <= UVD_ERROR_THRESHOLD);
        
        if (repeat1_one != repeat2_one || repeat1_zero != repeat2_zero) {
            reply.error_mask |= (1 << i);
            errors++;
        }
    }
    
    if (errors > 0) {
        VRL_LOG_TRACE(modules::SIMULATOR, "UVD: " + std::to_string(errors) + " bit errors");
    }
    
    reply.is_valid = true;
    
    double az_per_bin = 360.0 / 4096.0;
    double az_rad = azimuth * az_per_bin * M_PI / 180.0;
    double range_m = range * config_.radar.range_bin_uvd;
    reply.x = range_m * sin(az_rad);
    reply.y = range_m * cos(az_rad);
    
    return reply;
}

ReplySimulator::OverlapResult ReplySimulator::mix_two_rbs(
    const RBSReply& r1,
    const RBSReply& r2,
    int16_t range_offset,
    double amp_ratio) {
    
    VRL_LOG_DEBUG(modules::SIMULATOR, "Mixing two RBS replies: offset=" + 
                  std::to_string(range_offset) + ", ratio=" + std::to_string(amp_ratio));
    
    OverlapResult result;
    result.rbs_mixture.push_back(r1);
    result.rbs_mixture.push_back(r2);
    return result;
}

ReplySimulator::OverlapResult ReplySimulator::mix_two_uvd(
    const UVDReply& u1,
    const UVDReply& u2,
    int16_t range_offset,
    double amp_ratio) {
    
    VRL_LOG_DEBUG(modules::SIMULATOR, "Mixing two UVD replies: offset=" + 
                  std::to_string(range_offset) + ", ratio=" + std::to_string(amp_ratio));
    
    OverlapResult result;
    result.uvd_mixture.push_back(u1);
    result.uvd_mixture.push_back(u2);
    return result;
}

uint32_t ReplySimulator::compute_uvd_error_mask(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
    uint32_t original_data20) {
    
    (void)original_data20;
    
    const uint8_t UVD_ERROR_THRESHOLD = 50;
    uint32_t mask = 0;
    
    for (int i = 0; i < 20; ++i) {
        uint8_t left1 = amps[i * 2];
        uint8_t right1 = amps[i * 2 + 1];
        uint8_t left2 = amps[40 + i * 2];
        uint8_t right2 = amps[40 + i * 2 + 1];
        
        bool repeat1_one = (left1 <= UVD_ERROR_THRESHOLD && right1 > UVD_ERROR_THRESHOLD);
        bool repeat2_one = (left2 <= UVD_ERROR_THRESHOLD && right2 > UVD_ERROR_THRESHOLD);
        bool repeat1_zero = (left1 > UVD_ERROR_THRESHOLD && right1 <= UVD_ERROR_THRESHOLD);
        bool repeat2_zero = (left2 > UVD_ERROR_THRESHOLD && right2 <= UVD_ERROR_THRESHOLD);
        
        if (repeat1_one != repeat2_one || repeat1_zero != repeat2_zero) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

} // namespace radar
} // namespace vrl
