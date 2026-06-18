// include/vrl/radar/simulation/simulator.h
#pragma once

#include "../core/replies.h"
#include "../core/config.h"
#include <random>
#include <memory>

namespace vrl {
namespace radar {

// ============================================================================
// REPLY SIMULATOR
// ============================================================================

class ReplySimulator {
public:
    explicit ReplySimulator(const SimulatorConfig& config);
    
    // Генерация одиночных ответов
    RBSReply generate_rbs(
        uint16_t azimuth,
        uint16_t range,
        uint16_t code12,
        bool spi = false
    );
    
    UVDReply generate_uvd(
        uint16_t azimuth,
        uint16_t range,
        uint32_t data20
    );
    
    // Генерация перекрытий
    struct OverlapResult {
        std::vector<RBSReply> rbs_mixture;
        std::vector<UVDReply> uvd_mixture;
    };
    
    OverlapResult mix_two_rbs(
        const RBSReply& r1,
        const RBSReply& r2,
        int16_t range_offset,
        double amp_ratio = 1.0
    );
    
    OverlapResult mix_two_uvd(
        const UVDReply& u1,
        const UVDReply& u2,
        int16_t range_offset,
        double amp_ratio = 1.0
    );
    
    std::mt19937& rng() { return rng_; }
    
private:
    void add_noise_to_amplitudes(
        std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps,
        double snr_db
    );
    
    void add_noise_to_amplitudes(
        std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
        double snr_db
    );
    
    void generate_sls_channel_rbs(RBSReply& reply);
    void generate_sls_channel_uvd(UVDReply& reply);
    
    uint32_t compute_uvd_error_mask(
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
        uint32_t original_data20
    );
    
    SimulatorConfig config_;
    std::mt19937 rng_;
};

} // namespace radar
} // namespace vrl
