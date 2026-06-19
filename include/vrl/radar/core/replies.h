// include/vrl/radar/core/replies.h
#pragma once

#include "types.h"
#include "object_pool.hpp"
#include <array>
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include <variant>
#include <memory>

namespace vrl {
namespace radar {

// ============================================================================
// RBS REPLY
// ============================================================================

struct RBSReply {
    static constexpr size_t ETHER_POSITIONS = 18;
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes{};
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes_sls{};
    
    uint16_t code12{0};
    bool spi{false};
    std::array<uint8_t, 12> confidence{};
    
    uint16_t azimuth{0};
    uint16_t range{0};
    double x{0.0}, y{0.0};
    bool is_valid{false};
    
    uint8_t f1() const { return ether_amplitudes[0]; }
    uint8_t f2() const { return ether_amplitudes[14]; }
    uint8_t mid_pulse() const { return ether_amplitudes[7]; }
    uint8_t spi_pulse() const { return ether_amplitudes[17]; }
    
    uint8_t bit(size_t idx) const {
        static constexpr uint8_t ether_to_code12[12] = {
            1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13
        };
        if (idx < 12) {
            return ether_amplitudes[ether_to_code12[idx]];
        }
        return 0;
    }
    
    uint8_t bit_sls(size_t idx) const {
        static constexpr uint8_t ether_to_code12[12] = {
            1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13
        };
        if (idx < 12) return ether_amplitudes_sls[ether_to_code12[idx]];
        return 0;
    }
    
    // Для пула объектов
    void reset() {
        ether_amplitudes.fill(0);
        ether_amplitudes_sls.fill(0);
        code12 = 0;
        spi = false;
        confidence.fill(0);
        azimuth = 0;
        range = 0;
        x = 0.0;
        y = 0.0;
        is_valid = false;
    }
};

// ============================================================================
// UVD REPLY
// ============================================================================

struct UVDReply {
    static constexpr size_t ETHER_POSITIONS = 80;
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes{};
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes_sls{};
    
    uint32_t data20{0};
    uint32_t error_mask{0};
    
    uint16_t azimuth{0};
    uint16_t range{0};
    double x{0.0}, y{0.0};
    bool is_valid{false};
    
    uint8_t left_pulse(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes[repeat * 40 + bit_idx * 2];
    }
    
    uint8_t right_pulse(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes[repeat * 40 + bit_idx * 2 + 1];
    }
    
    void reset() {
        ether_amplitudes.fill(0);
        ether_amplitudes_sls.fill(0);
        data20 = 0;
        error_mask = 0;
        azimuth = 0;
        range = 0;
        x = 0.0;
        y = 0.0;
        is_valid = false;
    }
};

// ============================================================================
// SCAN REPLIES
// ============================================================================

struct ScanReplies {
    uint16_t azimuth{0};
    std::vector<RBSReply> rbs_replies;
    std::vector<UVDReply> uvd_replies;
    uint32_t timestamp_ms{0};
    
    ScanReplies() = default;
    ScanReplies(uint16_t az, uint32_t ts) : azimuth(az), timestamp_ms(ts) {}
    
    bool has_replies() const {
        return !rbs_replies.empty() || !uvd_replies.empty();
    }
    
    size_t reply_count() const {
        return rbs_replies.size() + uvd_replies.size();
    }
};

// ============================================================================
// REPLY POOLS
// ============================================================================

/**
 * @brief Глобальные пулы для RBS и UVD ответов
 */
class ReplyPools {
public:
    static ReplyPools& instance() {
        static ReplyPools instance;
        return instance;
    }
    
    core::ObjectPool<RBSReply>& rbs_pool() { return rbs_pool_; }
    core::ObjectPool<UVDReply>& uvd_pool() { return uvd_pool_; }
    
    /**
     * @brief Получить RBS ответ из пула
     */
    core::PooledObject<RBSReply> acquire_rbs() {
        return core::PooledObject<RBSReply>(rbs_pool_, rbs_pool_.acquire());
    }
    
    /**
     * @brief Получить UVD ответ из пула
     */
    core::PooledObject<UVDReply> acquire_uvd() {
        return core::PooledObject<UVDReply>(uvd_pool_, uvd_pool_.acquire());
    }
    
    /**
     * @brief Получить статистику пулов
     */
    struct PoolStats {
        core::ObjectPool<RBSReply>::Stats rbs_stats;
        core::ObjectPool<UVDReply>::Stats uvd_stats;
    };
    
    PoolStats get_stats() const {
        PoolStats stats;
        stats.rbs_stats = rbs_pool_.get_stats();
        stats.uvd_stats = uvd_pool_.get_stats();
        return stats;
    }
    
private:
    ReplyPools() = default;
    
    core::ObjectPool<RBSReply> rbs_pool_{64, 4096};
    core::ObjectPool<UVDReply> uvd_pool_{64, 4096};
};

// ============================================================================
// TARGET REPORT
// ============================================================================

using ReplySource = std::variant<const RBSReply*, const UVDReply*>;

struct TargetReport {
    enum class SourceType { RBS, UVD } type;
    
    double x{0.0}, y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    struct RBSData {
        uint16_t mode3a_code{0};
        uint16_t modec_altitude{0};
        bool spi{false};
    };
    
    struct UVDData {
        uint32_t raw_data20{0};
        uint8_t octal_id[5]{};
        uint16_t altitude{0};
        uint8_t fuel{0};
        bool pressure_ref{false};
        
        UVDData() {
            raw_data20 = 0;
            octal_id[0] = octal_id[1] = octal_id[2] = octal_id[3] = octal_id[4] = 0;
            altitude = 0;
            fuel = 0;
            pressure_ref = false;
        }
    };
    
    RBSData rbs;
    UVDData uvd;
    
    uint8_t signal_strength{0};
    bool is_reflection{false};
    bool is_sls_blanked{false};
    bool is_garbled{false};
    
    std::vector<ReplySource> sources;
    
    TargetReport() : type(SourceType::RBS) {
        x = 0.0;
        y = 0.0;
        azimuth_deg = 0.0;
        range_m = 0.0;
        signal_strength = 0;
        is_reflection = false;
        is_sls_blanked = false;
        is_garbled = false;
    }
    
    static TargetReport make_rbs() {
        TargetReport report;
        report.type = SourceType::RBS;
        return report;
    }
    
    static TargetReport make_uvd() {
        TargetReport report;
        report.type = SourceType::UVD;
        return report;
    }
    
    void add_source(const RBSReply* source) {
        sources.emplace_back(source);
    }
    
    void add_source(const UVDReply* source) {
        sources.emplace_back(source);
    }
    
    std::vector<const RBSReply*> get_rbs_sources() const {
        std::vector<const RBSReply*> result;
        for (const auto& s : sources) {
            if (const auto* rbs = std::get_if<const RBSReply*>(&s)) {
                result.push_back(*rbs);
            }
        }
        return result;
    }
    
    std::vector<const UVDReply*> get_uvd_sources() const {
        std::vector<const UVDReply*> result;
        for (const auto& s : sources) {
            if (const auto* uvd = std::get_if<const UVDReply*>(&s)) {
                result.push_back(*uvd);
            }
        }
        return result;
    }
    
    bool has_sources() const {
        return !sources.empty();
    }
    
    void clear_sources() {
        sources.clear();
    }
};

template<typename Visitor>
void visit_sources(const std::vector<ReplySource>& sources, Visitor&& visitor) {
    for (const auto& source : sources) {
        std::visit([&visitor](const auto* ptr) {
            if constexpr (std::is_same_v<decltype(ptr), const RBSReply*>) {
                visitor(ptr);
            } else if constexpr (std::is_same_v<decltype(ptr), const UVDReply*>) {
                visitor(ptr);
            }
        }, source);
    }
}

} // namespace radar
} // namespace vrl
