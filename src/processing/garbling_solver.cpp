// src/processing/garbling_solver.cpp
#include "vrl/radar/processing/garbling_solver.h"
#include "vrl/radar/utils/utils.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <map>
#include <vector>
#include <cstdint>
#include <functional>

namespace vrl {
namespace radar {

// ============================================================================
// GARBLING SOLVER BASE CLASS
// ============================================================================

GarblingSolver::GarblingSolver(const RadarConfig& config) : config_(config) {}

void GarblingSolver::log(const std::string& msg) const {
    if (debug_) {
        std::cout << "[GarblingSolver] " << msg << std::endl;
    }
}

// ============================================================================
// THRESHOLD GARBLING SOLVER
// ============================================================================

ThresholdGarblingSolver::ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold)
    : GarblingSolver(config), threshold_(threshold) {}

std::vector<uint16_t> ThresholdGarblingSolver::detect_possible_codes_rbs(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint16_t> possible_codes;
    std::map<uint16_t, int> code_votes;
    
    for (int i = 0; i < 12; ++i) {
        size_t pos = utils::bit_position(i);
        if (mixture[pos] > threshold_) {
            code_votes[1 << i] += 1;
        }
    }
    
    for (int i = 0; i < 12; i += 2) {
        size_t pos1 = utils::bit_position(i);
        size_t pos2 = utils::bit_position(i+1);
        uint8_t amp1 = mixture[pos1];
        uint8_t amp2 = mixture[pos2];
        
        if (amp1 > threshold_ && amp2 > threshold_) {
            code_votes[(1 << i) | (1 << (i+1))] += 2;
        }
    }
    
    std::vector<std::pair<int, uint16_t>> sorted;
    for (const auto& kv : code_votes) {
        sorted.emplace_back(kv.second, kv.first);
    }
    std::sort(sorted.begin(), sorted.end(), std::greater<std::pair<int, uint16_t>>());
    
    for (const auto& item : sorted) {
        if (item.first >= 2) {
            possible_codes.push_back(item.second);
        }
    }
    
    return possible_codes;
}

bool ThresholdGarblingSolver::check_code_presence_rbs(
    uint16_t code,
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    int matching = 0;
    int total = 0;
    
    for (int i = 0; i < 12; ++i) {
        bool expected = (code >> i) & 1;
        if (mixture[utils::bit_position(i)] > threshold_) {
            if (expected) matching++;
            total++;
        }
    }
    
    return total > 0 && (static_cast<double>(matching) / total >= 0.6);
}

std::vector<uint32_t> ThresholdGarblingSolver::detect_possible_data_uvd(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint32_t> possible_data;
    // TODO: Implement UVD data detection
    return possible_data;
}

SeparationResult<RBSReply> ThresholdGarblingSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    SeparationResult<RBSReply> result;
    result.method_used = "Threshold";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    if (mixture.size() == 1) {
        result.separated_replies = mixture;
        result.confidence = 1.0;
        return result;
    }
    
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> total{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total.size(); ++i) {
            total[i] = std::min(255, static_cast<int>(total[i]) + reply.ether_amplitudes[i]);
        }
    }
    
    auto possible_codes = detect_possible_codes_rbs(total);
    
    if (!expected_codes.empty()) {
        std::vector<uint16_t> filtered;
        for (uint16_t code : expected_codes) {
            if (std::find(possible_codes.begin(), possible_codes.end(), code) != possible_codes.end()) {
                filtered.push_back(code);
            }
        }
        if (!filtered.empty()) {
            possible_codes = filtered;
        }
    }
    
    for (uint16_t code : possible_codes) {
        RBSReply separated;
        separated.code12 = code;
        separated.azimuth = mixture[0].azimuth;
        separated.range = mixture[0].range;
        separated.spi = false;
        
        for (size_t i = 0; i < RBSReply::ETHER_POSITIONS; ++i) {
            if (i == 0 || i == 14) {
                separated.ether_amplitudes[i] = total[i];
            } else {
                int bit_idx = -1;
                for (int b = 0; b < 12; ++b) {
                    if (utils::bit_position(b) == i) {
                        bit_idx = b;
                        break;
                    }
                }
                if (bit_idx >= 0 && ((code >> bit_idx) & 1)) {
                    separated.ether_amplitudes[i] = total[i];
                } else {
                    separated.ether_amplitudes[i] = 0;
                }
            }
        }
        
        result.separated_replies.push_back(separated);
    }
    
    if (result.separated_replies.size() == mixture.size()) {
        result.confidence = 0.9;
        result.ambiguous = false;
    } else if (result.separated_replies.size() > 0) {
        result.confidence = 0.7;
        result.ambiguous = true;
    } else {
        result.confidence = 0.3;
        result.ambiguous = true;
    }
    
    return result;
}

SeparationResult<UVDReply> ThresholdGarblingSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_codes) {
    
    SeparationResult<UVDReply> result;
    result.method_used = "Threshold";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    if (mixture.size() == 1) {
        result.separated_replies = mixture;
        result.confidence = 1.0;
        return result;
    }
    
    // TODO: Полная реализация для UVD
    result.separated_replies.push_back(mixture[0]);
    result.confidence = 0.5;
    result.ambiguous = true;
    
    return result;
}

} // namespace radar
} // namespace vrl
