// src/processing/garbling_solver_iterative.cpp
#include "vrl/radar/processing/garbling_solver.h"
#include "vrl/radar/utils/utils.h"
#include <cmath>
#include <algorithm>

namespace vrl {
namespace radar {

IterativeSubtractionSolver::IterativeSubtractionSolver(
    const RadarConfig& config, 
    double min_amplitude_ratio,
    int max_iterations)
    : GarblingSolver(config)
    , min_amplitude_ratio_(min_amplitude_ratio)
    , max_iterations_(max_iterations) {}

template<typename ReplyType>
std::optional<ReplyType> IterativeSubtractionSolver::find_dominant_reply(
    const std::vector<ReplyType>& mixture) {
    
    if (mixture.empty()) return std::nullopt;
    
    auto max_it = std::max_element(mixture.begin(), mixture.end(),
        [](const ReplyType& a, const ReplyType& b) {
            double sum_a = 0, sum_b = 0;
            for (uint8_t amp : a.ether_amplitudes) sum_a += amp;
            for (uint8_t amp : b.ether_amplitudes) sum_b += amp;
            return sum_a < sum_b;
        });
    
    return *max_it;
}

template<typename ReplyType>
std::vector<ReplyType> IterativeSubtractionSolver::subtract_reply(
    const std::vector<ReplyType>& mixture,
    const ReplyType& to_subtract) {
    
    std::vector<ReplyType> result;
    
    for (const auto& reply : mixture) {
        ReplyType remaining = reply;
        
        for (size_t i = 0; i < remaining.ether_amplitudes.size(); ++i) {
            if (remaining.ether_amplitudes[i] > to_subtract.ether_amplitudes[i]) {
                remaining.ether_amplitudes[i] -= to_subtract.ether_amplitudes[i];
            } else {
                remaining.ether_amplitudes[i] = 0;
            }
        }
        
        bool has_content = false;
        for (uint8_t amp : remaining.ether_amplitudes) {
            if (amp > config_.min_amplitude * 2) {
                has_content = true;
                break;
            }
        }
        
        if (has_content) {
            result.push_back(remaining);
        }
    }
    
    return result;
}

double IterativeSubtractionSolver::evaluate_separation_quality(
    const std::vector<RBSReply>& original,
    const std::vector<RBSReply>& separated) {
    
    if (original.empty() || separated.empty()) return 0.0;
    
    return static_cast<double>(separated.size()) / original.size();
}

SeparationResult<RBSReply> IterativeSubtractionSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    SeparationResult<RBSReply> result;
    result.method_used = "IterativeSubtraction";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    std::vector<RBSReply> current = mixture;
    std::vector<RBSReply> extracted;
    
    for (int iter = 0; iter < max_iterations_ && !current.empty(); ++iter) {
        auto dominant = find_dominant_reply(current);
        if (!dominant) break;
        
        uint16_t code = 0;
        for (int i = 0; i < 12; ++i) {
            size_t pos = utils::bit_position(i);
            if (dominant->ether_amplitudes[pos] > config_.min_amplitude * 3) {
                code |= (1 << i);
            }
        }
        dominant->code12 = code;
        
        extracted.push_back(*dominant);
        current = subtract_reply(current, *dominant);
    }
    
    result.separated_replies = extracted;
    result.confidence = evaluate_separation_quality(mixture, extracted);
    result.ambiguous = (extracted.size() < mixture.size());
    
    return result;
}

SeparationResult<UVDReply> IterativeSubtractionSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_codes) {
    
    SeparationResult<UVDReply> result;
    result.method_used = "IterativeSubtraction";
    // TODO: Implement UVD iterative subtraction
    result.confidence = 0.5;
    return result;
}

// Явная инстанциация шаблонов
template std::optional<RBSReply> IterativeSubtractionSolver::find_dominant_reply<RBSReply>(
    const std::vector<RBSReply>&);
template std::vector<RBSReply> IterativeSubtractionSolver::subtract_reply<RBSReply>(
    const std::vector<RBSReply>&, const RBSReply&);

} // namespace radar
} // namespace vrl
