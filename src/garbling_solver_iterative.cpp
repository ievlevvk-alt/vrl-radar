// file: src/garbling_solver_iterative.cpp
#include "radar/garbling_solver.h"
#include <cmath>
#include <algorithm>

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
    
    // Находим ответ с максимальной суммарной амплитудой
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
    
    // Для каждого ответа в смеси
    for (const auto& reply : mixture) {
        ReplyType remaining = reply;
        
        // Вычитаем амплитуды to_subtract
        for (size_t i = 0; i < remaining.ether_amplitudes.size(); ++i) {
            if (remaining.ether_amplitudes[i] > to_subtract.ether_amplitudes[i]) {
                remaining.ether_amplitudes[i] -= to_subtract.ether_amplitudes[i];
            } else {
                remaining.ether_amplitudes[i] = 0;
            }
        }
        
        // Проверяем, осталось ли что-то значимое
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

SeparationResult<RBSReply> IterativeSubtractionSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    SeparationResult<RBSReply> result;
    result.method_used = "IterativeSubtraction";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    std::vector<RBSReply> current_mixture = mixture;
    std::vector<RBSReply> extracted;
    
    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Находим доминирующий ответ
        auto dominant = find_dominant_reply(current_mixture);
        if (!dominant) break;
        
        // Проверяем, достаточно ли он сильный
        double total_energy = 0;
        for (const auto& reply : current_mixture) {
            for (uint8_t amp : reply.ether_amplitudes) {
                total_energy += amp;
            }
        }
        
        double dominant_energy = 0;
        for (uint8_t amp : dominant->ether_amplitudes) {
            dominant_energy += amp;
        }
        
        if (dominant_energy / total_energy < min_amplitude_ratio_) {
            break;  // Слишком слабый для надежного выделения
        }
        
        // Извлекаем доминирующий ответ
        extracted.push_back(*dominant);
        
        // Вычитаем его из смеси
        current_mixture = subtract_reply(current_mixture, *dominant);
        
        if (current_mixture.empty()) break;
    }
    
    result.separated_replies = extracted;
    
    // Оценка качества
    if (extracted.size() == mixture.size()) {
        result.confidence = 0.9;
        result.ambiguous = false;
    } else if (extracted.size() > 0) {
        result.confidence = 0.7;
        result.ambiguous = (extracted.size() < mixture.size());
    }
    
    return result;
}

} // namespace radar