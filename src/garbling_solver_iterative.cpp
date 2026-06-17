// file: src/garbling_solver_iterative.cpp
#include "radar/garbling_solver.h"
#include "radar/utils.h"  // Добавляем
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
    
    std::vector<RBSReply> current = mixture;
    std::vector<RBSReply> extracted;
    
    for (int iter = 0; iter < max_iterations_ && !current.empty(); ++iter) {
        auto dominant = find_dominant_reply(current);
        if (!dominant) break;
        
        // Декодируем код доминирующего ответа
        uint16_t code = 0;
        for (int i = 0; i < 12; ++i) {
            size_t pos = utils::bit_position(i);  // Используем utils::
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


} // namespace radar