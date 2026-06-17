// src/garbling_solver_threshold.cpp
#include "radar/garbling_solver.h"
#include "radar/utils.h"  // Добавляем для utils::bit_position
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace radar {

ThresholdGarblingSolver::ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold)
    : GarblingSolver(config), threshold_(threshold) {}

std::vector<uint16_t> ThresholdGarblingSolver::detect_possible_codes_rbs(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint16_t> possible_codes;
    
    // Анализируем каждую позицию бита
    std::array<bool, 12> bit_present;
    std::array<uint8_t, 12> bit_amplitudes;
    
    for (int i = 0; i < 12; ++i) {
        size_t pos = utils::bit_position(i);  // Используем utils::bit_position
        uint8_t amp = mixture[pos];
        bit_present[i] = (amp > threshold_);
        bit_amplitudes[i] = amp;
    }
    
    // Проверяем наличие кадрирующих импульсов
    bool has_f1 = mixture[0] > threshold_;
    bool has_f2 = mixture[14] > threshold_;
    
    if (!has_f1 || !has_f2) {
        // Нет кадрирующих - возможно сильное перекрытие
        log("Warning: Missing framing pulses in mixture");
    }
    
    // Анализ амплитуд для выявления нескольких кодов
    std::map<uint16_t, int> code_votes;
    
    // Пытаемся собрать код из битов с высокой амплитудой
    uint16_t candidate_code = 0;
    for (int i = 0; i < 12; ++i) {
        if (bit_amplitudes[i] > threshold_ * 1.5) {  // очень сильный бит
            candidate_code |= (1 << i);
        }
    }
    if (candidate_code != 0) {
        code_votes[candidate_code] += 3;  // высокий вес
    }
    
    // Анализ пар битов
    for (int i = 0; i < 12; i += 2) {
        uint8_t amp1 = bit_amplitudes[i];
        uint8_t amp2 = bit_amplitudes[i+1];
        
        if (amp1 > threshold_ && amp2 <= threshold_) {
            // Только первый бит - возможен код с 0 во втором
            uint16_t code = (1 << i);
            code_votes[code] += 1;
        } else if (amp1 <= threshold_ && amp2 > threshold_) {
            // Только второй бит - возможен код с 0 в первом
            uint16_t code = (1 << (i+1));
            code_votes[code] += 1;
        } else if (amp1 > threshold_ && amp2 > threshold_) {
            // Оба бита - возможны два кода
            code_votes[(1 << i)] += 1;
            code_votes[(1 << (i+1))] += 1;
            // Или один код с обоими битами
            code_votes[(1 << i) | (1 << (i+1))] += 2;
        }
    }
    
    // Выбираем коды с наибольшим количеством голосов
    std::vector<std::pair<int, uint16_t>> sorted;
    for (const auto& [code, votes] : code_votes) {
        sorted.emplace_back(votes, code);
    }
    std::sort(sorted.begin(), sorted.end(), std::greater<>());
    
    for (const auto& [votes, code] : sorted) {
        if (votes >= 2) {  // минимум 2 голоса
            possible_codes.push_back(code);
        }
    }
    
    return possible_codes;
}

bool ThresholdGarblingSolver::check_code_presence_rbs(
    uint16_t code,
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    int matching_bits = 0;
    int total_bits = 0;
    
    for (int i = 0; i < 12; ++i) {
        bool bit_expected = (code >> i) & 1;
        size_t pos = utils::bit_position(i);  // Используем utils::bit_position
        uint8_t amplitude = mixture[pos];
        
        if (amplitude > threshold_) {
            if (bit_expected) matching_bits++;
            total_bits++;
        }
    }
    
    // Если бит ожидается, но амплитуда низкая - это может быть подавление
    for (int i = 0; i < 12; ++i) {
        bool bit_expected = (code >> i) & 1;
        size_t pos = utils::bit_position(i);
        uint8_t amplitude = mixture[pos];
        
        if (bit_expected && amplitude <= threshold_) {
            // Ожидаемый бит отсутствует - возможно подавлен перекрытием
            // Проверяем соседние позиции
            if (i > 0) {
                size_t prev_pos = utils::bit_position(i-1);
                if (mixture[prev_pos] > threshold_ * 2) {
                    // Возможно смещение из-за перекрытия
                    total_bits++;
                }
            }
        }
    }
    
    return (total_bits > 0) && (static_cast<double>(matching_bits) / total_bits >= 0.7);
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
    
    // Если только один ответ - просто возвращаем его
    if (mixture.size() == 1) {
        result.separated_replies = mixture;
        result.confidence = 1.0;
        return result;
    }
    
    // Суммируем эфирные амплитуды
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> total_amplitudes{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total_amplitudes.size(); ++i) {
            total_amplitudes[i] = std::min(255, 
                total_amplitudes[i] + reply.ether_amplitudes[i]);
        }
    }
    
    // Детектируем возможные коды
    auto possible_codes = detect_possible_codes_rbs(total_amplitudes);
    
    // Если есть ожидаемые коды, используем их как приоритетные
    if (!expected_codes.empty()) {
        std::vector<uint16_t> filtered;
        for (uint16_t code : expected_codes) {
            if (check_code_presence_rbs(code, total_amplitudes)) {
                filtered.push_back(code);
            }
        }
        if (!filtered.empty()) {
            possible_codes = filtered;
        }
    }
    
    // Создаем разделенные ответы
    for (uint16_t code : possible_codes) {
        RBSReply separated;
        separated.code12 = code;
        separated.azimuth = mixture[0].azimuth;  // Используем первый азимут
        separated.range = mixture[0].range;
        
        // Восстанавливаем эфирные амплитуды для этого кода
        // (упрощенно - берем максимум из смеси)
        for (size_t i = 0; i < RBSReply::ETHER_POSITIONS; ++i) {
            if (i == 0 || i == 14) {  // F1, F2 всегда есть
                separated.ether_amplitudes[i] = total_amplitudes[i];
            } else {
                int bit_idx = -1;
                for (int b = 0; b < 12; ++b) {
                    if (utils::bit_position(b) == i) {  // Используем utils::bit_position
                        bit_idx = b;
                        break;
                    }
                }
                if (bit_idx >= 0 && ((code >> bit_idx) & 1)) {
                    separated.ether_amplitudes[i] = total_amplitudes[i];
                }
            }
        }
        
        result.separated_replies.push_back(separated);
    }
    
    // Оцениваем уверенность
    if (mixture.size() == result.separated_replies.size()) {
        result.confidence = 0.8;
        result.ambiguous = false;
    } else if (result.separated_replies.size() < mixture.size()) {
        result.confidence = 0.6;
        result.ambiguous = true;
    } else {
        result.confidence = 0.4;
        result.ambiguous = true;
    }
    
    return result;
}

// Реализация для UVD (заглушка, можно расширить позже)
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
    // Пока просто возвращаем первый ответ
    result.separated_replies.push_back(mixture[0]);
    result.confidence = 0.5;
    result.ambiguous = true;
    
    return result;
}

} // namespace radar
