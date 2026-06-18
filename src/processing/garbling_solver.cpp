// src/processing/garbling_solver.cpp
#include "vrl/radar/processing/garbling_solver.h"
#include "vrl/radar/utils/utils.h"
#include "vrl/radar/utils/logger.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <map>
#include <vector>
#include <cstdint>
#include <functional>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {

// ============================================================================
// GARBLING SOLVER BASE CLASS
// ============================================================================

GarblingSolver::GarblingSolver(const RadarConfig& config) : config_(config) {}

void GarblingSolver::log(const std::string& msg) const {
    if (debug_) {
        VRL_LOG_DEBUG(modules::GARBLING, msg);
    }
}

// ============================================================================
// THRESHOLD GARBLING SOLVER - RBS
// ============================================================================

ThresholdGarblingSolver::ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold)
    : GarblingSolver(config), threshold_(threshold) {}

std::vector<uint16_t> ThresholdGarblingSolver::detect_possible_codes_rbs(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint16_t> possible_codes;
    std::map<uint16_t, int> code_votes;
    
    // Анализ отдельных бит
    for (int i = 0; i < 12; ++i) {
        size_t pos = bit_position(i);
        if (mixture[pos] > threshold_) {
            code_votes[1 << i] += 1;
        }
    }
    
    // Анализ пар бит (для обнаружения комбинаций)
    for (int i = 0; i < 12; i += 2) {
        size_t pos1 = bit_position(i);
        size_t pos2 = bit_position(i+1);
        uint8_t amp1 = mixture[pos1];
        uint8_t amp2 = mixture[pos2];
        
        if (amp1 > threshold_ && amp2 > threshold_) {
            code_votes[(1 << i) | (1 << (i+1))] += 2;
        }
    }
    
    // Сортировка по количеству голосов
    std::vector<std::pair<int, uint16_t>> sorted;
    for (const auto& kv : code_votes) {
        sorted.emplace_back(kv.second, kv.first);
    }
    std::sort(sorted.begin(), sorted.end(), std::greater<std::pair<int, uint16_t>>());
    
    // Отбираем коды с достаточным количеством голосов
    for (const auto& item : sorted) {
        if (item.first >= 2) {
            possible_codes.push_back(item.second);
        }
    }
    
    log("Detected " + std::to_string(possible_codes.size()) + " possible RBS codes");
    return possible_codes;
}

bool ThresholdGarblingSolver::check_code_presence_rbs(
    uint16_t code,
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    int matching = 0;
    int total = 0;
    
    for (int i = 0; i < 12; ++i) {
        bool expected = (code >> i) & 1;
        if (mixture[bit_position(i)] > threshold_) {
            if (expected) matching++;
            total++;
        }
    }
    
    bool present = total > 0 && (static_cast<double>(matching) / total >= 0.6);
    if (present) {
        log("Code 0" + std::to_string(code) + " present in mixture");
    }
    return present;
}

// ============================================================================
// THRESHOLD GARBLING SOLVER - UVD
// ============================================================================

std::vector<uint32_t> ThresholdGarblingSolver::detect_possible_data_uvd(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint32_t> possible_data;
    std::map<uint32_t, int> data_votes;
    
    // Анализируем каждый бит данных
    for (int bit = 0; bit < 20; ++bit) {
        uint8_t left1 = mixture[bit * 2];
        uint8_t right1 = mixture[bit * 2 + 1];
        uint8_t left2 = mixture[40 + bit * 2];
        uint8_t right2 = mixture[40 + bit * 2 + 1];
        
        // Определяем значение бита по первому повторению
        bool bit1_zero = (left1 > threshold_ && right1 <= threshold_);
        bool bit1_one = (left1 <= threshold_ && right1 > threshold_);
        
        // Определяем значение бита по второму повторению
        bool bit2_zero = (left2 > threshold_ && right2 <= threshold_);
        bool bit2_one = (left2 <= threshold_ && right2 > threshold_);
        
        // Если оба повторения согласованы
        if ((bit1_zero && bit2_zero) || (bit1_one && bit2_one)) {
            // Бит определен однозначно
        } else if (bit1_zero || bit2_zero) {
            // Бит скорее всего 0
            data_votes[0] += 1;
        } else if (bit1_one || bit2_one) {
            // Бит скорее всего 1
            data_votes[1 << bit] += 1;
        } else {
            // Неопределенный бит - пропускаем
            data_votes[0] += 0;
        }
    }
    
    // Формируем возможные данные
    uint32_t candidate_data = 0;
    for (int bit = 0; bit < 20; ++bit) {
        // Простое голосование - большинство повторений
        uint8_t left1 = mixture[bit * 2];
        uint8_t right1 = mixture[bit * 2 + 1];
        uint8_t left2 = mixture[40 + bit * 2];
        uint8_t right2 = mixture[40 + bit * 2 + 1];
        
        bool bit1 = (left1 > threshold_ && right1 <= threshold_) ? false : 
                   (left1 <= threshold_ && right1 > threshold_) ? true : false;
        bool bit2 = (left2 > threshold_ && right2 <= threshold_) ? false :
                   (left2 <= threshold_ && right2 > threshold_) ? true : false;
        
        if (bit1 && bit2) {
            candidate_data |= (1 << bit);
        } else if (bit1 || bit2) {
            // Только одно повторение указывает на 1
            // Добавляем с меньшим весом
        }
    }
    
    if (candidate_data > 0) {
        possible_data.push_back(candidate_data);
        log("Detected UVD data: 0x" + std::to_string(candidate_data));
    }
    
    // Дополнительно проверяем возможные альтернативы
    for (int bit = 0; bit < 20; ++bit) {
        if (data_votes[1 << bit] > 0) {
            uint32_t alt_data = candidate_data | (1 << bit);
            if (alt_data != candidate_data) {
                possible_data.push_back(alt_data);
            }
        }
    }
    
    return possible_data;
}

// ============================================================================
// THRESHOLD GARBLING SOLVER - SEPARATION
// ============================================================================

SeparationResult<RBSReply> ThresholdGarblingSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    SeparationResult<RBSReply> result;
    result.method_used = "Threshold";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    log("Separating " + std::to_string(mixture.size()) + " RBS replies");
    
    if (mixture.size() == 1) {
        result.separated_replies = mixture;
        result.confidence = 1.0;
        log("Single reply, no separation needed");
        return result;
    }
    
    // Суммируем амплитуды
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> total{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total.size(); ++i) {
            total[i] = std::min(255, static_cast<int>(total[i]) + reply.ether_amplitudes[i]);
        }
    }
    
    // Обнаруживаем возможные коды
    auto possible_codes = detect_possible_codes_rbs(total);
    
    // Фильтруем по ожидаемым кодам
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
    
    log("Found " + std::to_string(possible_codes.size()) + " possible codes");
    
    // Восстанавливаем каждый код
    for (uint16_t code : possible_codes) {
        RBSReply separated;
        separated.code12 = code;
        separated.azimuth = mixture[0].azimuth;
        separated.range = mixture[0].range;
        separated.spi = false;
        
        // Копируем фреймовые импульсы
        separated.ether_amplitudes[0] = total[0];  // F1
        separated.ether_amplitudes[14] = total[14]; // F2
        
        // Восстанавливаем биты данных
        for (int i = 0; i < 12; ++i) {
            size_t pos = bit_position(i);
            if ((code >> i) & 1) {
                separated.ether_amplitudes[pos] = total[pos];
            } else {
                separated.ether_amplitudes[pos] = 0;
            }
        }
        
        // Проверяем SPI
        if (total[17] > threshold_) {
            separated.spi = true;
            separated.ether_amplitudes[17] = total[17];
        }
        
        result.separated_replies.push_back(separated);
    }
    
    // Оценка уверенности
    if (result.separated_replies.size() == mixture.size()) {
        result.confidence = 0.9;
        result.ambiguous = false;
        log("Perfect separation: " + std::to_string(mixture.size()) + " replies");
    } else if (result.separated_replies.size() > 0) {
        result.confidence = 0.7;
        result.ambiguous = true;
        log("Partial separation: " + std::to_string(result.separated_replies.size()) + 
            " of " + std::to_string(mixture.size()));
    } else {
        result.confidence = 0.3;
        result.ambiguous = true;
        log("Failed to separate replies");
    }
    
    return result;
}

SeparationResult<UVDReply> ThresholdGarblingSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_data) {
    
    SeparationResult<UVDReply> result;
    result.method_used = "Threshold";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    log("Separating " + std::to_string(mixture.size()) + " UVD replies");
    
    if (mixture.size() == 1) {
        result.separated_replies = mixture;
        result.confidence = 1.0;
        log("Single UVD reply, no separation needed");
        return result;
    }
    
    // Суммируем амплитуды
    std::array<uint8_t, UVDReply::ETHER_POSITIONS> total{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total.size(); ++i) {
            total[i] = std::min(255, static_cast<int>(total[i]) + reply.ether_amplitudes[i]);
        }
    }
    
    // Обнаруживаем возможные данные
    auto possible_data = detect_possible_data_uvd(total);
    
    // Фильтруем по ожидаемым данным
    if (!expected_data.empty()) {
        std::vector<uint32_t> filtered;
        for (uint32_t data : expected_data) {
            if (std::find(possible_data.begin(), possible_data.end(), data) != possible_data.end()) {
                filtered.push_back(data);
            }
        }
        if (!filtered.empty()) {
            possible_data = filtered;
        }
    }
    
    log("Found " + std::to_string(possible_data.size()) + " possible UVD data");
    
    // Восстанавливаем каждые данные
    for (uint32_t data : possible_data) {
        UVDReply separated;
        separated.data20 = data;
        separated.azimuth = mixture[0].azimuth;
        separated.range = mixture[0].range;
        separated.is_valid = true;
        separated.error_mask = 0;
        
        // Восстанавливаем биты
        for (int bit = 0; bit < 20; ++bit) {
            bool bit_value = (data >> bit) & 1;
            size_t offset = bit * 2;
            
            if (!bit_value) {
                // Бит = 0: левый импульс > правого
                separated.ether_amplitudes[offset] = total[offset];
                separated.ether_amplitudes[offset + 1] = 0;
                separated.ether_amplitudes[40 + offset] = total[40 + offset];
                separated.ether_amplitudes[40 + offset + 1] = 0;
            } else {
                // Бит = 1: правый импульс > левого
                separated.ether_amplitudes[offset] = 0;
                separated.ether_amplitudes[offset + 1] = total[offset + 1];
                separated.ether_amplitudes[40 + offset] = 0;
                separated.ether_amplitudes[40 + offset + 1] = total[40 + offset + 1];
            }
        }
        
        // Проверяем согласованность повторений
        uint32_t error_mask = 0;
        for (int bit = 0; bit < 20; ++bit) {
            bool bit_value = (data >> bit) & 1;
            uint8_t left1 = separated.ether_amplitudes[bit * 2];
            uint8_t right1 = separated.ether_amplitudes[bit * 2 + 1];
            uint8_t left2 = separated.ether_amplitudes[40 + bit * 2];
            uint8_t right2 = separated.ether_amplitudes[40 + bit * 2 + 1];
            
            bool decoded1 = (left1 > threshold_ && right1 <= threshold_) ? false :
                           (left1 <= threshold_ && right1 > threshold_) ? true : false;
            bool decoded2 = (left2 > threshold_ && right2 <= threshold_) ? false :
                           (left2 <= threshold_ && right2 > threshold_) ? true : false;
            
            if (decoded1 != decoded2) {
                error_mask |= (1 << bit);
            }
        }
        
        separated.error_mask = error_mask;
        if (error_mask != 0) {
            log("UVD data 0x" + std::to_string(data) + " has errors: 0x" + std::to_string(error_mask));
        }
        
        result.separated_replies.push_back(separated);
    }
    
    // Оценка уверенности
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

// ============================================================================
// ITERATIVE SUBTRACTION SOLVER - RBS
// ============================================================================

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
        bool has_content = false;
        
        for (size_t i = 0; i < remaining.ether_amplitudes.size(); ++i) {
            if (remaining.ether_amplitudes[i] > to_subtract.ether_amplitudes[i]) {
                remaining.ether_amplitudes[i] -= to_subtract.ether_amplitudes[i];
            } else {
                remaining.ether_amplitudes[i] = 0;
            }
            
            if (remaining.ether_amplitudes[i] > config_.min_amplitude * 2) {
                has_content = true;
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
    
    // Качество определяется как отношение количества разделенных к исходным
    double quality = static_cast<double>(separated.size()) / original.size();
    
    // Дополнительный бонус за успешное разделение всех
    if (separated.size() == original.size()) {
        quality = std::min(1.0, quality * 1.1);
    }
    
    return std::min(1.0, quality);
}

SeparationResult<RBSReply> IterativeSubtractionSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    (void)expected_codes;
    
    SeparationResult<RBSReply> result;
    result.method_used = "IterativeSubtraction";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    log("Iterative separation of " + std::to_string(mixture.size()) + " RBS replies");
    
    std::vector<RBSReply> current = mixture;
    std::vector<RBSReply> extracted;
    
    for (int iter = 0; iter < max_iterations_ && !current.empty(); ++iter) {
        auto dominant = find_dominant_reply(current);
        if (!dominant) break;
        
        // Декодируем код из доминирующего ответа
        uint16_t code = 0;
        for (int i = 0; i < 12; ++i) {
            size_t pos = bit_position(i);
            if (dominant->ether_amplitudes[pos] > config_.min_amplitude * 3) {
                code |= (1 << i);
            }
        }
        dominant->code12 = code;
        
        // Определяем SPI
        if (dominant->ether_amplitudes[17] > config_.min_amplitude * 3) {
            dominant->spi = true;
        }
        
        extracted.push_back(*dominant);
        current = subtract_reply(current, *dominant);
        
        log("Iteration " + std::to_string(iter+1) + ": extracted code 0" + 
            std::to_string(code) + ", remaining " + std::to_string(current.size()) + " replies");
    }
    
    result.separated_replies = extracted;
    result.confidence = evaluate_separation_quality(mixture, extracted);
    result.ambiguous = (extracted.size() < mixture.size());
    
    log("Separated " + std::to_string(extracted.size()) + " of " + 
        std::to_string(mixture.size()) + " replies, confidence " + 
        std::to_string(result.confidence));
    
    return result;
}

// ============================================================================
// ITERATIVE SUBTRACTION SOLVER - UVD
// ============================================================================

SeparationResult<UVDReply> IterativeSubtractionSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_codes) {
    
    (void)expected_codes;
    
    SeparationResult<UVDReply> result;
    result.method_used = "IterativeSubtraction";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    log("Iterative separation of " + std::to_string(mixture.size()) + " UVD replies");
    
    std::vector<UVDReply> current = mixture;
    std::vector<UVDReply> extracted;
    
    for (int iter = 0; iter < max_iterations_ && !current.empty(); ++iter) {
        auto dominant = find_dominant_reply(current);
        if (!dominant) break;
        
        // Декодируем данные из доминирующего ответа
        uint32_t data = 0;
        for (int bit = 0; bit < 20; ++bit) {
            uint8_t left1 = dominant->ether_amplitudes[bit * 2];
            uint8_t right1 = dominant->ether_amplitudes[bit * 2 + 1];
            uint8_t left2 = dominant->ether_amplitudes[40 + bit * 2];
            uint8_t right2 = dominant->ether_amplitudes[40 + bit * 2 + 1];
            
            bool bit1 = (left1 > config_.min_amplitude * 3 && 
                        right1 <= config_.min_amplitude * 3) ? false :
                       (left1 <= config_.min_amplitude * 3 && 
                        right1 > config_.min_amplitude * 3) ? true : false;
            bool bit2 = (left2 > config_.min_amplitude * 3 && 
                        right2 <= config_.min_amplitude * 3) ? false :
                       (left2 <= config_.min_amplitude * 3 && 
                        right2 > config_.min_amplitude * 3) ? true : false;
            
            // Голосование - если оба повторения согласованы
            if (bit1 && bit2) {
                data |= (1 << bit);
            } else if (bit1 || bit2) {
                // Только одно повторение - берем его
                if (bit1) data |= (1 << bit);
                else if (bit2) data |= (1 << bit);
            }
        }
        dominant->data20 = data;
        
        // Вычисляем ошибки
        uint32_t error_mask = 0;
        for (int bit = 0; bit < 20; ++bit) {
            bool bit_value = (data >> bit) & 1;
            uint8_t left1 = dominant->ether_amplitudes[bit * 2];
            uint8_t right1 = dominant->ether_amplitudes[bit * 2 + 1];
            uint8_t left2 = dominant->ether_amplitudes[40 + bit * 2];
            uint8_t right2 = dominant->ether_amplitudes[40 + bit * 2 + 1];
            
            bool decoded1 = (left1 > threshold_for_uvd_ && right1 <= threshold_for_uvd_) ? false :
                           (left1 <= threshold_for_uvd_ && right1 > threshold_for_uvd_) ? true : false;
            bool decoded2 = (left2 > threshold_for_uvd_ && right2 <= threshold_for_uvd_) ? false :
                           (left2 <= threshold_for_uvd_ && right2 > threshold_for_uvd_) ? true : false;
            
            if (decoded1 != decoded2) {
                error_mask |= (1 << bit);
            }
        }
        dominant->error_mask = error_mask;
        
        extracted.push_back(*dominant);
        current = subtract_reply(current, *dominant);
        
        log("Iteration " + std::to_string(iter+1) + ": extracted data 0x" + 
            std::to_string(data) + ", remaining " + std::to_string(current.size()) + " replies");
    }
    
    result.separated_replies = extracted;
    result.confidence = std::min(1.0, static_cast<double>(extracted.size()) / mixture.size());
    result.ambiguous = (extracted.size() < mixture.size());
    
    log("Separated " + std::to_string(extracted.size()) + " of " + 
        std::to_string(mixture.size()) + " UVD replies");
    
    return result;
}

// ============================================================================
// ЯВНАЯ ИНСТАНЦИАЦИЯ ШАБЛОНОВ
// ============================================================================

template std::optional<RBSReply> IterativeSubtractionSolver::find_dominant_reply<RBSReply>(
    const std::vector<RBSReply>&);

template std::optional<UVDReply> IterativeSubtractionSolver::find_dominant_reply<UVDReply>(
    const std::vector<UVDReply>&);

template std::vector<RBSReply> IterativeSubtractionSolver::subtract_reply<RBSReply>(
    const std::vector<RBSReply>&, const RBSReply&);

template std::vector<UVDReply> IterativeSubtractionSolver::subtract_reply<UVDReply>(
    const std::vector<UVDReply>&, const UVDReply&);

} // namespace radar
} // namespace vrl
