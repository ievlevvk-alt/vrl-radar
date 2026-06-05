// file: src/garbling_solver.cpp
#include "radar/garbling_solver.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <map>      // Добавлено для std::map
#include <vector>   // Добавлено для std::vector
#include <cstdint>  // Добавлено для uint*_t
#include <functional> // Добавлено для std::greater

namespace radar {

// ---- GarblingSolver base class ----

GarblingSolver::GarblingSolver(const RadarConfig& config) : config_(config) {}

void GarblingSolver::log(const std::string& msg) const {
    if (debug_) {
        std::cout << "[GarblingSolver] " << msg << std::endl;
    }
}

// ---- ThresholdGarblingSolver ----

ThresholdGarblingSolver::ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold)
    : GarblingSolver(config), threshold_(threshold) {}

std::vector<uint16_t> ThresholdGarblingSolver::detect_possible_codes_rbs(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture) {
    
    std::vector<uint16_t> possible_codes;
    std::map<uint16_t, int> code_votes;
    
    // Анализируем каждую позицию бита
    for (int i = 0; i < 12; ++i) {
        size_t pos = utils::bit_position(i);
        if (mixture[pos] > threshold_) {
            code_votes[1 << i] += 1;
        }
    }
    
    // Проверяем комбинации битов
    for (int i = 0; i < 12; i += 2) {
        uint8_t amp1 = mixture[utils::bit_position(i)];
        uint8_t amp2 = mixture[utils::bit_position(i+1)];
        
        if (amp1 > threshold_ && amp2 > threshold_) {
            code_votes[(1 << i) | (1 << (i+1))] += 2;
        }
    }
    
    // Сортируем по голосам
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
    
    // Суммируем амплитуды
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> total{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total.size(); ++i) {
            total[i] = std::min(255, static_cast<int>(total[i]) + reply.ether_amplitudes[i]);
        }
    }
    
    auto possible_codes = detect_possible_codes_rbs(total);
    
    // Если есть ожидаемые коды, фильтруем
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
        separated.spi = false;  // По умолчанию
        
        // Копируем амплитуды для битов этого кода
        for (size_t i = 0; i < RBSReply::ETHER_POSITIONS; ++i) {
            if (i == 0 || i == 14) {  // F1, F2
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
    
    // Оцениваем уверенность
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
    // Пока просто возвращаем первый ответ
    result.separated_replies.push_back(mixture[0]);
    result.confidence = 0.5;
    result.ambiguous = true;
    
    return result;
}

// ---- CorrelationGarblingSolver ----

CorrelationGarblingSolver::CorrelationGarblingSolver(const RadarConfig& config)
    : GarblingSolver(config) {}

void CorrelationGarblingSolver::load_code_library(const std::vector<uint16_t>& codes) {
    known_codes_ = codes;
}

void CorrelationGarblingSolver::load_data_library(const std::vector<uint32_t>& data) {
    known_data_ = data;
}

std::array<uint8_t, RBSReply::ETHER_POSITIONS> CorrelationGarblingSolver::create_template_rbs(
    uint16_t code, bool spi) {
    
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> tmpl{};
    
    // F1
    tmpl[0] = 255;
    
    // Биты данных
    for (int i = 0; i < 12; ++i) {
        if ((code >> i) & 1) {
            tmpl[utils::bit_position(i)] = 255;
        }
    }
    
    // F2
    tmpl[14] = 255;
    
    // SPI
    if (spi) {
        tmpl[17] = 255;
    }
    
    return tmpl;
}

std::array<uint8_t, UVDReply::ETHER_POSITIONS> CorrelationGarblingSolver::create_template_uvd(
    uint32_t data) {
    
    std::array<uint8_t, UVDReply::ETHER_POSITIONS> tmpl{};
    
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 20; ++i) {
            size_t offset = repeat * 40 + i * 2;
            bool bit = (data >> i) & 1;
            
            if (!bit) {
                tmpl[offset] = 255;      // left
                tmpl[offset + 1] = 0;    // right
            } else {
                tmpl[offset] = 0;         // left
                tmpl[offset + 1] = 255;   // right
            }
        }
    }
    
    return tmpl;
}

double CorrelationGarblingSolver::compute_correlation(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& signal,
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& pattern) {
    
    double sum_signal = 0, sum_pattern = 0, sum_product = 0;
    
    for (size_t i = 0; i < signal.size(); ++i) {
        sum_signal += signal[i] * signal[i];
        sum_pattern += pattern[i] * pattern[i];
        sum_product += signal[i] * pattern[i];
    }
    
    if (sum_signal == 0 || sum_pattern == 0) return 0;
    
    return sum_product / std::sqrt(sum_signal * sum_pattern);
}

double CorrelationGarblingSolver::compute_correlation(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& signal,
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& pattern) {
    
    double sum_signal = 0, sum_pattern = 0, sum_product = 0;
    
    for (size_t i = 0; i < signal.size(); ++i) {
        sum_signal += signal[i] * signal[i];
        sum_pattern += pattern[i] * pattern[i];
        sum_product += signal[i] * pattern[i];
    }
    
    if (sum_signal == 0 || sum_pattern == 0) return 0;
    
    return sum_product / std::sqrt(sum_signal * sum_pattern);
}

SeparationResult<RBSReply> CorrelationGarblingSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    SeparationResult<RBSReply> result;
    result.method_used = "Correlation";
    
    if (mixture.empty()) {
        result.confidence = 1.0;
        return result;
    }
    
    // Суммируем амплитуды
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> total{};
    for (const auto& reply : mixture) {
        for (size_t i = 0; i < total.size(); ++i) {
            total[i] = std::min(255, static_cast<int>(total[i]) + reply.ether_amplitudes[i]);
        }
    }
    
    // Используем библиотеку известных кодов или ожидаемые коды
    std::vector<uint16_t> codes_to_try;
    if (!expected_codes.empty()) {
        codes_to_try = expected_codes;
    } else if (!known_codes_.empty()) {
        codes_to_try = known_codes_;
    } else {
        // Если нет библиотеки, используем пороговый метод
        ThresholdGarblingSolver threshold_solver(config_, 50);
        auto threshold_result = threshold_solver.separate_rbs(mixture);
        result = threshold_result;
        result.method_used = "Correlation+Threshold";
        return result;
    }
    
    // Ищем наилучшее соответствие
    double best_correlation = 0;
    uint16_t best_code = 0;
    
    for (uint16_t code : codes_to_try) {
        auto tmpl = create_template_rbs(code, false);
        double corr = compute_correlation(total, tmpl);
        
        if (corr > best_correlation) {
            best_correlation = corr;
            best_code = code;
        }
    }
    
    if (best_correlation > 0.7) {
        RBSReply separated;
        separated.code12 = best_code;
        separated.azimuth = mixture[0].azimuth;
        separated.range = mixture[0].range;
        separated.ether_amplitudes = total;
        
        result.separated_replies.push_back(separated);
        result.confidence = best_correlation;
        result.ambiguous = false;
    }
    
    return result;
}

SeparationResult<UVDReply> CorrelationGarblingSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_codes) {
    
    SeparationResult<UVDReply> result;
    result.method_used = "Correlation";
    // TODO: Implement UVD correlation
    result.confidence = 0.5;
    return result;
}

// ---- IterativeSubtractionSolver ----

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

double IterativeSubtractionSolver::evaluate_separation_quality(
    const std::vector<RBSReply>& original,
    const std::vector<RBSReply>& separated) {
    
    if (original.empty() || separated.empty()) return 0.0;
    
    // Простая метрика: отношение количества разделенных к исходным
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
        
        // Декодируем код доминирующего ответа (упрощенно)
        // В реальности нужно использовать декодер
        uint16_t code = 0;
        for (int i = 0; i < 12; ++i) {
            if (dominant->ether_amplitudes[utils::bit_position(i)] > config_.min_amplitude * 3) {
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

// ---- CompositeGarblingSolver ----

CompositeGarblingSolver::CompositeGarblingSolver(const RadarConfig& config)
    : GarblingSolver(config) {}

void CompositeGarblingSolver::add_method(std::unique_ptr<GarblingSolver> method, int priority) {
    methods_.push_back({std::move(method), priority});
    
    // Сортируем по приоритету (высший first)
    std::sort(methods_.begin(), methods_.end(),
        [](const MethodWithPriority& a, const MethodWithPriority& b) {
            return a.priority > b.priority;
        });
}

template<typename ReplyType>
SeparationResult<ReplyType> CompositeGarblingSolver::select_best_result(
    std::vector<SeparationResult<ReplyType>>& results) {
    
    if (results.empty()) return SeparationResult<ReplyType>();
    
    auto best = std::max_element(results.begin(), results.end(),
        [](const auto& a, const auto& b) {
            return a.confidence < b.confidence;
        });
    
    return *best;
}

SeparationResult<RBSReply> CompositeGarblingSolver::separate_rbs(
    const std::vector<RBSReply>& mixture,
    const std::vector<uint16_t>& expected_codes) {
    
    std::vector<SeparationResult<RBSReply>> results;
    
    for (auto& m : methods_) {
        auto result = m.method->separate_rbs(mixture, expected_codes);
        results.push_back(result);
        
        if (result.confidence > 0.9) break;  // Достаточно хороший результат
    }
    
    return select_best_result(results);
}

SeparationResult<UVDReply> CompositeGarblingSolver::separate_uvd(
    const std::vector<UVDReply>& mixture,
    const std::vector<uint32_t>& expected_codes) {
    
    std::vector<SeparationResult<UVDReply>> results;
    
    for (auto& m : methods_) {
        auto result = m.method->separate_uvd(mixture, expected_codes);
        results.push_back(result);
    }
    
    return select_best_result(results);
}

// Явная инстанциация шаблонов
template std::optional<RBSReply> IterativeSubtractionSolver::find_dominant_reply<RBSReply>(
    const std::vector<RBSReply>&);
template std::vector<RBSReply> IterativeSubtractionSolver::subtract_reply<RBSReply>(
    const std::vector<RBSReply>&, const RBSReply&);

template SeparationResult<RBSReply> CompositeGarblingSolver::select_best_result<RBSReply>(
    std::vector<SeparationResult<RBSReply>>&);

} // namespace radar