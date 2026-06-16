// file: include/radar/reply_processor.h
#pragma once

#include "replies.h"
#include "types.h"
#include <optional>
#include <vector>

namespace radar {

// Дополнительные характеристики ответа
struct ReplyFeatures {
    double snr_estimate{0.0};      // оценка SNR в дБ
    double pulse_stability{0.0};   // стабильность импульсов
    bool has_framing{false};       // наличие кадрирующих
    bool has_spi{false};           // наличие SPI
    int bit_errors{0};             // оценка количества битовых ошибок
    double confidence{0.0};        // общая уверенность
};

// Обработчик единичных ответов
class ReplyProcessor {
public:
    explicit ReplyProcessor(const RadarConfig& config);
    
    // Анализ RBS ответа
    ReplyFeatures analyze_rbs(const RBSReply& reply);
    
    // Анализ УВД ответа
    ReplyFeatures analyze_uvd(const UVDReply& reply);
    
    // Коррекция амплитуд
    void normalize_amplitudes(RBSReply& reply);
    void normalize_amplitudes(UVDReply& reply);
    
    // Декодирование с исправлением ошибок
    uint16_t decode_rbs_with_errors(const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps);
    uint32_t decode_uvd_with_errors(const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps);
    
    // Оценка качества
    double estimate_snr(const RBSReply& reply);
    double estimate_snr(const UVDReply& reply);
    
private:
    RadarConfig config_;
    
    double calculate_pulse_ratio(const RBSReply& reply);
    double calculate_pulse_ratio(const UVDReply& reply);
    int count_bit_errors(const RBSReply& reply);
};

} // namespace radar
