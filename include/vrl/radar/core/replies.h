// include/vrl/radar/core/replies.h
#pragma once

#include "types.h"
#include <array>
#include <vector>
#include <cstdint>
#include <string>
#include <cstring>
#include <variant>   // <-- ДОБАВЛЯЕМ
#include <memory>    // <-- ДОБАВЛЯЕМ

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
// TARGET REPORT - ИСПРАВЛЕННАЯ ВЕРСИЯ
// ============================================================================

// Типобезопасный указатель на источник ответа
using ReplySource = std::variant<const RBSReply*, const UVDReply*>;

struct TargetReport {
    enum class SourceType { RBS, UVD } type;
    
    double x{0.0}, y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    // Используем отдельные структуры вместо union
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
    
    // Храним оба набора данных, используем флаг type для выбора
    RBSData rbs;
    UVDData uvd;
    
    uint8_t signal_strength{0};
    bool is_reflection{false};
    bool is_sls_blanked{false};
    bool is_garbled{false};
    
    // ИСПРАВЛЕНО: используем variant вместо vector<const void*>
    std::vector<ReplySource> sources;
    
    // Конструктор по умолчанию
    TargetReport() : type(SourceType::RBS) {
        // Инициализация всех полей
        x = 0.0;
        y = 0.0;
        azimuth_deg = 0.0;
        range_m = 0.0;
        signal_strength = 0;
        is_reflection = false;
        is_sls_blanked = false;
        is_garbled = false;
        // rbs и uvd инициализируются автоматически
    }
    
    // Конструктор для RBS
    static TargetReport make_rbs() {
        TargetReport report;
        report.type = SourceType::RBS;
        return report;
    }
    
    // Конструктор для UVD
    static TargetReport make_uvd() {
        TargetReport report;
        report.type = SourceType::UVD;
        return report;
    }
    
    // Добавить источник ответа (типобезопасно)
    void add_source(const RBSReply* source) {
        sources.emplace_back(source);
    }
    
    void add_source(const UVDReply* source) {
        sources.emplace_back(source);
    }
    
    // Получить все RBS источники
    std::vector<const RBSReply*> get_rbs_sources() const {
        std::vector<const RBSReply*> result;
        for (const auto& s : sources) {
            if (const auto* rbs = std::get_if<const RBSReply*>(&s)) {
                result.push_back(*rbs);
            }
        }
        return result;
    }
    
    // Получить все UVD источники
    std::vector<const UVDReply*> get_uvd_sources() const {
        std::vector<const UVDReply*> result;
        for (const auto& s : sources) {
            if (const auto* uvd = std::get_if<const UVDReply*>(&s)) {
                result.push_back(*uvd);
            }
        }
        return result;
    }
    
    // Проверить, есть ли источники
    bool has_sources() const {
        return !sources.empty();
    }
    
    // Очистить источники
    void clear_sources() {
        sources.clear();
    }
};

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ РАБОТЫ С ИСТОЧНИКАМИ
// ============================================================================

/**
 * @brief Обойти все источники в отчете с помощью лямбды
 * 
 * Пример использования:
 * visit_sources(report.sources, [](const auto* source) {
 *     if constexpr (std::is_same_v<decltype(source), const RBSReply*>) {
 *         // Работа с RBS
 *     } else if constexpr (std::is_same_v<decltype(source), const UVDReply*>) {
 *         // Работа с UVD
 *     }
 * });
 */
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

/**
 * @brief Обойти все источники в отчете с помощью лямбды (перегрузка для одного источника)
 */
template<typename Visitor>
void visit_source(const ReplySource& source, Visitor&& visitor) {
    std::visit([&visitor](const auto* ptr) {
        if constexpr (std::is_same_v<decltype(ptr), const RBSReply*>) {
            visitor(ptr);
        } else if constexpr (std::is_same_v<decltype(ptr), const UVDReply*>) {
            visitor(ptr);
        }
    }, source);
}

} // namespace radar
} // namespace vrl
