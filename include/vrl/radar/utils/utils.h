// include/vrl/radar/utils/utils.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include <array>
#include <vector>

namespace vrl {
namespace radar {
namespace utils {

// Позиция бита в эфирном представлении
size_t bit_position(int bit_idx);

// Проверка перекрытия двух ответов
bool is_potential_overlap(
    uint16_t az1, uint16_t r1,
    uint16_t az2, uint16_t r2,
    const RadarConfig& cfg
);

// Вычисление достоверности битов RBS
std::array<uint8_t, 12> compute_rbs_confidence(const RBSReply& reply);

// Базовая валидация
bool validate_rbs(const RBSReply& reply, const RadarConfig& cfg);
bool validate_uvd(const UVDReply& reply, const RadarConfig& cfg);

// SLS проверка
bool is_sidelobe(const RBSReply& reply, double threshold_db = 3.0);
bool is_sidelobe(const UVDReply& reply, double threshold_db = 3.0);

} // namespace utils
} // namespace radar
} // namespace vrl
