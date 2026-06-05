#include "radar/simulator.h"
#include "radar/utils.h"
#include <cmath>
#include <algorithm>
#include <random>

namespace radar {

ReplySimulator::ReplySimulator(const SimulatorConfig& config)
    : config_(config)
    , rng_(std::random_device{}())
    , converter_(config.radar)
{}

void ReplySimulator::add_noise_to_amplitudes(
    std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps,
    double snr_db
) {
    if (snr_db <= 0) return;
    
    double signal_power = 128.0;
    double noise_power = signal_power / std::pow(10.0, snr_db / 10.0);
    double noise_std = std::sqrt(noise_power);
    
    std::normal_distribution<double> noise_dist(0.0, noise_std);
    
    for (auto& amp : amps) {
        if (amp > 0) {
            double noisy = amp + noise_dist(rng_);
            amp = static_cast<uint8_t>(std::clamp(noisy, 0.0, 255.0));
        }
    }
}

void ReplySimulator::add_noise_to_amplitudes(
    std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
    double snr_db
) {
    if (snr_db <= 0) return;
    
    double signal_power = 128.0;
    double noise_power = signal_power / std::pow(10.0, snr_db / 10.0);
    double noise_std = std::sqrt(noise_power);
    
    std::normal_distribution<double> noise_dist(0.0, noise_std);
    
    for (auto& amp : amps) {
        if (amp > 0) {
            double noisy = amp + noise_dist(rng_);
            amp = static_cast<uint8_t>(std::clamp(noisy, 0.0, 255.0));
        }
    }
}

uint16_t ReplySimulator::decode_rbs_from_ether(
    const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps
) {
    uint16_t code = 0;
    uint8_t threshold = 50;  // порог для определения "есть импульс"
    
    for (int i = 0; i < 12; ++i) {
        if (amps[2 + i] > threshold) {
            code |= (1 << i);
        }
    }
    return code;
}

uint32_t ReplySimulator::decode_uvd_from_ether(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps
) {
    uint32_t data = 0;
    uint8_t threshold = 50;
    
    // Используем первый повтор для декодирования
    for (int i = 0; i < 20; ++i) {
        uint8_t left = amps[i * 2];
        uint8_t right = amps[i * 2 + 1];
        
        // "Активная пауза": если левый > порога, а правый < порога - это 0
        // если левый < порога, а правый > порога - это 1
        bool left_high = left > threshold;
        bool right_high = right > threshold;
        
        if (left_high && !right_high) {
            // это 0 - ничего не делаем (бит уже 0)
        } else if (!left_high && right_high) {
            data |= (1 << i);
        }
        // иначе - неопределённое состояние, оставляем как есть
    }
    
    return data;
}

void ReplySimulator::generate_sls_channel_rbs(RBSReply& reply) {
    if (!config_.sls.enabled) {
        reply.ether_amplitudes_sls.fill(0);
        return;
    }
    
    // Определяем, находится ли ответ в главном или боковом лепестке
    std::bernoulli_distribution sidelobe_dist(config_.sls.sidelobe_probability);
    bool is_sidelobe = sidelobe_dist(rng_);
    
    for (size_t i = 0; i < RBSReply::ETHER_POSITIONS; ++i) {
        if (is_sidelobe) {
            // На боковом лепестке сигнал в SLS канале сильнее
            double attenuation = std::pow(10.0, config_.sls.sls_attenuation_db / 20.0);
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * attenuation, 0.0, 255.0)
            );
        } else {
            // На главном лепестке сигналы примерно равны
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * config_.sls.main_to_sls_ratio, 0.0, 255.0)
            );
        }
    }
}

void ReplySimulator::generate_sls_channel_uvd(UVDReply& reply) {
    if (!config_.sls.enabled) {
        reply.ether_amplitudes_sls.fill(0);
        return;
    }
    
    std::bernoulli_distribution sidelobe_dist(config_.sls.sidelobe_probability);
    bool is_sidelobe = sidelobe_dist(rng_);
    
    for (size_t i = 0; i < UVDReply::ETHER_POSITIONS; ++i) {
        if (is_sidelobe) {
            double attenuation = std::pow(10.0, config_.sls.sls_attenuation_db / 20.0);
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * attenuation, 0.0, 255.0)
            );
        } else {
            reply.ether_amplitudes_sls[i] = static_cast<uint8_t>(
                std::clamp(reply.ether_amplitudes[i] * config_.sls.main_to_sls_ratio, 0.0, 255.0)
            );
        }
    }
}

RBSReply ReplySimulator::generate_rbs(
    uint16_t azimuth,
    uint16_t range,
    uint16_t code12,
    bool spi
) {
    RBSReply reply;
    reply.azimuth = azimuth;
    reply.range = range;
    reply.code12 = code12 & 0x0FFF;
    reply.spi = spi;
    
    uint8_t base_amp = 200;
    
    // Заполняем эфирное представление в хронологическом порядке
    reply.ether_amplitudes.fill(0);
    
    // F1 (индекс 0)
    reply.ether_amplitudes[0] = base_amp;
    
    // Биты C1,A1,C2,A2,C4,A4 (индексы 1-6)
    // code12: бит0=C1, бит1=A1, бит2=C2, бит3=A2, бит4=C4, бит5=A4
    for (int i = 0; i < 6; ++i) {
        if ((code12 >> i) & 1) {
            reply.ether_amplitudes[1 + i] = base_amp;
        }
    }
    
    // X - центральная пауза (индекс 7) - всегда 0
    reply.ether_amplitudes[7] = 0;
    
    // Биты B1,D1,B2,D2,B4,D4 (индексы 8-13)
    // code12: бит6=B1, бит7=D1, бит8=B2, бит9=D2, бит10=B4, бит11=D4
    for (int i = 0; i < 6; ++i) {
        if ((code12 >> (6 + i)) & 1) {
            reply.ether_amplitudes[8 + i] = base_amp;
        }
    }
    
    // F2 (индекс 14)
    reply.ether_amplitudes[14] = base_amp;
    
    // SPARE1, SPARE2 (индексы 15-16) - всегда 0
    reply.ether_amplitudes[15] = 0;
    reply.ether_amplitudes[16] = 0;
    
    // SPI (индекс 17)
    if (spi) {
        reply.ether_amplitudes[17] = base_amp;
    }
    
    // Добавляем шум и вариации
    add_noise_to_amplitudes(reply.ether_amplitudes, config_.rbs.snr_db);
    
    if (config_.rbs.amp_variation > 0) {
        std::normal_distribution<double> amp_dist(1.0, config_.rbs.amp_variation);
        for (auto& amp : reply.ether_amplitudes) {
            if (amp > 0) {
                double factor = std::clamp(amp_dist(rng_), 0.5, 1.5);
                amp = static_cast<uint8_t>(std::clamp(amp * factor, 0.0, 255.0));
            }
        }
    }
    
    // Генерируем SLS канал
    generate_sls_channel_rbs(reply);
    
    // Вычисляем confidence через метод bit()
    reply.confidence = utils::compute_rbs_confidence(reply);
    reply.is_valid = utils::validate_rbs(reply, config_.radar);
    
    auto [x, y] = converter_.rbs_to_xy(azimuth, range);
    reply.x = x;
    reply.y = y;
    
    return reply;
}

UVDReply ReplySimulator::generate_uvd(
    uint16_t azimuth,
    uint16_t range,
    uint32_t data20
) {
    UVDReply reply;
    reply.azimuth = azimuth;
    reply.range = range;
    reply.data20 = data20 & 0x0FFFFF;
    
    uint8_t base_amp = 200;
    
    // Заполняем эфирное представление (80 отсчётов)
    reply.ether_amplitudes.fill(0);
    
    // Два повтора
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 20; ++i) {
            bool bit = (data20 >> i) & 1;
            size_t offset = repeat * 40 + i * 2;
            
            // "Активная пауза": для 0: left=1, right=0; для 1: left=0, right=1
            if (!bit) {  // 0
                reply.ether_amplitudes[offset] = base_amp;      // left
                reply.ether_amplitudes[offset + 1] = 0;         // right
            } else {     // 1
                reply.ether_amplitudes[offset] = 0;             // left
                reply.ether_amplitudes[offset + 1] = base_amp;  // right
            }
        }
    }
    
    // Добавляем шум
    add_noise_to_amplitudes(reply.ether_amplitudes, config_.uvd.snr_db);
    
    // Генерируем SLS канал
    generate_sls_channel_uvd(reply);
    
    // Вычисляем error_mask
    reply.error_mask = compute_uvd_error_mask(reply.ether_amplitudes, data20);
    
    // Добавляем случайные ошибки
    std::bernoulli_distribution error_dist(config_.uvd.error_probability);
    for (int i = 0; i < 20; ++i) {
        if (error_dist(rng_)) {
            reply.error_mask |= (1 << i);
        }
    }
    
    // Валидация
    reply.is_valid = utils::validate_uvd(reply, config_.radar);
    
    // Координаты
    auto [x, y] = converter_.uvd_to_xy(azimuth, range);
    reply.x = x;
    reply.y = y;
    
    return reply;
}

uint32_t ReplySimulator::compute_uvd_error_mask(
    const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
    uint32_t original_data20
) {
    uint32_t mask = 0;
    uint8_t threshold = 50;
    
    // Сравниваем два повтора
    for (int i = 0; i < 20; ++i) {
        // Первый повтор
        uint8_t left1 = amps[i * 2];
        uint8_t right1 = amps[i * 2 + 1];
        
        // Второй повтор
        uint8_t left2 = amps[40 + i * 2];
        uint8_t right2 = amps[40 + i * 2 + 1];
        
        // Декодируем каждый повтор
        bool repeat1_one = (left1 <= threshold && right1 > threshold);
        bool repeat2_one = (left2 <= threshold && right2 > threshold);
        
        bool repeat1_zero = (left1 > threshold && right1 <= threshold);
        bool repeat2_zero = (left2 > threshold && right2 <= threshold);
        
        // Если повторы дают разные результаты - бит сбойный
        if (repeat1_one != repeat2_one || repeat1_zero != repeat2_zero) {
            mask |= (1 << i);
        }
        
        // Если оба повтора не дают однозначного результата - тоже сбойный
        if ((!repeat1_one && !repeat1_zero) || (!repeat2_one && !repeat2_zero)) {
            mask |= (1 << i);
        }
    }
    
    return mask;
}

ReplySimulator::OverlapResult ReplySimulator::mix_two_rbs(
    const RBSReply& r1,
    const RBSReply& r2,
    int16_t range_offset,
    double amp_ratio
) {
    OverlapResult result;
    
    // В реальности здесь нужно суммировать амплитуды со сдвигом
    // Пока просто возвращаем оба ответа
    result.rbs_mixture.push_back(r1);
    result.rbs_mixture.push_back(r2);
    
    return result;
}

ReplySimulator::OverlapResult ReplySimulator::mix_two_uvd(
    const UVDReply& u1,
    const UVDReply& u2,
    int16_t range_offset,
    double amp_ratio
) {
    OverlapResult result;
    result.uvd_mixture.push_back(u1);
    result.uvd_mixture.push_back(u2);
    return result;
}

} // namespace radar