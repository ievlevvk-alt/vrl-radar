#pragma once

#include "types.h"
#include <cstdint>
#include <array>
#include <vector>
#include <optional>

namespace radar {

struct RBSReply {
    // Эфирное представление: 18 позиций в строгом временном порядке
    // Индекс: Назначение:
    //   0 - F1  (первый кадрирующий)
    //   1 - C1  (информационный бит)
    //   2 - A1  (информационный бит)
    //   3 - C2  (информационный бит)
    //   4 - A2  (информационный бит)
    //   5 - C4  (информационный бит)
    //   6 - A4  (информационный бит)
    //   7 - X   (центральная пауза - всегда 0)
    //   8 - B1  (информационный бит)
    //   9 - D1  (информационный бит)
    //  10 - B2  (информационный бит)
    //  11 - D2  (информационный бит)
    //  12 - B4  (информационный бит)
    //  13 - D4  (информационный бит)
    //  14 - F2  (второй кадрирующий)
    //  15 - SPARE1 (запасной, обычно 0)
    //  16 - SPARE2 (запасной, обычно 0)
    //  17 - SPI (импульс опознавания, если есть)
    static constexpr size_t ETHER_POSITIONS = 18;
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes{};
    
    // Эфирное представление канала подавления - такой же порядок
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes_sls{};
    
    // Декодированные поля (для удобства)
    uint16_t code12{0};  // код в стандартном порядке C1,A1,C2,A2,C4,A4,B1,D1,B2,D2,B4,D4
    bool spi{false};
    std::array<uint8_t, 12> confidence{};
    
    // Координаты
    uint16_t azimuth{0};
    uint16_t range{0};
    double x{0.0}, y{0.0};
    bool is_valid{false};
    
    // ---- Методы для доступа к импульсам по их функциональному назначению ----
    
    // Кадрирующие
    uint8_t f1() const { return ether_amplitudes[0]; }
    uint8_t f2() const { return ether_amplitudes[14]; }
    
    // Биты первой группы (C и A)
    uint8_t c1() const { return ether_amplitudes[1]; }
    uint8_t a1() const { return ether_amplitudes[2]; }
    uint8_t c2() const { return ether_amplitudes[3]; }
    uint8_t a2() const { return ether_amplitudes[4]; }
    uint8_t c4() const { return ether_amplitudes[5]; }
    uint8_t a4() const { return ether_amplitudes[6]; }
    
    // Центральная пауза
    uint8_t mid_pulse() const { return ether_amplitudes[7]; }
    
    // Биты второй группы (B и D)
    uint8_t b1() const { return ether_amplitudes[8]; }
    uint8_t d1() const { return ether_amplitudes[9]; }
    uint8_t b2() const { return ether_amplitudes[10]; }
    uint8_t d2() const { return ether_amplitudes[11]; }
    uint8_t b4() const { return ether_amplitudes[12]; }
    uint8_t d4() const { return ether_amplitudes[13]; }
    
    // Запасные
    uint8_t spare1() const { return ether_amplitudes[15]; }
    uint8_t spare2() const { return ether_amplitudes[16]; }
    
    // SPI
    uint8_t spi_pulse() const { return ether_amplitudes[17]; }
    
    // Универсальный доступ к биту по индексу 0..11 в порядке code12
    // (преобразует эфирный порядок в стандартный порядок code12)
    uint8_t bit(size_t idx) const {
        // Массив соответствия: эфирный индекс для каждого бита code12
        static constexpr uint8_t ether_to_code12[12] = {
            1,  // idx0 (C1)  -> эфирный индекс 1
            2,  // idx1 (A1)  -> эфирный индекс 2
            3,  // idx2 (C2)  -> эфирный индекс 3
            4,  // idx3 (A2)  -> эфирный индекс 4
            5,  // idx4 (C4)  -> эфирный индекс 5
            6,  // idx5 (A4)  -> эфирный индекс 6
            8,  // idx6 (B1)  -> эфирный индекс 8
            9,  // idx7 (D1)  -> эфирный индекс 9
            10, // idx8 (B2)  -> эфирный индекс 10
            11, // idx9 (D2)  -> эфирный индекс 11
            12, // idx10 (B4) -> эфирный индекс 12
            13  // idx11 (D4) -> эфирный индекс 13
        };
        
        if (idx < 12) {
            return ether_amplitudes[ether_to_code12[idx]];
        }
        return 0;
    }
    
    // Для SLS канала
    uint8_t f1_sls() const { return ether_amplitudes_sls[0]; }
    uint8_t f2_sls() const { return ether_amplitudes_sls[14]; }
    uint8_t bit_sls(size_t idx) const {
        static constexpr uint8_t ether_to_code12[12] = {
            1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13
        };
        if (idx < 12) return ether_amplitudes_sls[ether_to_code12[idx]];
        return 0;
    }
};

// ---- UVD Reply - полная версия с эфирным представлением ----
struct UVDReply {
    // Эфирное представление: 80 отсчётов
    // Порядок: [0..39] первый повтор, [40..79] второй повтор
    // Внутри каждого повтора: для каждого из 20 бит:
    // left[i], right[i] - две позиции "активной паузы"
    static constexpr size_t ETHER_POSITIONS = 80;
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes{};
    
    // Эфирное представление канала подавления
    std::array<uint8_t, ETHER_POSITIONS> ether_amplitudes_sls{};
    
    // Декодированные поля от платы
    uint32_t data20{0};               // 20 информационных бит (младшие 20 бит)
    uint32_t error_mask{0};            // 1 = сбойный/недостоверный бит (младшие 20 бит)
    
    // Координаты и служебные поля
    uint16_t azimuth{0};
    uint16_t range{0};
    double x{0.0};
    double y{0.0};
    bool is_valid{false};
    
    // Методы для удобного доступа к отдельным компонентам
    // Получить левую/правую позицию для бита i в повторе r (0 или 1)
    uint8_t left_pulse(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes[repeat * 40 + bit_idx * 2];
    }
    
    uint8_t right_pulse(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes[repeat * 40 + bit_idx * 2 + 1];
    }
    
    // Для SLS канала
    uint8_t left_pulse_sls(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes_sls[repeat * 40 + bit_idx * 2];
    }
    
    uint8_t right_pulse_sls(size_t bit_idx, size_t repeat = 0) const {
        if (bit_idx >= 20 || repeat >= 2) return 0;
        return ether_amplitudes_sls[repeat * 40 + bit_idx * 2 + 1];
    }
};

// ---- Целевой отчет после обработки (без изменений) ----
struct TargetReport {
    enum class SourceType { RBS, UVD } type;
    
    double x{0.0};
    double y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    union {
        struct {
            uint16_t mode3a_code;
            uint16_t modec_altitude;
            bool spi;
        } rbs;
        
        struct {
            uint32_t raw_data20;
            uint8_t octal_id[5];
            uint16_t altitude;
            uint8_t fuel;
            bool pressure_ref;
        } uvd;
    };
    
    uint8_t signal_strength{0};
    bool is_reflection{false};
    bool is_sls_blanked{false};
    bool is_garbled{false};
    
    std::vector<const void*> sources;
};

} // namespace radar