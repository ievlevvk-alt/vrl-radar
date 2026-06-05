#pragma once

#include "replies.h"
#include <random>
#include <memory>

namespace radar {

// Конфигурация имитатора
struct SimulatorConfig {
    // Общие параметры
    RadarConfig radar;
    
    // Параметры для RBS
    struct {
        double snr_db{20.0};
        double amp_variation{0.1};
        double f1f2_amp_ratio{1.0};
    } rbs;
    
    // Параметры для УВД
    struct {
        double snr_db{20.0};
        double error_probability{0.01};
    } uvd;
    
    // Параметры для SLS (канал подавления)
    struct {
        bool enabled{false};                // включен ли канал подавления
        double main_to_sls_ratio{1.0};       // отношение амплитуд для главного лепестка
        double sls_attenuation_db{20.0};     // подавление на боковых лепестках (SLS сильнее)
        double sidelobe_probability{0.1};    // вероятность того, что ответ принят по боковому лепестку
    } sls;
};

// Основной класс имитатора
class ReplySimulator {
public:
    explicit ReplySimulator(const SimulatorConfig& config);
    
    // Генерация одиночных ответов
    RBSReply generate_rbs(
        uint16_t azimuth,
        uint16_t range,
        uint16_t code12,
        bool spi = false
    );
    
    UVDReply generate_uvd(
        uint16_t azimuth,
        uint16_t range,
        uint32_t data20
    );
    
    // Генерация перекрытий
    struct OverlapResult {
        std::vector<RBSReply> rbs_mixture;
        std::vector<UVDReply> uvd_mixture;
    };
    
    OverlapResult mix_two_rbs(
        const RBSReply& r1,
        const RBSReply& r2,
        int16_t range_offset,     // сдвиг по дальности в дискретах
        double amp_ratio = 1.0
    );
    
    OverlapResult mix_two_uvd(
        const UVDReply& u1,
        const UVDReply& u2,
        int16_t range_offset,
        double amp_ratio = 1.0
    );
    
    // Доступ к генератору случайных чисел
    std::mt19937& rng() { return rng_; }
    
private:
    // Вспомогательные методы генерации
    void add_noise_to_amplitudes(
        std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps,
        double snr_db
    );
    
    void add_noise_to_amplitudes(
        std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
        double snr_db
    );
    
    // Генерация SLS канала
    void generate_sls_channel_rbs(RBSReply& reply);
    void generate_sls_channel_uvd(UVDReply& reply);
    
    // Декодирование из эфирного представления
    uint16_t decode_rbs_from_ether(const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& amps);
    uint32_t decode_uvd_from_ether(const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps);
    
    // Вычисление маски ошибок для УВД
    uint32_t compute_uvd_error_mask(
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& amps,
        uint32_t original_data20
    );
    
    SimulatorConfig config_;
    std::mt19937 rng_;
    
    class CoordConverter {
    public:
        explicit CoordConverter(const RadarConfig& cfg) : cfg_(cfg) {}
        
        std::pair<double, double> rbs_to_xy(uint16_t az, uint16_t range) const {
            double range_m = range * cfg_.range_bin_rbs;
            double az_deg = az * cfg_.azimuth_per_bin;
            double x, y;
            polar_to_xy(range_m, az_deg, x, y);
            return {x, y};
        }
        
        std::pair<double, double> uvd_to_xy(uint16_t az, uint16_t range) const {
            double range_m = range * cfg_.range_bin_uvd;
            double az_deg = az * cfg_.azimuth_per_bin;
            double x, y;
            polar_to_xy(range_m, az_deg, x, y);
            return {x, y};
        }
        
    private:
        const RadarConfig& cfg_;
    };
    
    CoordConverter converter_;
};

} // namespace radar