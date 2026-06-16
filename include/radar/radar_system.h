// file: include/radar/radar_system.h
#pragma once

#include "cluster_tracker.h"
#include "cluster_processor.h"
#include "simulator.h"
#include "track_manager.h"
#include <string>
#include <fstream>
#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <algorithm>

namespace radar {

// file: include/radar/radar_system.h

struct GeneratedTarget {
    enum class Type { RBS, UVD } type;
    
    // Основные параметры цели
    std::string name;                           // Имя цели (для отладки)
    double azimuth_deg;                         // Азимут в градусах (0-360)
    double range_km;                            // Дальность в км
    
    // Параметры ответов
    uint16_t rbs_code_octal;                    // Код для RBS в восьмеричном виде (0-7777)
    uint32_t uvd_data_dec;                      // Данные для УВД в десятичном виде (0-1048575)
    
    // Параметры движения
    double azimuth_speed_deg_per_rev;            // Скорость изменения азимута (град/оборот)
    double range_speed_km_per_rev;               // Скорость изменения дальности (км/оборот)
    
    // Параметры RBS
    bool spi;                                    // SPI для RBS (только для Mode A)
    
    // Параметры генерации
    bool enabled;                                // Активна ли цель
    int update_every_n_revolutions;              // Появляется каждые N оборотов (1 = каждый оборот)
    int revolution_offset;                       // Смещение для появления (0 = первый оборот)
    
    // Параметры высоты (общие для RBS и UVD)
    int altitude_meters;                         // Высота в метрах (-1000 до +126750)
    bool enable_altitude;                        // Передавать ли высоту
    
    // Параметры чередования для RBS
    bool alternate_code_altitude;                // Чередовать ответы с кодом и высотой (Mode A/Mode C)
    mutable int current_mode{0};                 // 0 = код (Mode A), 1 = высота (Mode C)
    
    // Параметры чередования для УВД
    bool alternate_data_altitude{false};                // Чередовать ответы с данными и высотой
    mutable int uvd_current_mode{0};             // 0 = данные (бортовой номер), 1 = высота
    
    // ========================================================================
    // Методы для RBS
    // ========================================================================
    
    // Получить бортовой номер (Mode A)
    uint16_t get_rbs_code() const {
        return rbs_code_octal;
    }
    
    // Преобразование высоты в код Грея для Mode C (RBS)
    uint16_t get_mode_c_code() const {
        // Высота в сотнях футов
        int altitude_100ft = static_cast<int>(altitude_meters / 30.48);
        
        // Ограничения Mode C: -1200 до +126700 футов
        if (altitude_100ft < -12) altitude_100ft = -12;
        if (altitude_100ft > 1267) altitude_100ft = 1267;
        
        // Смещение для устранения отрицательных значений
        int altitude_100ft_offset = altitude_100ft + 12;
        
        // Преобразование в двоично-десятичный код (BCD)
        int hundreds = (altitude_100ft_offset / 100) % 10;
        int tens = (altitude_100ft_offset / 10) % 10;
        int units = altitude_100ft_offset % 10;
        
        // Формирование 12-битного кода Грея
        uint16_t mode_c_code = 0;
        
        // Блок Units (биты 0-2: D1, D2, D4)
        uint8_t units_gray = units ^ (units >> 1);
        if (units_gray & 0x01) mode_c_code |= (1 << 0);  // D1
        if (units_gray & 0x02) mode_c_code |= (1 << 1);  // D2
        if (units_gray & 0x04) mode_c_code |= (1 << 2);  // D4
        
        // Блок Hundreds (биты 3-5: A1, A2, A4)
        uint8_t hundreds_gray = hundreds ^ (hundreds >> 1);
        if (hundreds_gray & 0x01) mode_c_code |= (1 << 3);  // A1
        if (hundreds_gray & 0x02) mode_c_code |= (1 << 4);  // A2
        if (hundreds_gray & 0x04) mode_c_code |= (1 << 5);  // A4
        
        // Блок Tens (биты 6-8: B1, B2, B4)
        uint8_t tens_gray = tens ^ (tens >> 1);
        if (tens_gray & 0x01) mode_c_code |= (1 << 6);  // B1
        if (tens_gray & 0x02) mode_c_code |= (1 << 7);  // B2
        if (tens_gray & 0x04) mode_c_code |= (1 << 8);  // B4
        
        return mode_c_code;
    }
    
    // Получить текущий код RBS (Mode A или Mode C) и SPI
    bool get_current_rbs_reply(uint16_t& code, bool& spi_out) const {
        if (enable_altitude && alternate_code_altitude) {
            if (current_mode == 0) {
                // Mode A - ответ с бортовым номером, SPI может быть
                code = get_rbs_code();
                spi_out = spi;
                return true;   // Mode A
            } else {
                // Mode C - ответ с высотой, SPI не передаётся
                code = get_mode_c_code();
                spi_out = false;
                return false;  // Mode C
            }
        }
        // Только Mode A
        code = get_rbs_code();
        spi_out = spi;
        return true;
    }
    
    // Переключить режим RBS
    void toggle_rbs_mode() {
        if (alternate_code_altitude) {
            current_mode = (current_mode + 1) % 2;
        }
    }
    
    // ========================================================================
    // Методы для УВД
    // ========================================================================
    
    // Получить высоту в десятках метров
    uint16_t get_altitude_decameters() const {
        int alt = altitude_meters;
        if (alt < -1000) alt = -1000;
        if (alt > 126750) alt = 126750;
        
        int alt_dam = alt / 10;
        if (alt_dam < 0) alt_dam = 0;
        
        return static_cast<uint16_t>(alt_dam & 0x1FFF);
    }
    
    // Получить только высоту (без идентификатора) для УВД
    uint32_t get_uvd_altitude_only() const {
        uint16_t altitude_dam = get_altitude_decameters();
        uint8_t q_code = 1;  // Q-код: 1 = стандартное давление
        
        uint32_t result = 0;
        result |= (altitude_dam & 0x1FFF);           // биты 0-12: высота
        result |= (q_code & 0x03) << 13;             // биты 13-14: Q-код
        // биты 15-19 оставляем 0
        
        return result & 0x0FFFFF;
    }
    
    // Получить текущие данные УВД (данные или высота)
    uint32_t get_current_uvd_data() const {
        if (!enable_altitude || !alternate_data_altitude) {
            // Только данные (бортовой номер)
            return uvd_data_dec & 0x0FFFFF;
        }
        
        if (uvd_current_mode == 0) {
            // Режим данных (бортовой номер)
            return uvd_data_dec & 0x0FFFFF;
        } else {
            // Режим высоты
            return get_uvd_altitude_only();
        }
    }
    
    // Переключить режим УВД
    void toggle_uvd_mode() {
        if (alternate_data_altitude) {
            uvd_current_mode = (uvd_current_mode + 1) % 2;
        }
    }
    
    // Получить полные данные УВД с высотой (для обратной совместимости)
    uint32_t get_uvd_data_with_altitude() const {
        if (!enable_altitude) {
            return uvd_data_dec & 0x0FFFFF;
        }
        
        uint16_t altitude_dam = get_altitude_decameters();
        uint8_t q_code = 1;
        uint8_t id = uvd_data_dec & 0x1F;
        
        uint32_t result = 0;
        result |= (altitude_dam & 0x1FFF);           // биты 0-12: высота
        result |= (q_code & 0x03) << 13;             // биты 13-14: Q-код
        result |= (id & 0x1F) << 15;                 // биты 15-19: идентификатор
        
        return result & 0x0FFFFF;
    }

    // Параметры линейного движения (добавить)
    bool use_linear_motion{false};          // Флаг использования линейного движения
    double speed_m_per_s{0.0};              // Скорость в м/с
    double course_deg{0.0};                 // Курс в градусах
    double initial_x_km{0.0};               // Начальная X (км)
    double initial_y_km{0.0};               // Начальная Y (км)

    // Метод для обновления позиции при линейном движении
    void update_linear_position(double time_delta_seconds) {
        if (!use_linear_motion) return;
        double course_rad = course_deg * M_PI / 180.0;
        double vx = speed_m_per_s * sin(course_rad);
        double vy = speed_m_per_s * cos(course_rad);
        // Пересчет в км/с
        double vx_km_s = vx / 1000.0;
        double vy_km_s = vy / 1000.0;

        // Обновляем положение (в км)
        // azimuth_deg и range_km пересчитываются из x,y
        double current_x_km = initial_x_km + vx_km_s * time_delta_seconds;
        double current_y_km = initial_y_km + vy_km_s * time_delta_seconds;
        range_km = sqrt(current_x_km*current_x_km + current_y_km*current_y_km);
        azimuth_deg = atan2(current_x_km, current_y_km) * 180.0 / M_PI;
        if (azimuth_deg < 0) azimuth_deg += 360.0;
    }

};


// Конфигурация всей системы
struct SystemConfig {
    RadarConfig radar;
    SimulatorConfig simulator;
    TrackerConfig tracker;
    
    struct ProcessingConfig {
        int max_gap_azimuth{8};
        int range_window{30};
        uint16_t range_tolerance{5};
        int min_hits{2};
        std::string output_file{"targets.txt"};
    } processing;
    
    double beamwidth_deg{5.0};
    
    std::vector<GeneratedTarget> rbs_targets;
    std::vector<GeneratedTarget> uvd_targets;
    
    static SystemConfig load_from_file(const std::string& filename);
    void save_to_file(const std::string& filename) const;
    
    bool has_targets() const {
        return !rbs_targets.empty() || !uvd_targets.empty();
    }
};

// Класс, объединяющий всю систему
class RadarSystem {
public:
    explicit RadarSystem(const SystemConfig& config);
    
    bool initialize();
    void process_scan(const ScanReplies& scan);
    void shutdown();
    
    ScanReplies generate_test_scan(uint16_t azimuth, uint32_t timestamp);
    
    using TargetCallback = std::function<void(const TargetReport&)>;
    void set_target_callback(TargetCallback callback) { target_callback_ = callback; }
    
    void enable_tracking(bool enable) { tracking_enabled_ = enable; }
    std::vector<Track> get_tracks() const;
    
    // Логирование входных ответов
    void enable_input_logging(bool enable, const std::string& filename = "input_log.txt");
    
    // Логирование сырых ответов (добавлено)
    void enable_raw_reply_logging(bool enable, const std::string& filename = "raw_replies.txt");
    
    struct Statistics {
        uint32_t scans_processed{0};
        uint32_t clusters_completed{0};
        uint32_t targets_reported{0};
        uint32_t rbs_targets{0};
        uint32_t uvd_targets{0};
        uint32_t garbled_targets{0};
        uint32_t north_markers{0};
        uint32_t active_tracks{0};
        
        void print() const;
    };
    
    const Statistics& get_statistics() const { return stats_; }
    
private:
    void end_of_revolution();
    void on_cluster_completed(const TargetCluster& cluster);
    void process_revolution_complete();
    void write_target_to_file(const TargetReport& target);
    void write_track_to_file(const Track& track);
    void write_north_marker();
    void sort_and_flush_pending_targets();
    
    SystemConfig config_;
    ClusterTracker tracker_;
    ClusterProcessor processor_;
    std::unique_ptr<ReplySimulator> simulator_;
    
    std::ofstream output_file_;
    std::ofstream input_log_file_;
    std::ofstream raw_reply_file_;        // Добавлено
    bool raw_reply_logging_enabled_{false}; // Добавлено
    TargetCallback target_callback_;
    
    Statistics stats_;
    uint32_t scan_counter_{0};
    uint16_t last_azimuth_{0};
    
    std::vector<TargetReport> pending_targets_;
    std::vector<TargetReport> revolution_targets_;
    
    bool tracking_enabled_{true};
    std::unique_ptr<TrackManager> track_manager_;
    
    uint32_t current_revolution_{0};
};

} // namespace radar
