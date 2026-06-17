// tools/1_generate_replies.cpp
#include "radar/radar_system.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <random>

using namespace radar;

// Глобальный генератор случайных чисел для ошибок
static std::mt19937 rng(std::random_device{}());

// ---- Функции для работы с Mode C ----

bool is_valid_mode_c_code(uint16_t code) {
    if (code == 0) return false;
    
    uint16_t d_bits = code & 0x07;
    uint16_t a_bits = (code >> 3) & 0x07;
    uint16_t b_bits = (code >> 6) & 0x07;
    
    uint8_t d_decoded = d_bits ^ (d_bits >> 1);
    uint8_t a_decoded = a_bits ^ (a_bits >> 1);
    uint8_t b_decoded = b_bits ^ (b_bits >> 1);
    
    if (d_decoded > 9 || a_decoded > 9 || b_decoded > 9) {
        return false;
    }
    
    int altitude_offset = a_decoded * 100 + b_decoded * 10 + d_decoded;
    int altitude_100ft = altitude_offset - 12;
    
    if (altitude_100ft < -12 || altitude_100ft > 1267) {
        return false;
    }
    
    return true;
}

// Декодирование Mode C - возвращает true если код можно декодировать
// (даже если высота неправильная)
bool decode_mode_c(uint16_t code, int& altitude_meters) {
    if (!is_valid_mode_c_code(code)) {
        altitude_meters = 0;
        return false;
    }
    
    uint16_t d_bits = code & 0x07;
    uint16_t a_bits = (code >> 3) & 0x07;
    uint16_t b_bits = (code >> 6) & 0x07;
    
    uint8_t d_decoded = d_bits ^ (d_bits >> 1);
    uint8_t a_decoded = a_bits ^ (a_bits >> 1);
    uint8_t b_decoded = b_bits ^ (b_bits >> 1);
    
    int altitude_offset = a_decoded * 100 + b_decoded * 10 + d_decoded;
    int altitude_100ft = altitude_offset - 12;
    
    altitude_meters = static_cast<int>(altitude_100ft * 30.48);
    
    // Даже если высота выходит за диапазон, код все равно валидный
    return true;
}

uint16_t corrupt_mode_c_code(uint16_t original_code) {
    std::uniform_int_distribution<int> bit_dist(0, 11);
    int bit_to_flip = bit_dist(rng);
    return original_code ^ (1 << bit_to_flip);
}

uint16_t generate_invalid_mode_c_code() {
    std::uniform_int_distribution<uint16_t> dist(0, 4095);
    uint16_t invalid_code;
    int attempts = 0;
    do {
        invalid_code = dist(rng);
        attempts++;
    } while (is_valid_mode_c_code(invalid_code) && attempts < 100);
    return invalid_code;
}

// ---- Функции для работы с Mode A ----

uint16_t corrupt_mode_a_code(uint16_t original_code) {
    std::uniform_int_distribution<int> bit_dist(0, 11);
    int bit_to_flip = bit_dist(rng);
    return original_code ^ (1 << bit_to_flip);
}

uint16_t generate_invalid_mode_a_code() {
    std::uniform_int_distribution<uint16_t> dist(1, 4095);
    return dist(rng);
}

// ---- Основные функции ----

void update_target_position(GeneratedTarget& target, double time_seconds, double revolution_time) {
    if (!target.use_linear_motion) return;
    
    double course_rad = target.course_deg * M_PI / 180.0;
    double vx = target.speed_m_per_s * sin(course_rad);
    double vy = target.speed_m_per_s * cos(course_rad);
    
    double dx_km = (vx * time_seconds) / 1000.0;
    double dy_km = (vy * time_seconds) / 1000.0;
    
    double current_x_km = target.initial_x_km + dx_km;
    double current_y_km = target.initial_y_km + dy_km;
    
    target.range_km = sqrt(current_x_km*current_x_km + current_y_km*current_y_km);
    target.azimuth_deg = atan2(current_x_km, current_y_km) * 180.0 / M_PI;
    if (target.azimuth_deg < 0) target.azimuth_deg += 360.0;
}

uint16_t get_mode_c_code(int altitude_meters) {
    int altitude_100ft = static_cast<int>(altitude_meters / 30.48);
    if (altitude_100ft < -12) altitude_100ft = -12;
    if (altitude_100ft > 1267) altitude_100ft = 1267;
    
    int altitude_100ft_offset = altitude_100ft + 12;
    int hundreds = (altitude_100ft_offset / 100) % 10;
    int tens = (altitude_100ft_offset / 10) % 10;
    int units = altitude_100ft_offset % 10;
    
    uint8_t units_gray = units ^ (units >> 1);
    uint8_t tens_gray = tens ^ (tens >> 1);
    uint8_t hundreds_gray = hundreds ^ (hundreds >> 1);
    
    uint16_t mode_c_code = 0;
    if (units_gray & 0x01) mode_c_code |= (1 << 0);
    if (units_gray & 0x02) mode_c_code |= (1 << 1);
    if (units_gray & 0x04) mode_c_code |= (1 << 2);
    if (hundreds_gray & 0x01) mode_c_code |= (1 << 3);
    if (hundreds_gray & 0x02) mode_c_code |= (1 << 4);
    if (hundreds_gray & 0x04) mode_c_code |= (1 << 5);
    if (tens_gray & 0x01) mode_c_code |= (1 << 6);
    if (tens_gray & 0x02) mode_c_code |= (1 << 7);
    if (tens_gray & 0x04) mode_c_code |= (1 << 8);
    return mode_c_code;
}

void generate_replies(const SystemConfig& config, double duration_seconds, 
                      const std::string& output_file,
                      double mode_a_error_prob, double mode_c_error_prob, 
                      double invalid_prob) {
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "Error: Cannot open " << output_file << std::endl;
        return;
    }

    out << "# Generated Replies and Antenna Markers\n";
    out << "# Format: time_sec,azimuth,range,type,code_data,altitude,spi,sls_ratio,is_valid,is_garble\n";
    out << "# Special markers:\n";
    out << "#   NORTH - time_sec,0,0,NORTH,0,0,0,0,0,0  (переход через Север)\n";
    out << "#   SECTOR - time_sec,azimuth,0,SECTOR,0,0,0,0,0,0  (каждый 128-й дискрет)\n";
    out << "# " << std::string(80, '-') << "\n";

    SimulatorConfig sim_config;
    sim_config.radar = config.radar;
    sim_config.rbs.snr_db = 20.0;
    sim_config.uvd.snr_db = 20.0;
    sim_config.sls.enabled = true;
    sim_config.sls.sidelobe_probability = 0.1;
    ReplySimulator simulator(sim_config);

    const double REVOLUTION_TIME = 5.0;
    const int AZIMUTH_STEPS = 4096;
    const double TIME_STEP = REVOLUTION_TIME / AZIMUTH_STEPS;

    struct TargetTrajectory {
        GeneratedTarget target;
        double start_time;
        double end_time;
        bool is_rbs;
        bool mode_a_toggle;
    };

    std::vector<TargetTrajectory> trajectories;

    for (auto& target : config.rbs_targets) {
        if (!target.enabled) continue;
        TargetTrajectory traj;
        traj.target = target;
        traj.start_time = 0;
        traj.end_time = duration_seconds;
        traj.is_rbs = true;
        traj.mode_a_toggle = true;
        trajectories.push_back(traj);
        
        std::cout << "Added RBS target: " << target.name 
                  << " at (" << target.azimuth_deg << "°, " << target.range_km << " km)\n";
        std::cout << "  Mode A code: 0" << std::oct << target.get_rbs_code() << std::dec << "\n";
        std::cout << "  Altitude: " << target.altitude_meters << " m\n";
        std::cout << "  Error probabilities: Mode A=" << mode_a_error_prob 
                  << ", Mode C=" << mode_c_error_prob 
                  << ", invalid=" << invalid_prob << "\n\n";
    }

    if (trajectories.empty()) {
        std::cerr << "Warning: No targets enabled in configuration\n";
        out.close();
        return;
    }

    int total_steps = static_cast<int>(duration_seconds / TIME_STEP);
    std::cout << "\nGenerating " << total_steps << " steps over " << duration_seconds << " seconds...\n";
    std::cout << "Time step: " << TIME_STEP << " seconds, Revolution time: " << REVOLUTION_TIME << " seconds\n\n";

    int reply_count = 0;
    int mode_a_errors = 0;
    int mode_c_errors = 0;
    int invalid_codes = 0;
    int progress_step = total_steps / 100;
    if (progress_step < 1) progress_step = 1;

    uint16_t prev_azimuth = 0;
    bool first = true;

    std::uniform_real_distribution<double> error_dist(0.0, 1.0);

    for (int step = 0; step < total_steps; ++step) {
        if (step % progress_step == 0) {
            int percent = (step * 100) / total_steps;
            std::cout << "\rProgress: " << percent << "% (" << step << "/" << total_steps << ")" 
                      << " Errors: " << mode_a_errors + mode_c_errors + invalid_codes << std::flush;
        }

        double time_sec = step * TIME_STEP;
        uint16_t azimuth = step % AZIMUTH_STEPS;

        // --- МАРКЕРЫ ---
        if (!first && azimuth < prev_azimuth && (prev_azimuth - azimuth) > 2048) {
            // Север
            out << std::fixed << std::setprecision(6) << time_sec << ",0,0,NORTH,0,0,0,0,0,0\n";
        }
        first = false;
        prev_azimuth = azimuth;
        
        if (azimuth % 128 == 0) {
            // Сектор
            out << std::fixed << std::setprecision(6) << time_sec << ","
                << azimuth << ",0,SECTOR,0,0,0,0,0,0\n";
        }

        // --- ГЕНЕРАЦИЯ ОТВЕТОВ ---
        for (auto& traj : trajectories) {
            if (time_sec < traj.start_time || time_sec > traj.end_time) continue;

            GeneratedTarget target = traj.target;
            update_target_position(target, time_sec, REVOLUTION_TIME);
            
            double half_beamwidth = config.beamwidth_deg / 2.0;
            double current_az_deg = azimuth * RadarConfig::azimuth_per_bin;
            double az_diff = std::abs(target.azimuth_deg - current_az_deg);
            az_diff = std::min(az_diff, 360.0 - az_diff);
            
            if (target.range_km < 0 || target.range_km > 500) continue;
            
            if (az_diff <= half_beamwidth) {
                if (traj.is_rbs) {
                    uint16_t rbs_code;
                    bool spi_value;
                    bool is_mode_a;
                    bool is_valid = true;
                    int decoded_altitude = 0;
                    
                    if (traj.mode_a_toggle) {
                        // Mode A - всегда is_valid = 1
                        uint16_t original_code = target.get_rbs_code();
                        
                        if (error_dist(rng) < mode_a_error_prob) {
                            if (error_dist(rng) < invalid_prob) {
                                rbs_code = generate_invalid_mode_a_code();
                                invalid_codes++;
                            } else {
                                rbs_code = corrupt_mode_a_code(original_code);
                                mode_a_errors++;
                            }
                        } else {
                            rbs_code = original_code;
                        }
                        
                        spi_value = target.spi;
                        is_mode_a = true;
                        is_valid = true;  // Mode A всегда валидный
                        decoded_altitude = 0;
                    } else {
                        // Mode C - is_valid = 1 если код декодируется
                        uint16_t original_code = get_mode_c_code(target.altitude_meters);
                        
                        if (error_dist(rng) < mode_c_error_prob) {
                            if (error_dist(rng) < invalid_prob) {
                                rbs_code = generate_invalid_mode_c_code();
                                invalid_codes++;
                            } else {
                                rbs_code = corrupt_mode_c_code(original_code);
                                mode_c_errors++;
                            }
                        } else {
                            rbs_code = original_code;
                        }
                        
                        spi_value = false;
                        is_mode_a = false;
                        is_valid = decode_mode_c(rbs_code, decoded_altitude);
                    }
                    
                    uint16_t range_bins = static_cast<uint16_t>(
                        target.range_km * 1000.0 / config.radar.range_bin_rbs
                    );
                    
                    auto reply = simulator.generate_rbs(azimuth, range_bins, rbs_code, spi_value);


                    // Вероятность того, что ответ искажен (перекрыт)
                    double garble_probability = 0.05;  // 5% ответов будут помечены как garble

                    // При генерации RBS_A или RBS_C:
                    bool is_garble = false;
                    if (error_dist(rng) < garble_probability) {
                        is_garble = true;
                        // Для Mode C при garble часто is_valid становится false
                        if (!is_mode_a) {
                            is_valid = false;
                        }
                    }                    
                    
                    // Записываем с добавленным полем is_garble
                    out << std::fixed << std::setprecision(6) << time_sec << ","
                        << azimuth << ","
                        << range_bins << ","
                        << (is_mode_a ? "RBS_A" : "RBS_C") << ","
                        << "0" << std::oct << rbs_code << std::dec << ","
                        << decoded_altitude << ","
                        << (spi_value ? "1" : "0") << ","
                        << "0" << ","
                        << (is_valid ? "1" : "0") << ","
                        << (is_garble ? "1" : "0") << "\n";


                    reply_count++;
                    traj.mode_a_toggle = !traj.mode_a_toggle;
                }
            }
        }
    }

    out.close();
    std::cout << "\rProgress: 100% (" << total_steps << "/" << total_steps << ")" 
              << " Errors: " << mode_a_errors + mode_c_errors + invalid_codes << "\n";
    std::cout << "\nGenerated " << reply_count << " replies to " << output_file << std::endl;
    std::cout << "  Mode A errors: " << mode_a_errors << "\n";
    std::cout << "  Mode C errors: " << mode_c_errors << "\n";
    std::cout << "  Invalid codes: " << invalid_codes << "\n";
    std::cout << "  Total errors: " << mode_a_errors + mode_c_errors + invalid_codes << "\n";
}

int main(int argc, char* argv[]) {
    std::string config_file = "radar.conf";
    double duration_seconds = 300.0;
    std::string output_file = "replies.txt";
    double mode_a_error_prob = 0.5;
    double mode_c_error_prob = 0.5;
    double invalid_prob = 0.3;

    if (argc > 1) config_file = argv[1];
    if (argc > 2) duration_seconds = std::stod(argv[2]);
    if (argc > 3) output_file = argv[3];
    if (argc > 4) mode_a_error_prob = std::stod(argv[4]);
    if (argc > 5) mode_c_error_prob = std::stod(argv[5]);
    if (argc > 6) invalid_prob = std::stod(argv[6]);

    std::cout << "=== Step 1: Generate Replies ===\n";
    std::cout << "Config: " << config_file << "\n";
    std::cout << "Duration: " << duration_seconds << " seconds\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Mode A error prob: " << mode_a_error_prob << "\n";
    std::cout << "Mode C error prob: " << mode_c_error_prob << "\n";
    std::cout << "Invalid code prob: " << invalid_prob << "\n\n";

    SystemConfig config = SystemConfig::load_from_file(config_file);
    
    generate_replies(config, duration_seconds, output_file, 
                     mode_a_error_prob, mode_c_error_prob, invalid_prob);

    return 0;
}
