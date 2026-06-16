// tools/1_generate_replies.cpp
#include "radar/radar_system.h"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>

using namespace radar;

struct GeneratedReply {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    char type[8];             // "RBS_A" или "RBS_C"
    uint32_t code_data;
    int altitude;
    bool spi;
    uint8_t sls_ratio;
};

void update_target_position(GeneratedTarget& target, double time_seconds, double revolution_time) {
    if (!target.use_linear_motion) return;
    
    double revolution_count = time_seconds / revolution_time;
    
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

// Функция для получения кода Mode C (высота)
uint16_t get_mode_c_code(int altitude_meters) {
    // Высота в сотнях футов
    int altitude_100ft = static_cast<int>(altitude_meters / 30.48);
    
    if (altitude_100ft < -12) altitude_100ft = -12;
    if (altitude_100ft > 1267) altitude_100ft = 1267;
    
    int altitude_100ft_offset = altitude_100ft + 12;
    
    int hundreds = (altitude_100ft_offset / 100) % 10;
    int tens = (altitude_100ft_offset / 10) % 10;
    int units = altitude_100ft_offset % 10;
    
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

void generate_replies(const SystemConfig& config, double duration_seconds, const std::string& output_file) {
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "Error: Cannot open " << output_file << std::endl;
        return;
    }

    out << "# Generated Replies\n";
    out << "# Format: time_sec,azimuth,range,type,code_data,altitude,spi,sls_ratio\n";
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
        bool mode_a_toggle;  // true = Mode A, false = Mode C
    };

    std::vector<TargetTrajectory> trajectories;

    for (auto& target : config.rbs_targets) {
        if (!target.enabled) continue;
        TargetTrajectory traj;
        traj.target = target;
        traj.start_time = 0;
        traj.end_time = duration_seconds;
        traj.is_rbs = true;
        traj.mode_a_toggle = true;  // Начинаем с Mode A
        trajectories.push_back(traj);
        
        std::cout << "Added RBS target: " << target.name 
                  << " at (" << target.azimuth_deg << "°, " << target.range_km << " km)\n";
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
    int progress_step = total_steps / 100;
    if (progress_step < 1) progress_step = 1;

    for (int step = 0; step < total_steps; ++step) {
        if (step % progress_step == 0) {
            int percent = (step * 100) / total_steps;
            std::cout << "\rProgress: " << percent << "% (" << step << "/" << total_steps << ")" << std::flush;
        }

        double time_sec = step * TIME_STEP;
        uint16_t azimuth = step % AZIMUTH_STEPS;

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
                    
                    // ЧЕРЕДОВАНИЕ: Mode A <-> Mode C
                    if (traj.mode_a_toggle) {
                        // Mode A - бортовой номер
                        rbs_code = target.get_rbs_code();
                        spi_value = target.spi;
                        is_mode_a = true;
                    } else {
                        // Mode C - высота
                        rbs_code = get_mode_c_code(target.altitude_meters);
                        spi_value = false;
                        is_mode_a = false;
                    }
                    
                    uint16_t range_bins = static_cast<uint16_t>(
                        target.range_km * 1000.0 / config.radar.range_bin_rbs
                    );
                    
                    auto reply = simulator.generate_rbs(azimuth, range_bins, rbs_code, spi_value);
                    
                    // Записываем с указанием типа
                    out << std::fixed << std::setprecision(6) << time_sec << ","
                        << azimuth << ","
                        << range_bins << ","
                        << (is_mode_a ? "RBS_A" : "RBS_C") << ","
                        << "0" << std::oct << rbs_code << std::dec << ","
                        << (is_mode_a ? 0 : target.altitude_meters) << ","
                        << (spi_value ? "1" : "0") << ","
                        << "0\n";
                    
                    reply_count++;
                    
                    // Переключаем режим для следующего ответа
                    traj.mode_a_toggle = !traj.mode_a_toggle;
                }
            }
        }
    }

    out.close();
    std::cout << "\rProgress: 100% (" << total_steps << "/" << total_steps << ")\n";
    std::cout << "\nGenerated " << reply_count << " replies to " << output_file << std::endl;
}

int main(int argc, char* argv[]) {
    std::string config_file = "radar.conf";
    double duration_seconds = 300.0;
    std::string output_file = "replies.txt";

    if (argc > 1) config_file = argv[1];
    if (argc > 2) duration_seconds = std::stod(argv[2]);
    if (argc > 3) output_file = argv[3];

    std::cout << "=== Step 1: Generate Replies ===\n";
    std::cout << "Config: " << config_file << "\n";
    std::cout << "Duration: " << duration_seconds << " seconds\n";
    std::cout << "Output: " << output_file << "\n\n";

    SystemConfig config = SystemConfig::load_from_file(config_file);
    
    generate_replies(config, duration_seconds, output_file);

    return 0;
}
