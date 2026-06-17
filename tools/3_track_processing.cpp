// tools/3_track_processing.cpp
#include "radar/track_manager.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cmath>

using namespace radar;

struct PlotData {
    double time_sec;
    double x_km;
    double y_km;
    double azimuth_deg;
    double range_km;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool altitude_valid;
    int altitude_attempts;
    bool spi;
    int reply_count;
    int garble_count;
    double azimuth_span_deg;
    double range_span_km;
    double first_reply_time;
    double last_reply_time;
};

bool parse_plot_line(const std::string& line, PlotData& plot) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    
    if (parts.size() < 17) return false;
    
    try {
        plot.time_sec = std::stod(parts[0]);
        plot.azimuth_deg = std::stod(parts[1]);
        plot.range_km = std::stod(parts[2]);
        plot.x_km = std::stod(parts[3]);
        plot.y_km = std::stod(parts[4]);
        plot.type = parts[5];
        
        // Код в восьмеричном виде - сохраняем как есть
        plot.code_data = static_cast<uint32_t>(std::stoi(parts[6], nullptr, 8));
        
        plot.altitude = std::stoi(parts[7]);
        plot.altitude_valid = (parts[8] == "1");
        plot.altitude_attempts = std::stoi(parts[9]);
        plot.spi = (parts[10] == "1");
        plot.reply_count = std::stoi(parts[11]);
        plot.garble_count = (parts.size() > 12) ? std::stoi(parts[12]) : 0;
        plot.azimuth_span_deg = std::stod(parts[13]);
        plot.range_span_km = std::stod(parts[14]);
        plot.first_reply_time = std::stod(parts[15]);
        plot.last_reply_time = std::stod(parts[16]);
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// Форматирование кода в восьмеричном виде с ведущим нулем
std::string format_code(uint32_t code) {
    std::stringstream ss;
    ss << std::oct << code;
    std::string result = ss.str();
    // Добавляем ведущие нули до 4 цифр
    while (result.length() < 4) {
        result = "0" + result;
    }
    // Добавляем ведущий ноль для восьмеричного числа
    result = "0" + result;
    return result;
}

int main(int argc, char* argv[]) {
    std::string input_file = "plots.txt";
    std::string output_file = "tracks.txt";
    double max_gate_distance_km = 5.0;
    double max_gate_azimuth_deg = 30.0;
    int min_hits_to_confirm = 3;
    int max_coast_count = 10;
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    if (argc > 3) max_gate_distance_km = std::stod(argv[3]);
    if (argc > 4) max_gate_azimuth_deg = std::stod(argv[4]);
    if (argc > 5) min_hits_to_confirm = std::stoi(argv[5]);
    if (argc > 6) max_coast_count = std::stoi(argv[6]);
    
    std::cout << "=== Step 3: Track Processing ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Max gate distance: " << max_gate_distance_km << " km\n";
    std::cout << "Max gate azimuth: " << max_gate_azimuth_deg << "°\n";
    std::cout << "Min hits to confirm: " << min_hits_to_confirm << "\n";
    std::cout << "Max coast count: " << max_coast_count << "\n\n";
    
    // Читаем плоты
    std::ifstream in(input_file);
    std::vector<PlotData> plots;
    std::string line;
    int line_num = 0;
    
    while (std::getline(in, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;
        
        PlotData plot;
        if (parse_plot_line(line, plot)) {
            plots.push_back(plot);
        }
    }
    in.close();
    
    std::cout << "Read " << plots.size() << " plots\n";
    
    // Настройка трекера
    TrackerConfig tracker_config;
    tracker_config.min_hits_to_confirm = min_hits_to_confirm;
    tracker_config.max_coast_count = max_coast_count;
    tracker_config.max_gate_distance = max_gate_distance_km * 1000.0;
    tracker_config.max_gate_azimuth = max_gate_azimuth_deg;
    tracker_config.enable_rbs_tracking = true;
    tracker_config.enable_uvd_tracking = false;
    tracker_config.debug_mode = false;
    tracker_config.process_noise = 0.5;
    tracker_config.measurement_noise = 0.1;
    
    TrackManager track_manager(tracker_config);
    
    std::ofstream out(output_file);
    out << "# Tracks\n";
    out << "# time_sec,track_id,x_km,y_km,speed_km_s,course_deg,code_data,altitude,altitude_valid,confidence,hit_count,state,garble_count,reply_count\n";
    out << "# " << std::string(80, '-') << "\n";
    
    int revolution = 0;
    PlotData prev_plot;
    bool has_prev = false;
    
    for (const auto& plot : plots) {
        // Пропускаем плоты с высоким процентом garble (> 30%)
        double garble_ratio = (plot.reply_count > 0) ? 
            static_cast<double>(plot.garble_count) / plot.reply_count : 0.0;
        
        if (garble_ratio > 0.3) {
            if (!has_prev) {
                has_prev = true;
                prev_plot = plot;
            }
            continue;
        }
        
        // Ручное вычисление скорости между соседними плотами
        double manual_speed_km_s = 0.0;
        double manual_course_deg = 0.0;
        
        if (has_prev) {
            double dt = plot.time_sec - prev_plot.time_sec;
            if (dt > 0.1) {
                double dx = plot.x_km - prev_plot.x_km;
                double dy = plot.y_km - prev_plot.y_km;
                double dist = sqrt(dx*dx + dy*dy);
                manual_speed_km_s = dist / dt;
                manual_course_deg = atan2(dx, dy) * 180.0 / M_PI;
                if (manual_course_deg < 0) manual_course_deg += 360.0;
            }
        }
        has_prev = true;
        prev_plot = plot;
        
        // Создаем отчет для трекера
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = plot.x_km * 1000.0;
        report.y = plot.y_km * 1000.0;
        report.azimuth_deg = plot.azimuth_deg;
        report.range_m = plot.range_km * 1000.0;
        report.rbs.mode3a_code = plot.code_data;
        report.rbs.modec_altitude = plot.altitude_valid ? plot.altitude : 0;
        report.rbs.spi = plot.spi;
        report.signal_strength = static_cast<uint8_t>(100 - garble_ratio * 100);
        
        // Обновляем трекер
        std::vector<TargetReport> reports = {report};
        track_manager.process_targets(reports, revolution++);
        
        // Получаем активные треки
        auto tracks = track_manager.get_active_tracks();
        
        // Записываем треки
        for (const auto& track : tracks) {
            double speed_display = (track.ground_speed > 0.1) ? 
                track.ground_speed / 1000.0 : manual_speed_km_s;
            double course_display = (track.course_deg > 0.1) ? 
                track.course_deg : manual_course_deg;
            
            // Форматируем код в восьмеричном виде
            std::string code_str = format_code(track.mode3a_code);
            
            out << std::fixed << std::setprecision(3) << plot.time_sec << ","
                << track.id << ","
                << std::setprecision(2) << track.x / 1000.0 << ","
                << track.y / 1000.0 << ","
                << std::setprecision(3) << speed_display << ","
                << std::setprecision(1) << course_display << ","
                << code_str << ","
                << track.altitude << ","
                << (track.altitude > 0 ? "1" : "0") << ","
                << std::setprecision(2) << track.confidence << ","
                << track.hit_count << ","
                << static_cast<int>(track.state) << ","
                << plot.garble_count << ","
                << plot.reply_count << "\n";
        }
    }
    
    out.close();
    
    // Вывод статистики
    auto final_tracks = track_manager.get_active_tracks();
    int confirmed = 0;
    for (const auto& t : final_tracks) {
        if (t.state == TrackState::ACTIVE) confirmed++;
    }
    
    std::cout << "Processed " << plots.size() << " plots\n";
    std::cout << "Active tracks: " << final_tracks.size() << "\n";
    std::cout << "Confirmed tracks: " << confirmed << "\n";
    std::cout << "Output written to " << output_file << "\n";
    
    return 0;
}
