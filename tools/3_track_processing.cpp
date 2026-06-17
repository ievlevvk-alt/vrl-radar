// tools/3_track_processing.cpp
#include "radar/track_manager.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <map>
#include <set>

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
        
        std::string code_str = parts[6];
        if (plot.type == "RBS" || plot.type == "RBS_A" || plot.type == "RBS_C") {
            plot.code_data = static_cast<uint32_t>(std::stoi(code_str, nullptr, 8));
        } else {
            plot.code_data = static_cast<uint32_t>(std::stoul(code_str));
        }
        
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

std::string format_code(uint32_t code, const std::string& type) {
    if (type == "RBS" || type == "RBS_A" || type == "RBS_C") {
        std::stringstream ss;
        ss << std::oct << code;
        std::string result = ss.str();
        while (result.length() < 4) result = "0" + result;
        return "0" + result;
    } else {
        return std::to_string(code);
    }
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
    
    // ===== НАСТРОЙКА ТРЕКЕРОВ =====
    TrackerConfig tracker_config;
    tracker_config.min_hits_to_confirm = min_hits_to_confirm;
    tracker_config.max_coast_count = max_coast_count;
    tracker_config.max_gate_distance = max_gate_distance_km * 1000.0;
    tracker_config.max_gate_azimuth = max_gate_azimuth_deg;
    tracker_config.enable_rbs_tracking = true;
    tracker_config.enable_uvd_tracking = true;
    tracker_config.debug_mode = false;
    tracker_config.process_noise = 0.5;
    tracker_config.measurement_noise = 0.1;
    
    // ===== ОТКРЫВАЕМ ФАЙЛЫ =====
    std::ifstream in(input_file);
    if (!in.is_open()) {
        std::cerr << "Error: Cannot open " << input_file << std::endl;
        return 1;
    }
    
    std::ofstream out(output_file);
    out << "# Tracks\n";
    out << "# time_sec,track_id,x_km,y_km,speed_km_s,course_deg,code_data,altitude,altitude_valid,confidence,hit_count,state,type\n";
    out << "# " << std::string(80, '-') << "\n";
    
    // ===== СОЗДАЁМ ТРЕКЕРЫ =====
    TrackManager rbs_tracker(tracker_config);
    TrackManager uvd_tracker(tracker_config);
    
    std::map<uint64_t, int> last_hit_rbs;
    std::map<uint64_t, int> last_hit_uvd;
    
    // Для ручного вычисления скорости
    std::map<uint64_t, PlotData> prev_rbs_plot;
    std::map<uint64_t, PlotData> prev_uvd_plot;
    
    std::string line;
    int line_num = 0;
    
    while (std::getline(in, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;
        
        PlotData plot;
        if (!parse_plot_line(line, plot)) continue;
        
        bool is_rbs = (plot.type == "RBS" || plot.type == "RBS_A" || plot.type == "RBS_C");
        
        TargetReport report;
        report.x = plot.x_km * 1000.0;
        report.y = plot.y_km * 1000.0;
        report.azimuth_deg = plot.azimuth_deg;
        report.range_m = plot.range_km * 1000.0;
        report.signal_strength = 100;
        
        int revolution = static_cast<int>(plot.time_sec / 5.0);
        
        if (is_rbs) {
            report.type = TargetReport::SourceType::RBS;
            report.rbs.mode3a_code = static_cast<uint16_t>(plot.code_data);
            report.rbs.modec_altitude = plot.altitude;
            report.rbs.spi = plot.spi;
            
            rbs_tracker.process_targets({report}, revolution);
            
            auto tracks = rbs_tracker.get_active_tracks();
            for (const auto& track : tracks) {
                auto it = last_hit_rbs.find(track.id);
                if (it == last_hit_rbs.end() || it->second != track.hit_count) {
                    
                    // ===== ВЫЧИСЛЯЕМ СКОРОСТЬ ВРУЧНУЮ =====
                    double speed_km_s = track.ground_speed / 1000.0;
                    double course_deg = track.course_deg;
                    
                    // Если трекер дал 0 - вычисляем сами
                    if (speed_km_s < 0.001) {
                        auto prev = prev_rbs_plot.find(track.id);
                        if (prev != prev_rbs_plot.end()) {
                            double dt = plot.time_sec - prev->second.time_sec;
                            if (dt > 0.1) {
                                double dx = plot.x_km - prev->second.x_km;
                                double dy = plot.y_km - prev->second.y_km;
                                double dist = sqrt(dx*dx + dy*dy);
                                speed_km_s = dist / dt;
                                course_deg = atan2(dx, dy) * 180.0 / M_PI;
                                if (course_deg < 0) course_deg += 360.0;
                            }
                        }
                        prev_rbs_plot[track.id] = plot;
                    }
                    // =====================================
                    
                    out << std::fixed << std::setprecision(3) << plot.time_sec << ","
                        << track.id << ","
                        << std::setprecision(2) << track.x / 1000.0 << ","
                        << track.y / 1000.0 << ","
                        << std::setprecision(3) << speed_km_s << ","
                        << std::setprecision(1) << course_deg << ","
                        << format_code(track.mode3a_code, "RBS") << ","
                        << track.altitude << ","
                        << (track.altitude > 0 ? "1" : "0") << ","
                        << std::setprecision(2) << track.confidence << ","
                        << track.hit_count << ","
                        << static_cast<int>(track.state) << ","
                        << "RBS\n";
                    
                    last_hit_rbs[track.id] = track.hit_count;
                }
            }
        } else {
            report.type = TargetReport::SourceType::UVD;
            report.uvd.raw_data20 = plot.code_data;
            report.uvd.altitude = plot.altitude;
            
            uvd_tracker.process_targets({report}, revolution);
            
            auto tracks = uvd_tracker.get_active_tracks();
            for (const auto& track : tracks) {
                auto it = last_hit_uvd.find(track.id);
                if (it == last_hit_uvd.end() || it->second != track.hit_count) {
                    
                    // ===== ВЫЧИСЛЯЕМ СКОРОСТЬ ДЛЯ УВД =====
                    double speed_km_s = track.ground_speed / 1000.0;
                    double course_deg = track.course_deg;
                    
                    if (speed_km_s < 0.001) {
                        auto prev = prev_uvd_plot.find(track.id);
                        if (prev != prev_uvd_plot.end()) {
                            double dt = plot.time_sec - prev->second.time_sec;
                            if (dt > 0.1) {
                                double dx = plot.x_km - prev->second.x_km;
                                double dy = plot.y_km - prev->second.y_km;
                                double dist = sqrt(dx*dx + dy*dy);
                                speed_km_s = dist / dt;
                                course_deg = atan2(dx, dy) * 180.0 / M_PI;
                                if (course_deg < 0) course_deg += 360.0;
                            }
                        }
                        prev_uvd_plot[track.id] = plot;
                    }
                    // =====================================
                    
                    uint64_t display_id = track.id + 1000;
                    
                    out << std::fixed << std::setprecision(3) << plot.time_sec << ","
                        << display_id << ","
                        << std::setprecision(2) << track.x / 1000.0 << ","
                        << track.y / 1000.0 << ","
                        << std::setprecision(3) << speed_km_s << ","
                        << std::setprecision(1) << course_deg << ","
                        << format_code(track.uvd_data20, "UVD") << ","
                        << track.altitude << ","
                        << (track.altitude > 0 ? "1" : "0") << ","
                        << std::setprecision(2) << track.confidence << ","
                        << track.hit_count << ","
                        << static_cast<int>(track.state) << ","
                        << "UVD\n";
                    
                    last_hit_uvd[track.id] = track.hit_count;
                }
            }
        }
    }
    
    out.close();
    in.close();
    
    // ===== СТАТИСТИКА =====
    auto rbs_tracks = rbs_tracker.get_active_tracks();
    auto uvd_tracks = uvd_tracker.get_active_tracks();
    
    int rbs_confirmed = 0, uvd_confirmed = 0;
    for (const auto& t : rbs_tracks) {
        if (t.state == TrackState::ACTIVE) rbs_confirmed++;
    }
    for (const auto& t : uvd_tracks) {
        if (t.state == TrackState::ACTIVE) uvd_confirmed++;
    }
    
    std::cout << "\n=== Results ===\n";
    std::cout << "RBS tracks: " << rbs_tracks.size() << " (confirmed: " << rbs_confirmed << ")\n";
    std::cout << "UVD tracks: " << uvd_tracks.size() << " (confirmed: " << uvd_confirmed << ")\n";
    std::cout << "Output written to " << output_file << "\n";
    
    return 0;
}
