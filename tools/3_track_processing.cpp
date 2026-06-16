// tools/3_track_processing.cpp
#include "radar/track_manager.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

struct PlotData {
    double time_sec;
    double x_km;
    double y_km;
    double azimuth_deg;
    double range_km;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool spi;
    int reply_count;
};

int main(int argc, char* argv[]) {
    std::string input_file = "plots.txt";
    std::string output_file = "tracks.txt";
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    
    std::cout << "=== Step 3: Track Processing ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n\n";
    
    // Читаем плоты
    std::ifstream in(input_file);
    std::vector<PlotData> plots;
    std::string line;
    
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        PlotData plot;
        std::string spi_str;
        
        ss >> plot.time_sec;
        ss.ignore(1, ',');
        ss >> plot.azimuth_deg;
        ss.ignore(1, ',');
        ss >> plot.range_km;
        ss.ignore(1, ',');
        ss >> plot.x_km;
        ss.ignore(1, ',');
        ss >> plot.y_km;
        ss.ignore(1, ',');
        ss >> plot.type;
        ss.ignore(1, ',');
        ss >> plot.code_data;
        ss.ignore(1, ',');
        ss >> plot.altitude;
        ss.ignore(1, ',');
        ss >> spi_str;
        plot.spi = (spi_str == "1");
        ss.ignore(1, ',');
        ss >> plot.reply_count;
        
        plots.push_back(plot);
    }
    in.close();
    
    std::cout << "Read " << plots.size() << " plots\n";
    
    // Создаем трекер
    radar::TrackerConfig tracker_config;
    tracker_config.min_hits_to_confirm = 3;
    tracker_config.max_coast_count = 10;
    tracker_config.max_gate_distance = 2.0;  // 2 км
    tracker_config.max_gate_azimuth = 30.0;
    tracker_config.debug_mode = false;
    
    radar::TrackManager track_manager(tracker_config);
    
    // Обрабатываем плоты последовательно
    std::ofstream out(output_file);
    out << "# Tracks\n";
    out << "# time_sec,track_id,x_km,y_km,speed_km_s,course_deg,code_data,altitude,confidence,hit_count\n";
    
    int revolution = 0;
    for (const auto& plot : plots) {
        // Конвертируем PlotData в TargetReport
        radar::TargetReport report;
        report.type = (plot.type == "RBS") ? 
            radar::TargetReport::SourceType::RBS : 
            radar::TargetReport::SourceType::UVD;
        report.x = plot.x_km * 1000.0;  // в метры
        report.y = plot.y_km * 1000.0;
        report.azimuth_deg = plot.azimuth_deg;
        report.range_m = plot.range_km * 1000.0;
        
        if (plot.type == "RBS") {
            report.rbs.mode3a_code = plot.code_data;
            report.rbs.modec_altitude = plot.altitude;
            report.rbs.spi = plot.spi;
        } else {
            report.uvd.raw_data20 = plot.code_data;
            report.uvd.altitude = plot.altitude;
        }
        
        // Обновляем трекер
        std::vector<radar::TargetReport> reports = {report};
        track_manager.process_targets(reports, revolution++);
        
        // Получаем активные треки
        auto tracks = track_manager.get_active_tracks();
        
        // Записываем треки
        for (const auto& track : tracks) {
            out << std::fixed << std::setprecision(3) << plot.time_sec << ","
                << track.id << ","
                << track.x / 1000.0 << ","  // в км
                << track.y / 1000.0 << ","
                << track.ground_speed / 1000.0 << ","  // в км/с
                << track.course_deg << ","
                << track.mode3a_code << ","
                << track.altitude << ","
                << track.confidence << ","
                << track.hit_count << "\n";
        }
    }
    out.close();
    
    std::cout << "Processed " << plots.size() << " plots, "
              << track_manager.get_active_tracks().size() << " active tracks\n";
    
    return 0;
}
