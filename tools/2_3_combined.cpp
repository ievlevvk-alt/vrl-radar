// tools/2_3_combined.cpp
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/core/config.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <set>

using namespace vrl::radar;

// Константы для преобразования
constexpr double AZIMUTH_BIN_DEG = 360.0 / 4096.0;

// ============================================================================
// СТРУКТУРЫ ДАННЫХ
// ============================================================================

struct Reply {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool spi;
    uint8_t sls_ratio;
    bool is_valid;
    bool is_garble;
    double x;
    double y;
};

struct PlotData {
    double time_sec;
    double azimuth_deg;
    double range_km;
    double x_km;
    double y_km;
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

// ============================================================================
// ФОРМАТИРОВАНИЕ КОДА
// ============================================================================

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

// ============================================================================
// ЗАПИСЬ ПЛОТОВ В ФАЙЛ
// ============================================================================

void write_plots_to_file(const std::vector<PlotData>& plots, std::ofstream& out_plots) {
    if (plots.empty()) {
        std::cout << "DEBUG: plots empty, skipping write" << std::endl;
        return;
    }
    if (!out_plots.is_open()) {
        std::cout << "DEBUG: out_plots is NOT open" << std::endl;
        return;
    }
    
    std::cout << "DEBUG: Writing " << plots.size() << " plots to file" << std::endl;

    if (plots.empty() || !out_plots.is_open()) return;
    
    for (const auto& plot : plots) {
        out_plots << std::fixed << std::setprecision(6) << plot.time_sec << ","
                  << std::setprecision(3) << plot.azimuth_deg << ","
                  << plot.range_km << ","
                  << plot.x_km << ","
                  << plot.y_km << ","
                  << plot.type << ","
                  << std::oct << plot.code_data << std::dec << ","
                  << plot.altitude << ","
                  << (plot.altitude_valid ? "1" : "0") << ","
                  << plot.altitude_attempts << ","
                  << (plot.spi ? "1" : "0") << ","
                  << plot.reply_count << ","
                  << plot.garble_count << ","
                  << plot.azimuth_span_deg << ","
                  << plot.range_span_km << ","
                  << plot.first_reply_time << ","
                  << plot.last_reply_time << "\n";
    }
    out_plots.flush();
}

// ============================================================================
// ПАРСИНГ ОТВЕТОВ
// ============================================================================

bool parse_reply_line(const std::string& line, Reply& reply) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() < 10) return false;
    
    try {
        reply.time_sec = std::stod(parts[0]);
        reply.azimuth = static_cast<uint16_t>(std::stoi(parts[1]));
        reply.range = static_cast<uint16_t>(std::stoi(parts[2]));
        reply.type = parts[3];
        
        if (!parts[4].empty() && parts[4][0] == '0') {
            reply.code_data = static_cast<uint32_t>(std::stoi(parts[4], nullptr, 8));
        } else {
            reply.code_data = static_cast<uint32_t>(std::stoi(parts[4]));
        }
        
        reply.altitude = std::stoi(parts[5]);
        reply.spi = (parts[6] == "1");
        reply.sls_ratio = static_cast<uint8_t>(std::stoi(parts[7]));
        reply.is_valid = (parts[8] == "1");
        reply.is_garble = (parts[9] == "1");
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ============================================================================
// КЛАСТЕРИЗАТОР
// ============================================================================

class OnlineClusterer {
public:
    OnlineClusterer(double range_bin_m = 30.0,
                    int range_threshold_bins = 5,
                    int azimuth_threshold_bins = 3,
                    int completion_gap_bins = 8)
        : range_bin_m_(range_bin_m)
        , range_threshold_bins_(range_threshold_bins)
        , azimuth_threshold_bins_(azimuth_threshold_bins)
        , completion_gap_bins_(completion_gap_bins)
        , current_azimuth_(0) {}
    
    void update_azimuth(uint16_t azimuth) {
        current_azimuth_ = azimuth;
    }
    
    void add_reply(const Reply& reply) {
        std::cout << "[ADD_REPLY] type=" << reply.type 
                << ", az=" << reply.azimuth 
                << ", range=" << reply.range 
                << ", active_clusters=" << active_clusters_.size() << std::endl;
        
        double az_rad = (reply.azimuth * AZIMUTH_BIN_DEG) * M_PI / 180.0;
        double range_m = reply.range * range_bin_m_;
        
        Reply r = reply;
        r.x = range_m * sin(az_rad);
        r.y = range_m * cos(az_rad);
        
        bool added = false;
        for (auto& cluster : active_clusters_) {
            if (cluster.is_near(r, range_threshold_bins_, azimuth_threshold_bins_)) {
                cluster.add_reply(r);
                added = true;
                break;
            }
        }
        
        if (!added) {
            Cluster new_cluster;
            new_cluster.add_reply(r);
            active_clusters_.push_back(new_cluster);
        }
        
        current_azimuth_ = reply.azimuth;
    }



    void process_sector(uint16_t azimuth) {
        current_azimuth_ = azimuth;
        check_completed_clusters();
        
        // ОТЛАДКА
        std::cout << "[SECTOR] azimuth=" << azimuth 
                << ", active_clusters=" << active_clusters_.size() 
                << ", completed_plots=" << completed_plots_.size() << std::endl;
    }


    std::vector<PlotData> get_completed_plots() {
        std::vector<PlotData> result;
        
        // 1. Сначала забираем уже готовые плоты из completed_plots_
        result = std::move(completed_plots_);
        completed_plots_.clear();
        
        // 2. Проверяем активные кластеры и создаём новые плоты
        auto it = active_clusters_.begin();
        while (it != active_clusters_.end()) {
            if (it->is_completed(current_azimuth_, completion_gap_bins_)) {
                if (it->replies.size() >= 2) {
                    PlotData plot = create_plot_from_cluster(*it);
                    result.push_back(plot);
                }
                it = active_clusters_.erase(it);
            } else {
                ++it;
            }
        }
        
        std::cout << "[GET_PLOTS] Returning " << result.size() << " plots" << std::endl;
        return result;
    }


    void finish_all() {
        for (auto& cluster : active_clusters_) {
            if (cluster.replies.size() >= 2) {
                PlotData plot = create_plot_from_cluster(cluster);
                completed_plots_.push_back(plot);
            }
        }
        active_clusters_.clear();
    }
    
    std::vector<PlotData> get_final_plots() {
        auto result = std::move(completed_plots_);
        completed_plots_.clear();
        return result;
    }
    
private:
    struct Cluster {
        std::vector<Reply> replies;
        uint16_t min_azimuth{4096};
        uint16_t max_azimuth{0};
        uint16_t min_range{65535};
        uint16_t max_range{0};
        uint16_t last_azimuth{0};
        uint16_t last_range{0};
        double first_time{0};
        double last_time{0};
        double center_x{0};
        double center_y{0};
        bool has_center{false};
        int garble_count{0};
        
        std::map<uint32_t, int> code_counts;
        std::map<int, int> altitude_counts;
        uint32_t best_code{0};
        int best_altitude{0};
        int best_code_count{0};
        int best_altitude_count{0};
        
        void add_reply(const Reply& r) {
            // ОТЛАДКА: показываем, что ответ добавляется
            std::cout << "[CLUSTER_ADD] type=" << r.type 
                    << ", az=" << r.azimuth 
                    << ", range=" << r.range << std::endl;
            
            if (replies.empty()) {
                first_time = r.time_sec;
                center_x = r.x;
                center_y = r.y;
                has_center = true;
                min_azimuth = max_azimuth = r.azimuth;
                min_range = max_range = r.range;
                last_azimuth = r.azimuth;
                last_range = r.range;
            }
            replies.push_back(r);
            if (r.is_garble) garble_count++;
            
            last_time = r.time_sec;
            last_azimuth = r.azimuth;
            last_range = r.range;
            min_azimuth = std::min(min_azimuth, r.azimuth);
            max_azimuth = std::max(max_azimuth, r.azimuth);
            min_range = std::min(min_range, r.range);
            max_range = std::max(max_range, r.range);
            
            if (r.type == "UVD_DATA" || r.type == "RBS_A") {
                code_counts[r.code_data]++;
                if (code_counts[r.code_data] > best_code_count) {
                    best_code_count = code_counts[r.code_data];
                    best_code = r.code_data;
                }
            }
            
            if (r.altitude > 0) {
                altitude_counts[r.altitude]++;
                if (altitude_counts[r.altitude] > best_altitude_count) {
                    best_altitude_count = altitude_counts[r.altitude];
                    best_altitude = r.altitude;
                }
            }
            
            if (has_center) {
                double alpha = 1.0 / replies.size();
                center_x = center_x * (1 - alpha) + r.x * alpha;
                center_y = center_y * (1 - alpha) + r.y * alpha;
            }
        }


        bool is_near(const Reply& r, int range_threshold_bins, int azimuth_threshold_bins) const {
            if (replies.empty()) return true;
            
            int range_diff = std::abs(static_cast<int>(r.range) - static_cast<int>(last_range));
            if (range_diff > range_threshold_bins) return false;
            
            int az_diff = static_cast<int>(r.azimuth) - static_cast<int>(last_azimuth);
            if (az_diff < 0) az_diff += 4096;
            if (az_diff > azimuth_threshold_bins) return false;
            
            return true;
        }
        
        bool is_completed(uint16_t current_azimuth, int completion_gap_bins) const {
            if (replies.empty()) return false;
            
            int az_diff = static_cast<int>(current_azimuth) - last_azimuth;
            if (az_diff < 0) az_diff += 4096;
            
            return az_diff > completion_gap_bins;
        }
    };
    
    void check_completed_clusters() {
        auto it = active_clusters_.begin();
        while (it != active_clusters_.end()) {
            if (it->is_completed(current_azimuth_, completion_gap_bins_)) {
                std::cout << "[CHECK] Cluster completed! replies=" << it->replies.size() << std::endl;
                if (it->replies.size() >= 2) {
                    PlotData plot = create_plot_from_cluster(*it);
                    completed_plots_.push_back(plot);
                    std::cout << "[CHECK] Plot created: type=" << plot.type 
                            << ", code=" << plot.code_data << std::endl;
                }
                it = active_clusters_.erase(it);
            } else {
                ++it;
            }
        }
    }


    PlotData create_plot_from_cluster(const Cluster& cluster) {
        PlotData plot;
        
        bool has_rbs = false;
        bool has_uvd = false;
        
        for (const auto& reply : cluster.replies) {
            if (reply.type == "RBS_A" || reply.type == "RBS_C") has_rbs = true;
            if (reply.type == "UVD_DATA" || reply.type == "UVD_ALT") has_uvd = true;
        }
        
        if (has_rbs && has_uvd) {
            plot.type = "MIXED";
        } else if (has_rbs) {
            plot.type = "RBS";
        } else if (has_uvd) {
            plot.type = "UVD";
        } else {
            plot.type = "UNKNOWN";
        }
        
        if (cluster.best_code != 0) {
            plot.code_data = cluster.best_code;
        } else {
            plot.code_data = cluster.replies[0].code_data;
        }
        
        if (cluster.best_altitude != 0) {
            plot.altitude = cluster.best_altitude;
            plot.altitude_valid = true;
            plot.altitude_attempts = cluster.best_altitude_count;
        } else {
            plot.altitude = 0;
            plot.altitude_valid = false;
            plot.altitude_attempts = 0;
        }
        
        plot.reply_count = cluster.replies.size();
        plot.garble_count = cluster.garble_count;
        plot.spi = false;
        
        double sum_sin = 0.0;
        double sum_cos = 0.0;
        double sum_range_bins = 0.0;
        double first_time = cluster.replies[0].time_sec;
        double last_time = cluster.replies[0].time_sec;
        
        for (const auto& reply : cluster.replies) {
            double az_rad = reply.azimuth * AZIMUTH_BIN_DEG * M_PI / 180.0;
            sum_sin += sin(az_rad);
            sum_cos += cos(az_rad);
            sum_range_bins += reply.range;
            first_time = std::min(first_time, reply.time_sec);
            last_time = std::max(last_time, reply.time_sec);
        }
        
        double avg_az_rad = atan2(sum_sin, sum_cos);
        if (avg_az_rad < 0) avg_az_rad += 2 * M_PI;
        
        double avg_azimuth_bins = avg_az_rad * 4096.0 / (2 * M_PI);
        double avg_range_bins = sum_range_bins / cluster.replies.size();
        
        plot.azimuth_deg = avg_azimuth_bins * AZIMUTH_BIN_DEG;
        plot.range_km = avg_range_bins * range_bin_m_ / 1000.0;
        plot.x_km = plot.range_km * sin(avg_az_rad);
        plot.y_km = plot.range_km * cos(avg_az_rad);
        
        int az_span_bins = static_cast<int>(cluster.max_azimuth) - static_cast<int>(cluster.min_azimuth);
        if (az_span_bins < 0) az_span_bins += 4096;
        int range_span_bins = static_cast<int>(cluster.max_range) - static_cast<int>(cluster.min_range);
        
        plot.azimuth_span_deg = az_span_bins * AZIMUTH_BIN_DEG;
        plot.range_span_km = range_span_bins * range_bin_m_ / 1000.0;
        
        plot.time_sec = (first_time + last_time) / 2.0;
        plot.first_reply_time = first_time;
        plot.last_reply_time = last_time;
        
        return plot;
    }
    
    double range_bin_m_;
    int range_threshold_bins_;
    int azimuth_threshold_bins_;
    int completion_gap_bins_;
    
    uint16_t current_azimuth_{0};
    
    std::vector<Cluster> active_clusters_;
    std::vector<PlotData> completed_plots_;
};

// ============================================================================
// ОБРАБОТКА ПЛОТА В ТРЕКЕРЕ
// ============================================================================

void process_plot_in_tracker(const PlotData& plot, 
                             TrackManager& tracker,
                             std::map<uint64_t, int>& last_hit,
                             std::map<uint64_t, PlotData>& prev_plot,
                             std::ofstream& out_tracks,
                             const std::string& type,
                             uint64_t id_offset,
                             double revolution_time) {
    
    std::cout << "[TRACKER] Processing plot: type=" << type 
              << ", time=" << plot.time_sec 
              << ", code=" << plot.code_data << std::endl;

    TargetReport report;
    report.x = plot.x_km * 1000.0;
    report.y = plot.y_km * 1000.0;
    report.azimuth_deg = plot.azimuth_deg;
    report.range_m = plot.range_km * 1000.0;
    report.signal_strength = 100;
    
    int revolution = static_cast<int>(plot.time_sec / revolution_time);
    
    if (type == "RBS") {
        report.type = TargetReport::SourceType::RBS;
        report.rbs.mode3a_code = static_cast<uint16_t>(plot.code_data);
        report.rbs.modec_altitude = plot.altitude;
        report.rbs.spi = plot.spi;
    } else {
        report.type = TargetReport::SourceType::UVD;
        report.uvd.raw_data20 = plot.code_data;
        report.uvd.altitude = plot.altitude;
    }
    
    tracker.process_targets({report}, revolution);
    std::cout << "[TRACKER] process_targets done" << std::endl;
    
    auto tracks = tracker.get_active_tracks();
    std::cout << "[TRACKER] Got " << tracks.size() << " active tracks" << std::endl;

    for (const auto& track : tracks) {
        std::cout << "[TRACKER] Track id=" << track.id 
                  << ", hit_count=" << track.hit_count 
                  << ", state=" << static_cast<int>(track.state) << std::endl;

        auto it = last_hit.find(track.id);
        bool is_new_or_updated = (it == last_hit.end() || it->second != track.hit_count);
        std::cout << "[TRACKER] is_new_or_updated=" << is_new_or_updated << std::endl;

        if (is_new_or_updated) {
            std::cout << "[TRACKER] WRITING TRACK TO FILE!" << std::endl;
            double speed_km_s = track.ground_speed / 1000.0;
            double course_deg = track.course_deg;
            
            if (speed_km_s < 0.001) {
                auto prev = prev_plot.find(track.id);
                if (prev != prev_plot.end()) {
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
                prev_plot[track.id] = plot;
            }
            
            uint64_t display_id = track.id + id_offset;
            
            uint32_t code = (type == "RBS") ? track.mode3a_code : track.uvd_data20;
            

            out_tracks << std::fixed << std::setprecision(3) << plot.time_sec << ","
                    << display_id << ","
                    << std::setprecision(2) << track.x / 1000.0 << ","
                    << track.y / 1000.0 << ","
                    << std::setprecision(3) << speed_km_s << ","
                    << std::setprecision(1) << course_deg << ","
                    << format_code(code, type) << ","
                    << track.altitude << ","
                    << (track.altitude > 0 ? "1" : "0") << ","
                    << std::setprecision(2) << track.confidence << ","
                    << track.hit_count << ","
                    << static_cast<int>(track.state) << ","
                    << type << ","
                    << (track.code_reliable ? "1" : "0") << ","
                    << (track.altitude_reliable ? "1" : "0") << "\n";            


            last_hit[track.id] = track.hit_count;
        } else {
            std::cout << "[TRACKER] SKIPPING (already written)" << std::endl;
        }

#if 0
        if (it == last_hit.end() || it->second != track.hit_count) {
            
            double speed_km_s = track.ground_speed / 1000.0;
            double course_deg = track.course_deg;
            
            if (speed_km_s < 0.001) {
                auto prev = prev_plot.find(track.id);
                if (prev != prev_plot.end()) {
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
                prev_plot[track.id] = plot;
            }
            
            uint64_t display_id = track.id + id_offset;
            
            uint32_t code = (type == "RBS") ? track.mode3a_code : track.uvd_data20;
            

            out_tracks << std::fixed << std::setprecision(3) << plot.time_sec << ","
                    << display_id << ","
                    << std::setprecision(2) << track.x / 1000.0 << ","
                    << track.y / 1000.0 << ","
                    << std::setprecision(3) << speed_km_s << ","
                    << std::setprecision(1) << course_deg << ","
                    << format_code(code, type) << ","
                    << track.altitude << ","
                    << (track.altitude > 0 ? "1" : "0") << ","
                    << std::setprecision(2) << track.confidence << ","
                    << track.hit_count << ","
                    << static_cast<int>(track.state) << ","
                    << type << ","
                    << (track.code_reliable ? "1" : "0") << ","
                    << (track.altitude_reliable ? "1" : "0") << "\n";            


            last_hit[track.id] = track.hit_count;
        }
#endif        
    }
}

// ============================================================================
// ЗАГРУЗКА КОНФИГУРАЦИИ
// ============================================================================

struct ProcessingConfig {
    std::string input_file = "replies.txt";
    std::string tracks_file = "tracks_combined.txt";
    std::string plots_file = "";
    
    int range_threshold_bins = 5;
    int azimuth_threshold_bins = 3;
    int completion_gap_bins = 8;
    
    double max_gate_distance_km = 5.0;
    double max_gate_azimuth_deg = 30.0;
    int min_hits_to_confirm = 3;
    int max_coast_count = 10;
    double process_noise = 0.5;
    double measurement_noise = 0.1;
    double revolution_time = 5.0;
    
    bool debug_mode = false;
};

ProcessingConfig load_config(const std::string& config_file) {
    ProcessingConfig config;
    
    ConfigParser parser;
    if (!parser.load(config_file)) {
        std::cerr << "Warning: Cannot load config file " << config_file 
                  << ", using defaults\n";
        return config;
    }
    
    config.range_threshold_bins = parser.get_or_default("range_threshold_bins", 5);
    config.azimuth_threshold_bins = parser.get_or_default("azimuth_threshold_bins", 3);
    config.completion_gap_bins = parser.get_or_default("completion_gap_bins", 8);
    
    config.max_gate_distance_km = parser.get_or_default("max_gate_distance", 300.0) / 1000.0;
    config.max_gate_azimuth_deg = parser.get_or_default("max_gate_azimuth", 30.0);
    config.min_hits_to_confirm = parser.get_or_default("min_hits_to_confirm", 3);
    config.max_coast_count = parser.get_or_default("max_coast_count", 10);
    config.process_noise = parser.get_or_default("process_noise", 0.5);
    config.measurement_noise = parser.get_or_default("measurement_noise", 0.1);
    config.revolution_time = parser.get_or_default("revolution_time", 5.0);
    config.debug_mode = parser.get_or_default("tracking_debug", false);
    
    config.plots_file = parser.get_or_default<std::string>("plots_output_file", "");
    
    std::cout << "Loaded config from " << config_file << "\n";
    std::cout << "  Range threshold bins: " << config.range_threshold_bins << "\n";
    std::cout << "  Azimuth threshold bins: " << config.azimuth_threshold_bins << "\n";
    std::cout << "  Completion gap bins: " << config.completion_gap_bins << "\n";
    std::cout << "  Max gate distance: " << config.max_gate_distance_km << " km\n";
    std::cout << "  Max gate azimuth: " << config.max_gate_azimuth_deg << "°\n";
    std::cout << "  Min hits to confirm: " << config.min_hits_to_confirm << "\n";
    std::cout << "  Max coast count: " << config.max_coast_count << "\n";
    std::cout << "  Process noise: " << config.process_noise << "\n";
    std::cout << "  Measurement noise: " << config.measurement_noise << "\n";
    if (!config.plots_file.empty()) {
        std::cout << "  Plots output: " << config.plots_file << "\n";
    }
    
    return config;
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    std::string config_file = "../radar.conf";
    std::string input_file = "replies.txt";
    std::string tracks_file = "tracks_combined.txt";
    
    if (argc > 1) config_file = argv[1];
    if (argc > 2) input_file = argv[2];
    if (argc > 3) tracks_file = argv[3];
    
    std::cout << "=== Step 2+3: Combined Plot Formation and Track Processing ===\n";
    std::cout << "Config: " << config_file << "\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Tracks output: " << tracks_file << "\n\n";
    
    ProcessingConfig config = load_config(config_file);
    config.input_file = input_file;
    config.tracks_file = tracks_file;
    
    TrackerConfig tracker_config;
    tracker_config.min_hits_to_confirm = config.min_hits_to_confirm;
    tracker_config.max_coast_count = config.max_coast_count;
    tracker_config.max_gate_distance = config.max_gate_distance_km * 1000.0;
    tracker_config.max_gate_azimuth = config.max_gate_azimuth_deg;
    tracker_config.enable_rbs_tracking = true;
    tracker_config.enable_uvd_tracking = true;
    tracker_config.debug_mode = config.debug_mode;
    tracker_config.process_noise = config.process_noise;
    tracker_config.measurement_noise = config.measurement_noise;
    
    std::ifstream in(config.input_file);
    if (!in.is_open()) {
        std::cerr << "Error: Cannot open " << config.input_file << std::endl;
        return 1;
    }
    
    std::ofstream out_tracks(config.tracks_file);
    out_tracks << "# Tracks (from combined processing)\n";
    out_tracks << "# time_sec,track_id,x_km,y_km,speed_km_s,course_deg,code_data,altitude,altitude_valid,confidence,hit_count,state,type,code_reliable,alt_reliable\n";
    out_tracks << "# " << std::string(80, '-') << "\n";
    
    std::ofstream out_plots;
    bool write_plots = !config.plots_file.empty();
    if (write_plots) {
        out_plots.open(config.plots_file);
        if (out_plots.is_open()) {
            out_plots << "# Plots (from combined processing)\n";
            out_plots << "# time_sec,azimuth_deg,range_km,x_km,y_km,type,code_data,altitude,altitude_valid,altitude_attempts,spi,reply_count,garble_count,azimuth_span_deg,range_span_km,first_reply_time,last_reply_time\n";
            out_plots << "# " << std::string(80, '-') << "\n";
            std::cout << "Writing plots to: " << config.plots_file << "\n";
        } else {
            std::cerr << "Warning: Cannot open plots file: " << config.plots_file << std::endl;
            write_plots = false;
        }
    }
    
    OnlineClusterer rbs_clusterer(30.0, 
                                   config.range_threshold_bins, 
                                   config.azimuth_threshold_bins, 
                                   config.completion_gap_bins);
    OnlineClusterer uvd_clusterer(60.0, 
                                   config.range_threshold_bins * 2, 
                                   config.azimuth_threshold_bins * 2, 
                                   config.completion_gap_bins);
    
    TrackManager rbs_tracker(tracker_config);
    TrackManager uvd_tracker(tracker_config);
    
    std::map<uint64_t, int> last_hit_rbs;
    std::map<uint64_t, int> last_hit_uvd;
    std::map<uint64_t, PlotData> prev_rbs_plot;
    std::map<uint64_t, PlotData> prev_uvd_plot;
    
    std::string line;
    int line_num = 0;
    int rbs_replies_processed = 0;
    int uvd_replies_processed = 0;
    int plots_generated = 0;
    
    while (std::getline(in, line)) {
        line_num++;
        if (line_num % 1000 == 0) {
            std::cout << "\rProcessing line " << line_num << "..." << std::flush;
        }
        
        if (line.empty() || line[0] == '#') continue;
        
        Reply reply;
        if (!parse_reply_line(line, reply)) continue;
        
        if (reply.type == "SECTOR") {
            std::cout << "[MAIN] PROCESSING SECTOR at " << reply.azimuth << std::endl;            
            rbs_clusterer.process_sector(reply.azimuth);
            uvd_clusterer.process_sector(reply.azimuth);
            
            auto rbs_plots = rbs_clusterer.get_completed_plots();
            for (const auto& plot : rbs_plots) {
                if (write_plots && out_plots.is_open()) {
                    write_plots_to_file({plot}, out_plots);
                }
                process_plot_in_tracker(plot, rbs_tracker, last_hit_rbs, prev_rbs_plot, 
                                       out_tracks, "RBS", 0, config.revolution_time);
                plots_generated++;
            }
            
            auto uvd_plots = uvd_clusterer.get_completed_plots();
            for (const auto& plot : uvd_plots) {
                if (write_plots && out_plots.is_open()) {
                    write_plots_to_file({plot}, out_plots);
                }
                process_plot_in_tracker(plot, uvd_tracker, last_hit_uvd, prev_uvd_plot,
                                       out_tracks, "UVD", 1000, config.revolution_time);
                plots_generated++;
            }
            
        } else if (reply.type == "RBS_A" || reply.type == "RBS_C") {
            rbs_clusterer.add_reply(reply);
            rbs_replies_processed++;
            
            #if 0
            auto plots = rbs_clusterer.get_completed_plots();
            for (const auto& plot : plots) {
                if (write_plots && out_plots.is_open()) {
                    write_plots_to_file({plot}, out_plots);
                }
                process_plot_in_tracker(plot, rbs_tracker, last_hit_rbs, prev_rbs_plot,
                                       out_tracks, "RBS", 0, config.revolution_time);
                plots_generated++;
            }
            #endif
            
        } else if (reply.type == "UVD_DATA" || reply.type == "UVD_ALT") {
            uvd_clusterer.add_reply(reply);
            uvd_replies_processed++;
            
            #if 0
            auto plots = uvd_clusterer.get_completed_plots();
            for (const auto& plot : plots) {
                if (write_plots && out_plots.is_open()) {
                    write_plots_to_file({plot}, out_plots);
                }
                process_plot_in_tracker(plot, uvd_tracker, last_hit_uvd, prev_uvd_plot,
                                       out_tracks, "UVD", 1000, config.revolution_time);
                plots_generated++;
            }
            #endif
        }
    }
    
    std::cout << "\nFinishing remaining clusters...\n";
    
    rbs_clusterer.finish_all();
    auto final_rbs_plots = rbs_clusterer.get_final_plots();
    for (const auto& plot : final_rbs_plots) {
        if (write_plots && out_plots.is_open()) {
            write_plots_to_file({plot}, out_plots);
        }
        process_plot_in_tracker(plot, rbs_tracker, last_hit_rbs, prev_rbs_plot,
                               out_tracks, "RBS", 0, config.revolution_time);
        plots_generated++;
    }
    
    uvd_clusterer.finish_all();
    auto final_uvd_plots = uvd_clusterer.get_final_plots();
    for (const auto& plot : final_uvd_plots) {
        if (write_plots && out_plots.is_open()) {
            write_plots_to_file({plot}, out_plots);
        }
        process_plot_in_tracker(plot, uvd_tracker, last_hit_uvd, prev_uvd_plot,
                               out_tracks, "UVD", 1000, config.revolution_time);
        plots_generated++;
    }
    
    out_tracks.close();
    if (write_plots && out_plots.is_open()) {
        out_plots.close();
    }
    in.close();
    
    std::cout << "\nProcessed " << rbs_replies_processed << " RBS replies and " 
              << uvd_replies_processed << " UVD replies\n";
    std::cout << "Generated " << plots_generated << " plots\n";
    
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
    std::cout << "Tracks written to " << config.tracks_file << "\n";
    if (write_plots) {
        std::cout << "Plots written to " << config.plots_file << "\n";
    }
    
    return 0;
}
