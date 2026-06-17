// tools/2_form_plots.cpp
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <set>

// Константы для преобразования
constexpr double AZIMUTH_BIN_DEG = 360.0 / 4096.0;  // 0.087890625°

struct Reply {
    double time_sec;
    uint16_t azimuth;      // в МАИ (0-4095)
    uint16_t range;        // в дискретах
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

struct Plot {
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

// Кластеризатор, работающий в МАИ и дискретах
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
        , current_azimuth_(0) {
        
        std::cout << "  Clusterer: range_bin=" << range_bin_m << "m"
                  << ", range_threshold=" << range_threshold_bins << " bins"
                  << ", azimuth_threshold=" << azimuth_threshold_bins << " MAI"
                  << ", completion_gap=" << completion_gap_bins << " MAI\n";
    }
    
    void update_azimuth(uint16_t azimuth) {
        current_azimuth_ = azimuth;
        check_completed_clusters();
    }
    
    void add_reply(const Reply& reply) {
        check_completed_clusters();
        
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
    }
    
    std::vector<Plot> get_completed_plots() {
        auto result = std::move(completed_plots_);
        completed_plots_.clear();
        return result;
    }
    
    void finish_all() {
        for (auto& cluster : active_clusters_) {
            if (cluster.replies.size() >= 2) {
                completed_plots_.push_back(create_plot(cluster));
            }
        }
        active_clusters_.clear();
    }
    
    size_t active_clusters_count() const {
        return active_clusters_.size();
    }
    
private:
    struct Cluster {
        std::vector<Reply> replies;
        uint16_t min_azimuth{4096};
        uint16_t max_azimuth{0};
        uint16_t min_range{65535};
        uint16_t max_range{0};
        uint16_t last_azimuth{0};      // последний азимут в кластере
        uint16_t last_range{0};        // последняя дальность в кластере
        double first_time{0};
        double last_time{0};
        double center_x{0};
        double center_y{0};
        bool has_center{false};
        int garble_count{0};
        
        // Статистика по кодам и высотам
        std::map<uint32_t, int> code_counts;
        std::map<int, int> altitude_counts;
        uint32_t best_code{0};
        int best_altitude{0};
        int best_code_count{0};
        int best_altitude_count{0};
        
        void add_reply(const Reply& r) {
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
            
            // Собираем статистику по кодам (номер борта)
            if (r.type == "UVD_DATA" || r.type == "RBS_A") {
                code_counts[r.code_data]++;
                if (code_counts[r.code_data] > best_code_count) {
                    best_code_count = code_counts[r.code_data];
                    best_code = r.code_data;
                }
            }
            
            // Собираем статистику по высоте
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
        
        // Проверка близости к ПОСЛЕДНЕМУ ответу в кластере
        bool is_near(const Reply& r, int range_threshold_bins, int azimuth_threshold_bins) const {
            if (replies.empty()) return true;
            
            // Проверяем по дальности (от последнего ответа)
            int range_diff = std::abs(static_cast<int>(r.range) - static_cast<int>(last_range));
            if (range_diff > range_threshold_bins) return false;
            
            // Проверяем по азимуту (от последнего ответа, с учётом цикличности)
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
        std::vector<int> to_remove;
        
        for (size_t i = 0; i < active_clusters_.size(); ++i) {
            auto& cluster = active_clusters_[i];
            if (cluster.replies.empty()) continue;
            
            if (cluster.is_completed(current_azimuth_, completion_gap_bins_)) {
                if (cluster.replies.size() >= 2) {
                    completed_plots_.push_back(create_plot(cluster));
                }
                to_remove.push_back(i);
            }
        }
        
        for (int i = static_cast<int>(to_remove.size()) - 1; i >= 0; --i) {
            int idx = to_remove[i];
            active_clusters_.erase(active_clusters_.begin() + idx);
        }
    }
    
    Plot create_plot(const Cluster& cluster) {
        Plot plot;
        
        // Тип плота - определяем по типу ответов в кластере
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
        
        // Выбираем наилучший номер борта
        if (cluster.best_code != 0) {
            plot.code_data = cluster.best_code;
        } else {
            plot.code_data = cluster.replies[0].code_data;
        }
        
        // Выбираем наилучшую высоту
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
        
        // Вычисляем средний азимут (в МАИ) с учётом цикличности
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
        
        plot.azimuth_deg = avg_az_rad * 180.0 / M_PI;
        
        double avg_range_bins = sum_range_bins / cluster.replies.size();
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
    std::vector<Plot> completed_plots_;
};

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

int main(int argc, char* argv[]) {
    std::string input_file = "replies.txt";
    std::string output_file = "plots.txt";
    
    int range_threshold_bins = 5;
    int azimuth_threshold_bins = 3;
    int completion_gap_bins = 8;
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    if (argc > 3) range_threshold_bins = std::stoi(argv[3]);
    if (argc > 4) azimuth_threshold_bins = std::stoi(argv[4]);
    if (argc > 5) completion_gap_bins = std::stoi(argv[5]);
    
    std::cout << "=== Step 2: Form Plots (Online Clustering) ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Range threshold: " << range_threshold_bins << " bins\n";
    std::cout << "Azimuth threshold: " << azimuth_threshold_bins << " MAI\n";
    std::cout << "Completion gap: " << completion_gap_bins << " MAI\n\n";
    
    std::ifstream in(input_file);
    if (!in.is_open()) {
        std::cerr << "Error: Cannot open " << input_file << std::endl;
        return 1;
    }
    
    std::cout << "Initializing RBS clusterer...\n";
    OnlineClusterer rbs_clusterer(30.0, range_threshold_bins, azimuth_threshold_bins, completion_gap_bins);
    
    std::cout << "Initializing UVD clusterer...\n";
    OnlineClusterer uvd_clusterer(60.0, range_threshold_bins * 2, azimuth_threshold_bins * 2, completion_gap_bins);
    std::cout << std::endl;
    
    std::string line;
    int line_num = 0;
    int rbs_replies_processed = 0;
    int uvd_replies_processed = 0;
    std::vector<Plot> all_plots;
    
    while (std::getline(in, line)) {
        line_num++;
        if (line_num % 1000 == 0) {
            std::cout << "\rProcessing line " << line_num << "..." << std::flush;
        }
        
        if (line.empty() || line[0] == '#') continue;
        
        Reply reply;
        if (!parse_reply_line(line, reply)) continue;
        
        if (reply.type == "SECTOR") {
            rbs_clusterer.process_sector(reply.azimuth);
            uvd_clusterer.process_sector(reply.azimuth);
            
            auto plots = rbs_clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), plots.begin(), plots.end());
            
            auto uvd_plots = uvd_clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), uvd_plots.begin(), uvd_plots.end());
            
        } else if (reply.type == "RBS_A" || reply.type == "RBS_C") {
            rbs_clusterer.add_reply(reply);
            rbs_replies_processed++;
            
            auto plots = rbs_clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), plots.begin(), plots.end());
            
        } else if (reply.type == "UVD_DATA" || reply.type == "UVD_ALT") {
            uvd_clusterer.add_reply(reply);
            uvd_replies_processed++;
            
            auto plots = uvd_clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), plots.begin(), plots.end());
        }
    }
    
    std::cout << "\nFinishing remaining clusters...\n";
    
    rbs_clusterer.finish_all();
    auto final_rbs_plots = rbs_clusterer.get_completed_plots();
    all_plots.insert(all_plots.end(), final_rbs_plots.begin(), final_rbs_plots.end());
    
    uvd_clusterer.finish_all();
    auto final_uvd_plots = uvd_clusterer.get_completed_plots();
    all_plots.insert(all_plots.end(), final_uvd_plots.begin(), final_uvd_plots.end());
    
    std::cout << "\nProcessed " << rbs_replies_processed << " RBS replies and " 
              << uvd_replies_processed << " UVD replies\n";
    std::cout << "Generated " << all_plots.size() << " plots total\n";
    
    std::sort(all_plots.begin(), all_plots.end(),
        [](const Plot& a, const Plot& b) {
            return a.time_sec < b.time_sec;
        });
    
    std::ofstream out(output_file);
    if (!out.is_open()) {
        std::cerr << "Error: Cannot open " << output_file << std::endl;
        return 1;
    }
    
    out << "# Plots\n";
    out << "# time_sec,azimuth_deg,range_km,x_km,y_km,type,code_data,altitude,altitude_valid,altitude_attempts,spi,reply_count,garble_count,azimuth_span_deg,range_span_km,first_reply_time,last_reply_time\n";
    out << "# " << std::string(80, '-') << "\n";
    
    for (const auto& plot : all_plots) {
        // Форматируем код в зависимости от типа
        std::string code_str;
        if (plot.type == "UVD") {
            // УВД - десятичный
            code_str = std::to_string(plot.code_data);
        } else {
            // RBS - восьмеричный с ведущим нулём
            std::stringstream ss;
            ss << std::oct << plot.code_data;
            code_str = ss.str();
            while (code_str.length() < 4) code_str = "0" + code_str;
            code_str = "0" + code_str;
        }
        
        out << std::fixed << std::setprecision(6) << plot.time_sec << ","
            << std::setprecision(3) << plot.azimuth_deg << ","
            << plot.range_km << ","
            << plot.x_km << ","
            << plot.y_km << ","
            << plot.type << ","
            << code_str << ","
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

    out.close();
    
    int rbs_plots = 0, uvd_plots = 0;
    for (const auto& p : all_plots) {
        if (p.type == "RBS" || p.type == "RBS_A" || p.type == "RBS_C") rbs_plots++;
        else if (p.type == "UVD" || p.type == "UVD_DATA" || p.type == "UVD_ALT") uvd_plots++;
    }
    
    std::cout << "RBS plots: " << rbs_plots << ", UVD plots: " << uvd_plots << "\n";
    std::cout << "Output written to " << output_file << "\n";
    
    return 0;
}
