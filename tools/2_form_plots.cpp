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
    bool is_garble;      // Искажен/перекрыт
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
    int garble_count;    // Количество искаженных ответов в плоте
    double azimuth_span_deg;
    double range_span_km;
    double first_reply_time;
    double last_reply_time;
};

class OnlineClusterer {
public:
    OnlineClusterer(double range_bin_m = 30.0, double azimuth_bin_deg = 0.08789,
                    double range_threshold_km = 1.5, double azimuth_threshold_deg = 3.0,
                    double completion_azimuth_deg = 10.0)
        : range_bin_m_(range_bin_m)
        , azimuth_bin_deg_(azimuth_bin_deg)
        , range_threshold_m_(range_threshold_km * 1000.0)
        , azimuth_threshold_deg_(azimuth_threshold_deg)
        , completion_azimuth_deg_(completion_azimuth_deg)
        , current_azimuth_(0) {
        
        std::cout << "Clustering: range_threshold=" << range_threshold_km << " km"
                  << ", azimuth_threshold=" << azimuth_threshold_deg << "°"
                  << ", completion_azimuth=" << completion_azimuth_deg << "°\n";
    }
    
    void update_azimuth(uint16_t azimuth) {
        current_azimuth_ = azimuth;
        check_completed_clusters();
    }
    
    void add_reply(const Reply& reply) {
        check_completed_clusters();
        
        if (reply.type == "RBS_A") {
            double az_rad = (reply.azimuth * azimuth_bin_deg_) * M_PI / 180.0;
            double range_m = reply.range * range_bin_m_;
            
            Reply r = reply;
            r.x = range_m * sin(az_rad);
            r.y = range_m * cos(az_rad);
            
            bool added = false;
            for (auto& cluster : active_clusters_) {
                if (cluster.is_near(r, range_threshold_m_, azimuth_threshold_deg_,
                                    range_bin_m_, azimuth_bin_deg_)) {
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
        } else if (reply.type == "RBS_C") {
            mode_c_replies_.push_back(reply);
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
        mode_c_replies_.clear();
    }
    
private:
    struct Cluster {
        std::vector<Reply> replies;
        double min_azimuth{4096};
        double max_azimuth{0};
        double min_range{1e9};
        double max_range{0};
        double first_time{0};
        double last_time{0};
        double center_x{0};
        double center_y{0};
        bool has_center{false};
        int last_azimuth{0};
        uint32_t primary_code{0};
        std::map<uint32_t, int> code_counts;
        int garble_count{0};
        
        void add_reply(const Reply& r) {
            if (replies.empty()) {
                first_time = r.time_sec;
                center_x = r.x;
                center_y = r.y;
                has_center = true;
                primary_code = r.code_data;
            }
            replies.push_back(r);
            if (r.is_garble) garble_count++;
            
            last_time = r.time_sec;
            last_azimuth = r.azimuth;
            min_azimuth = std::min(min_azimuth, static_cast<double>(r.azimuth));
            max_azimuth = std::max(max_azimuth, static_cast<double>(r.azimuth));
            min_range = std::min(min_range, static_cast<double>(r.range));
            max_range = std::max(max_range, static_cast<double>(r.range));
            
            code_counts[r.code_data]++;
            if (code_counts[r.code_data] > code_counts[primary_code]) {
                primary_code = r.code_data;
            }
            
            if (has_center) {
                double alpha = 1.0 / replies.size();
                center_x = center_x * (1 - alpha) + r.x * alpha;
                center_y = center_y * (1 - alpha) + r.y * alpha;
            }
        }
        
        bool is_near(const Reply& r, double range_threshold_m, double azimuth_threshold_deg,
                     double range_bin_m, double azimuth_bin_deg) const {
            if (replies.empty()) return true;
            
            if (has_center) {
                double dx = r.x - center_x;
                double dy = r.y - center_y;
                double dist = sqrt(dx*dx + dy*dy);
                if (dist < range_threshold_m * 0.5) {
                    return true;
                }
            }
            
            for (const auto& existing : replies) {
                double az_diff = std::abs(static_cast<double>(r.azimuth) - existing.azimuth);
                az_diff = std::min(az_diff, 4096.0 - az_diff);
                double az_deg = az_diff * azimuth_bin_deg;
                
                double range_diff = std::abs(static_cast<double>(r.range) - existing.range);
                double range_m = range_diff * range_bin_m;
                
                if (az_deg < azimuth_threshold_deg && range_m < range_threshold_m) {
                    return true;
                }
            }
            
            return false;
        }
        
        bool is_completed(uint16_t current_azimuth, double azimuth_bin_deg, 
                          double completion_azimuth_deg) const {
            if (replies.empty()) return false;
            
            int16_t az_diff = static_cast<int16_t>(current_azimuth - last_azimuth);
            if (az_diff < 0) az_diff += 4096;
            double az_deg = az_diff * azimuth_bin_deg;
            
            return az_deg > completion_azimuth_deg;
        }
    };
    
    void check_completed_clusters() {
        std::vector<int> to_remove;
        
        for (size_t i = 0; i < active_clusters_.size(); ++i) {
            auto& cluster = active_clusters_[i];
            if (cluster.replies.empty()) continue;
            
            if (cluster.is_completed(current_azimuth_, azimuth_bin_deg_, 
                                     completion_azimuth_deg_)) {
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
    
    int find_altitude_for_plot(const Plot& plot) {
        int best_altitude = 0;
        int best_count = 0;
        std::map<int, int> altitude_counts;
        
        for (const auto& reply : mode_c_replies_) {
            // Пропускаем искаженные ответы
            if (reply.is_garble) continue;
            
            double time_diff = std::abs(reply.time_sec - plot.time_sec);
            if (time_diff > 0.5) continue;
            
            double az_diff = std::abs(reply.azimuth * azimuth_bin_deg_ - plot.azimuth_deg);
            az_diff = std::min(az_diff, 360.0 - az_diff);
            if (az_diff > 5.0) continue;
            
            double range_km = reply.range * range_bin_m_ / 1000.0;
            double range_diff = std::abs(range_km - plot.range_km);
            if (range_diff > 2.0) continue;
            
            // is_valid может быть false при garble, но мы уже отфильтровали garble
            if (reply.is_valid && reply.altitude > 0) {
                altitude_counts[reply.altitude]++;
                if (altitude_counts[reply.altitude] > best_count) {
                    best_count = altitude_counts[reply.altitude];
                    best_altitude = reply.altitude;
                }
            }
        }
        
        return best_altitude;
    }
    
    Plot create_plot(const Cluster& cluster) {
        Plot plot;
        plot.type = "RBS";
        plot.code_data = cluster.primary_code;
        plot.reply_count = cluster.replies.size();
        plot.garble_count = cluster.garble_count;
        plot.spi = cluster.replies[0].spi;
        plot.altitude = 0;
        plot.altitude_valid = false;
        plot.altitude_attempts = 0;
        
        double sum_az = 0.0;
        double sum_range = 0.0;
        double first_time = cluster.replies[0].time_sec;
        double last_time = cluster.replies[0].time_sec;
        
        for (const auto& reply : cluster.replies) {
            double az_deg = reply.azimuth * azimuth_bin_deg_;
            sum_az += az_deg;
            sum_range += reply.range;
            first_time = std::min(first_time, reply.time_sec);
            last_time = std::max(last_time, reply.time_sec);
        }
        
        plot.time_sec = (first_time + last_time) / 2.0;
        plot.first_reply_time = first_time;
        plot.last_reply_time = last_time;
        
        plot.azimuth_deg = sum_az / cluster.replies.size();
        plot.range_km = (sum_range / cluster.replies.size()) * range_bin_m_ / 1000.0;
        
        plot.azimuth_span_deg = (cluster.max_azimuth - cluster.min_azimuth) * azimuth_bin_deg_;
        plot.range_span_km = (cluster.max_range - cluster.min_range) * range_bin_m_ / 1000.0;
        
        double az_rad = plot.azimuth_deg * M_PI / 180.0;
        plot.x_km = plot.range_km * sin(az_rad);
        plot.y_km = plot.range_km * cos(az_rad);
        
        // Ищем высоту (игнорируем garble)
        int altitude = find_altitude_for_plot(plot);
        if (altitude > 0) {
            plot.altitude = altitude;
            plot.altitude_valid = true;
            plot.altitude_attempts = 1;
        }
        
        return plot;
    }
    
    double range_bin_m_;
    double azimuth_bin_deg_;
    double range_threshold_m_;
    double azimuth_threshold_deg_;
    double completion_azimuth_deg_;
    
    uint16_t current_azimuth_{0};
    
    std::vector<Cluster> active_clusters_;
    std::vector<Reply> mode_c_replies_;
    std::vector<Plot> completed_plots_;
};

bool parse_reply_line(const std::string& line, Reply& reply) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() < 10) return false;  // Теперь 10 полей
    
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
    double range_threshold_km = 1.5;
    double azimuth_threshold_deg = 3.0;
    double completion_azimuth_deg = 10.0;
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    if (argc > 3) range_threshold_km = std::stod(argv[3]);
    if (argc > 4) azimuth_threshold_deg = std::stod(argv[4]);
    if (argc > 5) completion_azimuth_deg = std::stod(argv[5]);
    
    std::cout << "=== Step 2: Form Plots (Online Clustering) ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Range threshold: " << range_threshold_km << " km\n";
    std::cout << "Azimuth threshold: " << azimuth_threshold_deg << "°\n";
    std::cout << "Completion azimuth: " << completion_azimuth_deg << "°\n\n";
    
    std::ifstream in(input_file);
    OnlineClusterer clusterer(30.0, 0.08789, range_threshold_km, azimuth_threshold_deg,
                              completion_azimuth_deg);
    std::string line;
    int line_num = 0;
    int replies_processed = 0;
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
            clusterer.process_sector(reply.azimuth);
            auto plots = clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), plots.begin(), plots.end());
        } else if (reply.type == "RBS_A" || reply.type == "RBS_C") {
            clusterer.add_reply(reply);
            replies_processed++;
            
            auto plots = clusterer.get_completed_plots();
            all_plots.insert(all_plots.end(), plots.begin(), plots.end());
        }
    }
    
    std::cout << "\nFinishing remaining clusters...\n";
    clusterer.finish_all();
    auto final_plots = clusterer.get_completed_plots();
    all_plots.insert(all_plots.end(), final_plots.begin(), final_plots.end());
    
    std::cout << "Processed " << replies_processed << " replies (RBS_A + RBS_C)\n";
    std::cout << "Generated " << all_plots.size() << " plots\n";
    
    std::ofstream out(output_file);
    out << "# Plots\n";
    out << "# time_sec,azimuth_deg,range_km,x_km,y_km,type,code_data,altitude,altitude_valid,altitude_attempts,spi,reply_count,garble_count,azimuth_span_deg,range_span_km,first_reply_time,last_reply_time\n";
    out << "# " << std::string(80, '-') << "\n";
    
    for (const auto& plot : all_plots) {
        out << std::fixed << std::setprecision(6) << plot.time_sec << ","
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
    out.close();
    
    return 0;
}
