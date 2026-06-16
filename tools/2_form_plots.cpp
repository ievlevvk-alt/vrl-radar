// tools/2_form_plots.cpp
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

struct Reply {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    std::string type;        // "RBS_A" или "RBS_C"
    uint32_t code_data;
    int altitude;
    bool spi;
    uint8_t sls_ratio;
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
    bool spi;
    int reply_count;
    double azimuth_span_deg;
    double range_span_km;
};

// Группировка ответов в плоты с учетом положения луча
std::vector<Plot> form_plots(const std::vector<Reply>& replies, 
                              double revolution_time = 5.0,
                              int azimuth_steps = 4096) {
    std::vector<Plot> plots;
    
    if (replies.empty()) return plots;
    
    // 1. Группируем по времени (окно = время одного дискрета азимута)
    double time_bin = revolution_time / azimuth_steps;
    
    // 2. Группируем по коду и временному окну
    struct Key {
        uint32_t code;
        int time_bin_idx;
        
        bool operator<(const Key& other) const {
            if (time_bin_idx != other.time_bin_idx) return time_bin_idx < other.time_bin_idx;
            return code < other.code;
        }
    };
    
    std::map<Key, std::vector<Reply>> groups;
    
    for (const auto& reply : replies) {
        // Только RBS_A (бортовые номера) для формирования плотов
        if (reply.type != "RBS_A") continue;
        
        Key key;
        key.code = reply.code_data;
        key.time_bin_idx = static_cast<int>(reply.time_sec / time_bin);
        groups[key].push_back(reply);
    }
    
    std::cout << "Found " << groups.size() << " groups by code and time\n";
    
    // 3. Для каждой группы формируем плот
    for (const auto& [key, group] : groups) {
        if (group.size() < 2) continue;  // Минимум 2 ответа для плота
        
        Plot plot;
        plot.time_sec = key.time_bin_idx * time_bin;
        plot.type = "RBS";
        plot.code_data = key.code;
        plot.reply_count = group.size();
        plot.spi = group[0].spi;
        plot.altitude = 0;  // Будет заполнено позже из Mode C
        
        // Вычисляем средние значения
        double sum_az = 0.0;
        double sum_range = 0.0;
        double min_az = 4096, max_az = 0;
        double min_range = 1e9, max_range = 0;
        
        for (const auto& reply : group) {
            double az_deg = reply.azimuth * 360.0 / 4096.0;
            sum_az += az_deg;
            sum_range += reply.range;
            min_az = std::min(min_az, az_deg);
            max_az = std::max(max_az, az_deg);
            min_range = std::min(min_range, static_cast<double>(reply.range));
            max_range = std::max(max_range, static_cast<double>(reply.range));
        }
        
        plot.azimuth_deg = sum_az / group.size();
        plot.range_km = (sum_range / group.size()) * 0.03;  // 30 м на дискрет
        
        // Разброс
        plot.azimuth_span_deg = max_az - min_az;
        plot.range_span_km = (max_range - min_range) * 0.03;
        
        // Декартовы координаты
        double az_rad = plot.azimuth_deg * M_PI / 180.0;
        plot.x_km = plot.range_km * std::sin(az_rad);
        plot.y_km = plot.range_km * std::cos(az_rad);
        
        // Ищем соответствующую высоту из Mode C ответов
        // (ответы с высотой приходят в том же временном окне)
        for (const auto& reply : replies) {
            if (reply.type == "RBS_C") {
                double time_diff = std::abs(reply.time_sec - plot.time_sec);
                if (time_diff < time_bin * 2) {
                    // Проверяем, что это та же цель (по азимуту и дальности)
                    double az_diff = std::abs(reply.azimuth * 360.0 / 4096.0 - plot.azimuth_deg);
                    az_diff = std::min(az_diff, 360.0 - az_diff);
                    double range_diff = std::abs(reply.range * 0.03 - plot.range_km);
                    
                    if (az_diff < 2.0 && range_diff < 1.0) {
                        plot.altitude = reply.altitude;
                        break;
                    }
                }
            }
        }
        
        plots.push_back(plot);
    }
    
    // Сортируем по времени
    std::sort(plots.begin(), plots.end(), 
              [](const Plot& a, const Plot& b) { return a.time_sec < b.time_sec; });
    
    return plots;
}

int main(int argc, char* argv[]) {
    std::string input_file = "replies.txt";
    std::string output_file = "plots.txt";
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    
    std::cout << "=== Step 2: Form Plots ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n\n";
    
    // Читаем ответы
    std::ifstream in(input_file);
    std::vector<Reply> replies;
    std::string line;
    
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        Reply reply;
        std::string spi_str;
        int sls;
        
        ss >> reply.time_sec;
        ss.ignore(1, ',');
        ss >> reply.azimuth;
        ss.ignore(1, ',');
        ss >> reply.range;
        ss.ignore(1, ',');
        ss >> reply.type;
        ss.ignore(1, ',');
        ss >> reply.code_data;
        ss.ignore(1, ',');
        ss >> reply.altitude;
        ss.ignore(1, ',');
        ss >> spi_str;
        reply.spi = (spi_str == "1");
        ss.ignore(1, ',');
        ss >> sls;
        reply.sls_ratio = static_cast<uint8_t>(sls);
        
        replies.push_back(reply);
    }
    in.close();
    
    std::cout << "Read " << replies.size() << " replies\n";
    
    // Подсчет типов
    int rbs_a = 0, rbs_c = 0;
    for (const auto& r : replies) {
        if (r.type == "RBS_A") rbs_a++;
        else if (r.type == "RBS_C") rbs_c++;
    }
    std::cout << "  RBS_A: " << rbs_a << ", RBS_C: " << rbs_c << "\n";
    
    // Формируем плоты
    auto plots = form_plots(replies);
    
    // Записываем плоты
    std::ofstream out(output_file);
    out << "# Plots\n";
    out << "# time_sec,azimuth_deg,range_km,x_km,y_km,type,code_data,altitude,spi,reply_count,azimuth_span_deg,range_span_km\n";
    
    for (const auto& plot : plots) {
        out << std::fixed << std::setprecision(6) << plot.time_sec << ","
            << std::setprecision(3) << plot.azimuth_deg << ","
            << plot.range_km << ","
            << plot.x_km << ","
            << plot.y_km << ","
            << plot.type << ","
            << plot.code_data << ","
            << plot.altitude << ","
            << (plot.spi ? "1" : "0") << ","
            << plot.reply_count << ","
            << plot.azimuth_span_deg << ","
            << plot.range_span_km << "\n";
    }
    out.close();
    
    std::cout << "Generated " << plots.size() << " plots\n";
    return 0;
}
