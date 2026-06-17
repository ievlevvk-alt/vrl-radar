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

// Структура для хранения ответа
struct Reply {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    std::string type;        // "RBS_A", "RBS_C", "NORTH", "SECTOR"
    uint32_t code_data;
    int altitude;
    bool spi;
    uint8_t sls_ratio;
    bool is_valid;           // Флаг валидности высоты
};

// Структура для хранения плота
struct Plot {
    double time_sec;              // Время формирования плота (середина)
    double azimuth_deg;           // Средний азимут
    double range_km;              // Средняя дальность
    double x_km;                  // X координата
    double y_km;                  // Y координата
    std::string type;             // "RBS"
    uint32_t code_data;           // Код цели
    int altitude;                 // Высота (из Mode C)
    bool altitude_valid;          // Флаг валидности высоты
    int altitude_attempts;        // Количество попыток получить высоту
    bool spi;                     // SPI
    int reply_count;              // Количество ответов
    double azimuth_span_deg;      // Разброс по азимуту
    double range_span_km;         // Разброс по дальности
    int sector_start;             // Начальный сектор
    int sector_end;               // Конечный сектор
    double first_reply_time;      // Время первого ответа
    double last_reply_time;       // Время последнего ответа
    uint16_t first_azimuth;       // Азимут первого ответа
    uint16_t last_azimuth;        // Азимут последнего ответа
};

// Класс для формирования плотов по азимутальному положению луча
class PlotFormer {
public:
    PlotFormer(int sectors_per_revolution = 32, double beamwidth_deg = 5.0) 
        : sectors_per_revolution_(sectors_per_revolution)
        , beamwidth_deg_(beamwidth_deg)
        , beamwidth_bins_(static_cast<uint16_t>(beamwidth_deg / 0.08789)) {
        
        std::cout << "Beamwidth: " << beamwidth_deg_ << "° (" << beamwidth_bins_ << " bins)\n";
    }
    
    // Обработка одного элемента
    void process_item(const Reply& reply) {
        // --- ОБРАБОТКА МАРКЕРОВ ---
        if (reply.type == "NORTH") {
            current_revolution_++;
            check_azimuth_timeouts(reply.azimuth);
            return;
        }
        
        if (reply.type == "SECTOR") {
            current_sector_ = reply.azimuth / (4096 / sectors_per_revolution_);
            check_azimuth_timeouts(reply.azimuth);
            return;
        }
        
        // --- ОБРАБОТКА ОТВЕТОВ (только RBS_A) ---
        if (reply.type != "RBS_A") return;
        
        uint32_t code = reply.code_data;
        
        if (pending_plots_.find(code) == pending_plots_.end()) {
            pending_plots_[code] = PendingPlot();
            pending_plots_[code].first_seen_time = reply.time_sec;
            pending_plots_[code].first_seen_azimuth = reply.azimuth;
        }
        
        auto& pending = pending_plots_[code];
        pending.replies.push_back(reply);
        pending.last_update_time = reply.time_sec;
        pending.last_update_azimuth = reply.azimuth;
        pending.sectors_covered.insert(current_sector_);
        pending.reply_count++;
        
        check_plot_completion(code);
    }
    
    // Проверка завершения плотов по азимуту
    void check_azimuth_timeouts(uint16_t current_azimuth) {
        std::vector<uint32_t> to_finish;
        
        for (const auto& [code, pending] : pending_plots_) {
            if (pending.replies.empty()) continue;
            
            uint16_t last_az = pending.last_update_azimuth;
            int16_t az_diff = static_cast<int16_t>(current_azimuth - last_az);
            
            if (az_diff < 0) az_diff += 4096;
            
            if (az_diff > beamwidth_bins_ * 2) {
                to_finish.push_back(code);
            }
        }
        
        for (uint32_t code : to_finish) {
            finish_plot(code);
        }
    }
    
    // Проверка конкретного плота на завершение
    void check_plot_completion(uint32_t code) {
        auto it = pending_plots_.find(code);
        if (it == pending_plots_.end()) return;
        
        auto& pending = it->second;
        if (pending.replies.size() < 2) return;
        
        uint16_t min_az = 4096, max_az = 0;
        for (const auto& reply : pending.replies) {
            min_az = std::min(min_az, reply.azimuth);
            max_az = std::max(max_az, reply.azimuth);
        }
        
        int16_t span = static_cast<int16_t>(max_az - min_az);
        if (span < 0) span += 4096;
        
        if (span > beamwidth_bins_ * 3) {
            finish_plot(code);
        }
    }
    
    // Завершить плот
    void finish_plot(uint32_t code) {
        auto it = pending_plots_.find(code);
        if (it == pending_plots_.end() || it->second.replies.empty()) return;
        
        auto& pending = it->second;
        
        if (pending.replies.size() >= 2) {
            Plot plot = create_plot(pending.replies);
            completed_plots_.push_back(plot);
        }
        
        pending_plots_.erase(it);
    }
    
    // Принудительно завершить все плоты
    void finish_all() {
        auto codes = std::vector<uint32_t>();
        for (const auto& [code, _] : pending_plots_) {
            codes.push_back(code);
        }
        for (uint32_t code : codes) {
            finish_plot(code);
        }
    }
    
    // Получить завершенные плоты
    std::vector<Plot> get_completed_plots() {
        auto result = std::move(completed_plots_);
        completed_plots_.clear();
        return result;
    }
    
private:
    struct PendingPlot {
        std::vector<Reply> replies;
        double first_seen_time{0.0};
        double last_update_time{0.0};
        uint16_t first_seen_azimuth{0};
        uint16_t last_update_azimuth{0};
        std::set<int> sectors_covered;
        int reply_count{0};
    };
    
    // Создание плота из накопленных ответов
    Plot create_plot(const std::vector<Reply>& replies) {
        Plot plot;
        plot.type = "RBS";
        plot.code_data = replies[0].code_data;
        plot.reply_count = replies.size();
        plot.spi = replies[0].spi;
        plot.altitude = 0;
        plot.altitude_valid = false;
        plot.altitude_attempts = 0;
        
        int min_sector = 999, max_sector = -1;
        uint16_t min_az = 4096, max_az = 0;
        
        for (const auto& r : replies) {
            int sector = r.azimuth / (4096 / sectors_per_revolution_);
            min_sector = std::min(min_sector, sector);
            max_sector = std::max(max_sector, sector);
            min_az = std::min(min_az, r.azimuth);
            max_az = std::max(max_az, r.azimuth);
        }
        plot.sector_start = min_sector;
        plot.sector_end = max_sector;
        plot.first_azimuth = min_az;
        plot.last_azimuth = max_az;
        
        double sum_az = 0.0;
        double sum_range = 0.0;
        double min_range = 1e9, max_range = 0;
        double first_time = replies[0].time_sec;
        double last_time = replies[0].time_sec;
        
        for (const auto& reply : replies) {
            double az_deg = reply.azimuth * 360.0 / 4096.0;
            sum_az += az_deg;
            sum_range += reply.range;
            min_range = std::min(min_range, static_cast<double>(reply.range));
            max_range = std::max(max_range, static_cast<double>(reply.range));
            first_time = std::min(first_time, reply.time_sec);
            last_time = std::max(last_time, reply.time_sec);
        }
        
        plot.time_sec = (first_time + last_time) / 2.0;
        plot.first_reply_time = first_time;
        plot.last_reply_time = last_time;
        
        plot.azimuth_deg = sum_az / replies.size();
        if (plot.azimuth_deg < 0) plot.azimuth_deg += 360.0;
        
        plot.range_km = (sum_range / replies.size()) * 0.03;
        
        int16_t az_span = static_cast<int16_t>(max_az - min_az);
        if (az_span < 0) az_span += 4096;
        plot.azimuth_span_deg = az_span * 360.0 / 4096.0;
        plot.range_span_km = (max_range - min_range) * 0.03;
        
        double az_rad = plot.azimuth_deg * M_PI / 180.0;
        plot.x_km = plot.range_km * std::sin(az_rad);
        plot.y_km = plot.range_km * std::cos(az_rad);
        
        // Ищем высоту из Mode C ответов
        double time_window = 0.1;
        int valid_altitude_count = 0;
        int invalid_altitude_count = 0;
        double altitude_sum = 0.0;
        
        for (const auto& reply : replies) {
            if (reply.type == "RBS_C") {
                double az_diff = std::abs(reply.azimuth * 360.0 / 4096.0 - plot.azimuth_deg);
                az_diff = std::min(az_diff, 360.0 - az_diff);
                double range_diff = std::abs(reply.range * 0.03 - plot.range_km);
                
                if (az_diff < 2.0 && range_diff < 1.0) {
                    if (reply.is_valid) {
                        valid_altitude_count++;
                        altitude_sum += reply.altitude;
                    } else {
                        invalid_altitude_count++;
                    }
                }
            }
        }
        
        plot.altitude_attempts = valid_altitude_count + invalid_altitude_count;
        
        if (valid_altitude_count > 0) {
            plot.altitude = static_cast<int>(altitude_sum / valid_altitude_count);
            plot.altitude_valid = true;
        } else if (invalid_altitude_count > 0) {
            plot.altitude = 0;
            plot.altitude_valid = false;
        } else {
            plot.altitude = 0;
            plot.altitude_valid = false;
        }
        
        return plot;
    }
    
    int sectors_per_revolution_;
    double beamwidth_deg_;
    uint16_t beamwidth_bins_;
    int current_sector_{-1};
    int current_revolution_{0};
    
    std::map<uint32_t, PendingPlot> pending_plots_;
    std::vector<Plot> completed_plots_;
};

int main(int argc, char* argv[]) {
    std::string input_file = "replies.txt";
    std::string output_file = "plots.txt";
    double beamwidth_deg = 5.0;
    
    if (argc > 1) input_file = argv[1];
    if (argc > 2) output_file = argv[2];
    if (argc > 3) beamwidth_deg = std::stod(argv[3]);
    
    std::cout << "=== Step 2: Form Plots ===\n";
    std::cout << "Input: " << input_file << "\n";
    std::cout << "Output: " << output_file << "\n";
    std::cout << "Beamwidth: " << beamwidth_deg << "°\n\n";
    
    // Читаем все элементы
    std::ifstream in(input_file);
    std::vector<Reply> items;
    std::string line;
    
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        
        std::stringstream ss(line);
        Reply reply;
        std::string spi_str;
        int sls;
        int is_valid;
        
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
        ss.ignore(1, ',');
        ss >> is_valid;
        reply.is_valid = (is_valid == 1);
        
        if (ss.fail()) {
            reply.is_valid = true;
            ss.clear();
        }
        
        items.push_back(reply);
    }
    in.close();
    
    std::cout << "Read " << items.size() << " items\n";
    
    int rbs_a = 0, rbs_c = 0, north = 0, sector = 0;
    for (const auto& r : items) {
        if (r.type == "RBS_A") rbs_a++;
        else if (r.type == "RBS_C") rbs_c++;
        else if (r.type == "NORTH") north++;
        else if (r.type == "SECTOR") sector++;
    }
    std::cout << "  RBS_A: " << rbs_a << ", RBS_C: " << rbs_c << "\n";
    std::cout << "  NORTH: " << north << ", SECTOR: " << sector << "\n\n";
    
    // Формируем плоты
    PlotFormer plot_former(32, beamwidth_deg);
    std::vector<Plot> all_plots;
    
    for (const auto& item : items) {
        plot_former.process_item(item);
        
        auto plots = plot_former.get_completed_plots();
        all_plots.insert(all_plots.end(), plots.begin(), plots.end());
    }
    
    plot_former.finish_all();
    auto final_plots = plot_former.get_completed_plots();
    all_plots.insert(all_plots.end(), final_plots.begin(), final_plots.end());
    
    // Записываем плоты
    std::ofstream out(output_file);
    out << "# Plots\n";
    out << "# time_sec,azimuth_deg,range_km,x_km,y_km,type,code_data,altitude,altitude_valid,altitude_attempts,spi,reply_count,azimuth_span_deg,range_span_km,sector_start,sector_end,first_reply_time,last_reply_time\n";
    out << "# " << std::string(80, '-') << "\n";
    
    for (const auto& plot : all_plots) {
        out << std::fixed << std::setprecision(6) << plot.time_sec << ","
            << std::setprecision(3) << plot.azimuth_deg << ","
            << plot.range_km << ","
            << plot.x_km << ","
            << plot.y_km << ","
            << plot.type << ","
            << plot.code_data << ","
            << plot.altitude << ","
            << (plot.altitude_valid ? "1" : "0") << ","
            << plot.altitude_attempts << ","
            << (plot.spi ? "1" : "0") << ","
            << plot.reply_count << ","
            << plot.azimuth_span_deg << ","
            << plot.range_span_km << ","
            << plot.sector_start << ","
            << plot.sector_end << ","
            << plot.first_reply_time << ","
            << plot.last_reply_time << "\n";
    }
    out.close();
    
    std::cout << "Generated " << all_plots.size() << " plots\n";
    return 0;
}
