// file: src/radar_system.cpp
#include "radar/radar_system.h"
#include "radar/utils.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace radar {

// Загрузка конфигурации из файла (упрощённая версия)
SystemConfig SystemConfig::load_from_file(const std::string& filename) {
    SystemConfig config;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open config file " << filename 
                  << ", using defaults\n";
        return config;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        if (std::getline(iss, key, '=')) {
            std::string value;
            std::getline(iss, value);
            
            // Парсинг параметров (упрощённо)
            if (key == "range_bin_rbs") config.radar.range_bin_rbs = std::stod(value);
            else if (key == "range_bin_uvd") config.radar.range_bin_uvd = std::stod(value);
            else if (key == "max_azimuth_diff_for_overlap") 
                config.radar.max_azimuth_diff_for_overlap = std::stod(value);
            else if (key == "max_range_diff_for_overlap") 
                config.radar.max_range_diff_for_overlap = std::stoi(value);
            else if (key == "min_amplitude") 
                config.radar.min_amplitude = std::stoi(value);
            else if (key == "max_gap_azimuth") 
                config.processing.max_gap_azimuth = std::stoi(value);
            else if (key == "range_window") 
                config.processing.range_window = std::stoi(value);
            else if (key == "range_tolerance") 
                config.processing.range_tolerance = std::stoi(value);
            else if (key == "min_hits") 
                config.processing.min_hits = std::stoi(value);
            else if (key == "output_file") 
                config.processing.output_file = value;
        }
    }
    
    return config;
}

void SystemConfig::save_to_file(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot create config file " << filename << "\n";
        return;
    }
    
    file << "# Radar configuration\n";
    file << "range_bin_rbs=" << radar.range_bin_rbs << "\n";
    file << "range_bin_uvd=" << radar.range_bin_uvd << "\n";
    file << "max_azimuth_diff_for_overlap=" << radar.max_azimuth_diff_for_overlap << "\n";
    file << "max_range_diff_for_overlap=" << radar.max_range_diff_for_overlap << "\n";
    file << "min_amplitude=" << static_cast<int>(radar.min_amplitude) << "\n";
    file << "\n# Processing configuration\n";
    file << "max_gap_azimuth=" << processing.max_gap_azimuth << "\n";
    file << "range_window=" << processing.range_window << "\n";
    file << "range_tolerance=" << processing.range_tolerance << "\n";
    file << "min_hits=" << processing.min_hits << "\n";
    file << "output_file=" << processing.output_file << "\n";
}

void RadarSystem::Statistics::print() const {
    std::cout << "\n=== Radar System Statistics ===\n";
    std::cout << "Scans processed: " << scans_processed << "\n";
    std::cout << "Clusters completed: " << clusters_completed << "\n";
    std::cout << "North markers: " << north_markers << "\n";
    std::cout << "Targets reported: " << targets_reported 
              << " (RBS:" << rbs_targets << ", UVD:" << uvd_targets << ")\n";
    std::cout << "Garbled targets: " << garbled_targets << "\n";
    std::cout << "============================\n";
}

RadarSystem::RadarSystem(const SystemConfig& config)
    : config_(config)
    , tracker_(config.processing.max_gap_azimuth, config.processing.range_window)
    , processor_(config.radar)
    , simulator_(std::make_unique<ReplySimulator>(config.simulator))
    , last_azimuth_(0) {
    
    // Настройка процессора
    processor_.set_range_tolerance(config.processing.range_tolerance);
    processor_.set_min_hits(config.processing.min_hits);
}

bool RadarSystem::initialize() {
    // Открываем выходной файл
    output_file_.open(config_.processing.output_file);
    if (!output_file_.is_open()) {
        std::cerr << "Error: Cannot open output file " 
                  << config_.processing.output_file << "\n";
        return false;
    }
    
    // Записываем заголовок
    output_file_ << "# Radar Target Reports\n";
    output_file_ << "# Format: timestamp,type,azimuth_deg,range_m,x,y,"
                 << "code/data,spi/emergency,altitude,signal_strength,garbled,sls_blanked\n";
    output_file_ << "# North marker: N,scan_number,azimuth,timestamp\n";
    output_file_ << "# " << std::string(80, '-') << "\n";
    
    std::cout << "Radar system initialized\n";
    std::cout << "Output file: " << config_.processing.output_file << "\n";
    
    return true;
}

void RadarSystem::process_scan(const ScanReplies& scan) {
    // Проверяем переход через Север: текущий азимут меньше предыдущего
    if (scan.azimuth < last_azimuth_) {
        // Произошёл переход через 0
        write_north_marker(scan.timestamp_ms, scan.azimuth);
        stats_.north_markers++;
    }
    
    // Передаём сканирование в трекер
    tracker_.process_scan(scan);
    stats_.scans_processed++;
    last_azimuth_ = scan.azimuth;
    scan_counter_++;
    
    // Получаем завершённые кластеры
    auto clusters = tracker_.get_completed_clusters();
    stats_.clusters_completed += clusters.size();
    
    // Обрабатываем каждый кластер
    for (const auto& cluster : clusters) {
        on_cluster_completed(cluster);
    }
}

void RadarSystem::on_cluster_completed(const TargetCluster& cluster) {
    static int cluster_count = 0;
    cluster_count++;
    
    // Обрабатываем кластер
    auto targets = processor_.process_cluster(cluster);
    
    // Сохраняем плоты во временный буфер
    for (const auto& target : targets) {
        pending_targets_.push_back(target);
    }
}

void RadarSystem::sort_and_flush_pending_targets() {
    if (pending_targets_.empty()) return;
    
    // Сортируем по азимуту
    std::sort(pending_targets_.begin(), pending_targets_.end(),
        [](const TargetReport& a, const TargetReport& b) {
            return a.azimuth_deg < b.azimuth_deg;
        });
    
    // Выводим все плоты
    for (const auto& target : pending_targets_) {
        // Обновляем статистику
        stats_.targets_reported++;
        if (target.type == TargetReport::SourceType::RBS) {
            stats_.rbs_targets++;
        } else {
            stats_.uvd_targets++;
        }
        if (target.is_garbled) {
            stats_.garbled_targets++;
        }
        
        // Выводим в файл
        write_target_to_file(target);
        
        // Вызываем callback, если есть
        if (target_callback_) {
            target_callback_(target);
        }
    }
    
    // Очищаем буфер
    pending_targets_.clear();
}

void RadarSystem::write_target_to_file(const TargetReport& target) {
    if (!output_file_.is_open()) return;
    
    // Время (в данном случае номер сканирования)
    output_file_ << scan_counter_ << ",";
    
    // Тип
    output_file_ << (target.type == TargetReport::SourceType::RBS ? "RBS" : "UVD") << ",";
    
    // Координаты
    output_file_ << std::fixed << std::setprecision(2)
                 << target.azimuth_deg << ","
                 << target.range_m << ","
                 << target.x << ","
                 << target.y << ",";
    
    // Специфическая информация
    if (target.type == TargetReport::SourceType::RBS) {
        output_file_ << "0x" << std::hex << target.rbs.mode3a_code << std::dec << ","
                     << (target.rbs.spi ? "1" : "0") << ","
                     << target.rbs.modec_altitude << ",";
    } else {
        output_file_ << "0x" << std::hex << target.uvd.raw_data20 << std::dec << ","
                     << (target.uvd.pressure_ref ? "1" : "0") << ","
                     << target.uvd.altitude << ",";
    }
    
    // Общие флаги
    output_file_ << static_cast<int>(target.signal_strength) << ","
                 << (target.is_garbled ? "1" : "0") << ","
                 << (target.is_sls_blanked ? "1" : "0") << "\n";
    
    // Сбрасываем буфер для гарантированной записи
    output_file_.flush();
}

void RadarSystem::write_north_marker(uint32_t timestamp, uint16_t azimuth) {
    if (!output_file_.is_open()) return;
    
    // Перед записью метки Севера сортируем и выводим все накопленные плоты
    sort_and_flush_pending_targets();
    
    output_file_ << "N," << scan_counter_ << "," << azimuth << "," << timestamp << "\n";
    output_file_.flush();
    
    std::cout << "\n>>> NORTH MARKER at scan " << scan_counter_ 
              << ", azimuth " << azimuth << " <<<\n";
}

void RadarSystem::shutdown() {
    // Принудительно закрываем все активные кластеры
    tracker_.reset();
    
    // Выводим оставшиеся плоты
    sort_and_flush_pending_targets();
    
    // Закрываем файл
    if (output_file_.is_open()) {
        output_file_.close();
    }
    
    // Выводим статистику
    stats_.print();
}

ScanReplies RadarSystem::generate_test_scan(uint16_t azimuth, uint32_t timestamp) {
    ScanReplies scan(azimuth, timestamp);
    
    // Несколько целей на разных дальностях с разными кодами
    
    // Цель 1: RBS, дальность 500, код 0x123 (появляется часто)
    if (azimuth % 3 == 0 || azimuth % 5 == 0) {
        auto rbs = simulator_->generate_rbs(azimuth, 500, 0x123, azimuth % 10 == 0);
        scan.rbs_replies.push_back(rbs);
    }
    
    // Цель 2: RBS, дальность 505, код 0x456 (появляется реже)
    if (azimuth % 4 == 0) {
        auto rbs = simulator_->generate_rbs(azimuth, 505, 0x456, false);
        scan.rbs_replies.push_back(rbs);
    }
    
    // Цель 3: RBS, дальность 600, код 0x789 (появляется периодически)
    if (azimuth % 6 == 0) {
        auto rbs = simulator_->generate_rbs(azimuth, 600, 0x789, false);
        scan.rbs_replies.push_back(rbs);
    }
    
    // Цель 4: УВД, дальность 800, данные 0x12345 (появляется часто)
    if (azimuth % 2 == 0) {
        auto uvd = simulator_->generate_uvd(azimuth, 800, 0x12345);
        scan.uvd_replies.push_back(uvd);
    }
    
    // Цель 5: УВД, дальность 805, данные 0x6789A (появляется реже)
    if (azimuth % 5 == 0) {
        auto uvd = simulator_->generate_uvd(azimuth, 805, 0x6789A);
        scan.uvd_replies.push_back(uvd);
    }
    
    return scan;
}

} // namespace radar