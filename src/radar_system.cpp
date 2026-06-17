// file: src/radar_system.cpp
#include "radar/radar_system.h"
#include "radar/utils.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace radar {

// ============================================================================
// SystemConfig implementation
// ============================================================================




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
    file << "\n# Tracker configuration\n";
    file << "min_hits_to_confirm=" << tracker.min_hits_to_confirm << "\n";
    file << "max_coast_count=" << tracker.max_coast_count << "\n";
    file << "max_gate_distance=" << tracker.max_gate_distance << "\n";
    file << "max_gate_azimuth=" << tracker.max_gate_azimuth << "\n";
    file << "tracking_debug=" << (tracker.debug_mode ? "true" : "false") << "\n";
}

// ============================================================================
// Statistics implementation
// ============================================================================

void RadarSystem::Statistics::print() const {
    std::cout << "\n=== Radar System Statistics ===\n";
    std::cout << "Revolutions processed: " << scans_processed / 2048 << "\n";
    std::cout << "Scans processed: " << scans_processed << "\n";
    std::cout << "Clusters completed: " << clusters_completed << "\n";
    std::cout << "North markers: " << north_markers << "\n";
    std::cout << "Targets reported: " << targets_reported 
              << " (RBS:" << rbs_targets << ", UVD:" << uvd_targets << ")\n";
    std::cout << "Garbled targets: " << garbled_targets << "\n";
    std::cout << "Active tracks: " << active_tracks << "\n";
    std::cout << "============================\n";
}

// ============================================================================
// RadarSystem implementation
// ============================================================================

RadarSystem::RadarSystem(const SystemConfig& config)
    : config_(config)
    , tracker_(config.processing.max_gap_azimuth, config.processing.range_window)
    , processor_(config.radar)
    , simulator_(std::make_unique<ReplySimulator>(config.simulator))
    , last_azimuth_(0) {
    
    processor_.set_range_tolerance(config.processing.range_tolerance);
    processor_.set_min_hits(config.processing.min_hits);
}

bool RadarSystem::initialize() {
    output_file_.open(config_.processing.output_file);
    if (!output_file_.is_open()) {
        std::cerr << "Error: Cannot open output file " 
                  << config_.processing.output_file << "\n";
        return false;
    }
    
    // Заголовок файла
    output_file_ << "# Radar Target Reports and Tracks\n";
    output_file_ << "# Format:\n";
    output_file_ << "#   TARGET: R,revolution,type,azimuth_deg,range_m,x,y,code,altitude\n";
    output_file_ << "#   TRACK:  T,revolution,track_id,state,x,y,speed,course,code,altitude,confidence,hits\n";
    output_file_ << "#   NORTH:  N,revolution,azimuth\n";
    output_file_ << "# " << std::string(80, '-') << "\n";
    
    track_manager_ = std::make_unique<TrackManager>(config_.tracker);
    revolution_targets_.clear();
    revolution_targets_.reserve(500);
    
    std::cout << "Radar system initialized\n";
    std::cout << "Output file: " << config_.processing.output_file << "\n";
    std::cout << "Tracking: " << (tracking_enabled_ ? "enabled" : "disabled") << "\n";
    
    return true;
}


// file: src/radar_system.cpp - добавить функцию enable_input_logging

void RadarSystem::enable_input_logging(bool enable, const std::string& filename) {
    if (enable) {
        input_log_file_.open(filename);
        if (input_log_file_.is_open()) {
            input_log_file_ << "# Input Radar Replies Log\n";
            input_log_file_ << "# Format: revolution,azimuth,type,range,code/data,sls_ratio\n";
            input_log_file_ << "# Columns:\n";
            input_log_file_ << "#   revolution - current revolution number\n";
            input_log_file_ << "#   azimuth - azimuth bin (0-4095)\n";
            input_log_file_ << "#   type - RBS or UVD\n";
            input_log_file_ << "#   range - range bin\n";
            input_log_file_ << "#   code/data - RBS code or UVD data\n";
            input_log_file_ << "#   sls_ratio - SLS/Main amplitude ratio (for sidelobe detection)\n";
            std::cout << "Input logging enabled to: " << filename << "\n";
        } else {
            std::cerr << "Warning: Cannot open input log file: " << filename << "\n";
        }
    } else {
        if (input_log_file_.is_open()) {
            input_log_file_.close();
            std::cout << "Input logging disabled\n";
        }
    }
}

// Модифицировать process_scan для логирования
void RadarSystem::process_scan(const ScanReplies& scan) {
    // Проверяем переход через Север
    if (scan.azimuth < last_azimuth_) {
        end_of_revolution();
    }
    
    // Логирование входных ответов
    if (input_log_file_.is_open()) {
        for (const auto& reply : scan.rbs_replies) {
            double sls_ratio = (reply.ether_amplitudes_sls[0] > 0) ? 
                static_cast<double>(reply.ether_amplitudes_sls[0]) / reply.ether_amplitudes[0] : 0;
            input_log_file_ << current_revolution_ << ","
                           << scan.azimuth << ",RBS,"
                           << reply.range << ",0x" << std::hex << reply.code12 << std::dec << ","
                           << sls_ratio << "\n";
        }
        for (const auto& reply : scan.uvd_replies) {
            double sls_ratio = (reply.ether_amplitudes_sls[0] > 0) ? 
                static_cast<double>(reply.ether_amplitudes_sls[0]) / reply.ether_amplitudes[0] : 0;
            input_log_file_ << current_revolution_ << ","
                           << scan.azimuth << ",UVD,"
                           << reply.range << ",0x" << std::hex << reply.data20 << std::dec << ","
                           << sls_ratio << "\n";
        }
        input_log_file_.flush();
    }
    
    tracker_.process_scan(scan);
    stats_.scans_processed++;
    last_azimuth_ = scan.azimuth;
    scan_counter_++;
    
    auto clusters = tracker_.get_completed_clusters();
    stats_.clusters_completed += clusters.size();
    
    for (const auto& cluster : clusters) {
        auto targets = processor_.process_cluster(cluster);
        for (const auto& target : targets) {
            revolution_targets_.push_back(target);
        }
    }
}





void RadarSystem::shutdown() {
    // Обрабатываем последний оборот
    if (!revolution_targets_.empty()) {
        process_revolution_complete();
    }
    
    tracker_.reset();
    
    if (output_file_.is_open()) {
        output_file_.close();
    }
    
    stats_.print();
}

std::vector<Track> RadarSystem::get_tracks() const {
    if (!track_manager_) return {};
    return track_manager_->get_active_tracks();
}

ScanReplies RadarSystem::generate_test_scan(uint16_t azimuth, uint32_t /*timestamp*/) {
    ScanReplies scan(azimuth, 0);
    
    static int last_revolution = -1;
    if (last_revolution != static_cast<int>(current_revolution_)) {
        last_revolution = current_revolution_;
        for (auto& target : config_.rbs_targets) {
            target.current_mode = 0;
        }
        for (auto& target : config_.uvd_targets) {
            target.uvd_current_mode = 0;
        }
        if (config_.tracker.debug_mode) {
            std::cout << "\n=== New Revolution " << current_revolution_ << " ===\n";
        }
        
        // Отладочный вывод о состоянии логирования
        if (raw_reply_logging_enabled_) {
            std::cout << "[DEBUG] Raw reply logging enabled, file open: " 
                      << raw_reply_file_.is_open() << "\n";
        }
    }
    
    double half_beamwidth_deg = config_.beamwidth_deg / 2.0;
    double current_az_deg = azimuth * RadarConfig::azimuth_per_bin;
    
    auto is_in_beam = [&](double target_az_deg) -> bool {
        double az_diff = std::abs(current_az_deg - target_az_deg);
        az_diff = std::min(az_diff, 360.0 - az_diff);
        return az_diff <= half_beamwidth_deg;
    };
    
    // В начале функции generate_test_scan, после получения текущего оборота
    double time_delta_seconds = current_revolution_ * 2.5; // ~2.5 секунды на оборот

    // Генерация RBS целей
    for (auto& target : config_.rbs_targets) {
        if (!target.enabled) continue;
        
        if (target.use_linear_motion) {
            target.update_linear_position(time_delta_seconds);
        }

        double current_az_deg_target = target.azimuth_deg + current_revolution_ * target.azimuth_speed_deg_per_rev;
        while (current_az_deg_target >= 360.0) current_az_deg_target -= 360.0;
        while (current_az_deg_target < 0) current_az_deg_target += 360.0;
        
        double current_range_km = target.range_km + current_revolution_ * target.range_speed_km_per_rev;
        if (current_range_km < 0) continue;
        
        if (is_in_beam(current_az_deg_target)) {
            uint16_t range_bins = static_cast<uint16_t>(current_range_km * 1000.0 / config_.radar.range_bin_rbs);
            
            uint16_t rbs_code;
            bool spi_value;
            bool is_mode_a = target.get_current_rbs_reply(rbs_code, spi_value);
            
            auto rbs = simulator_->generate_rbs(azimuth, range_bins, rbs_code, spi_value);
            scan.rbs_replies.push_back(rbs);
            
            // Запись в файл СРАЗУ ПОСЛЕ ГЕНЕРАЦИИ
            if (raw_reply_logging_enabled_ && raw_reply_file_.is_open()) {
                double sls_ratio = (rbs.ether_amplitudes_sls[0] > 0) ? 
                    static_cast<double>(rbs.ether_amplitudes_sls[0]) / rbs.ether_amplitudes[0] : 0;
                
                std::string reply_type = is_mode_a ? "MODE_A" : "MODE_C";
                
                raw_reply_file_ << current_revolution_ << ","
                               << azimuth << ","
                               << reply_type << ","
                               << range_bins << ","
                               << "0" << std::oct << rbs_code << std::dec << ","
                               << std::fixed << std::setprecision(3) << sls_ratio << ","
                               << (rbs.is_valid ? "1" : "0") << ","
                               << target.altitude_meters << ","
                               << (spi_value ? "SPI" : "NO_SPI") << "\n";
                raw_reply_file_.flush();  // Немедленная запись на диск
            }
            
            // Переключаем режим для следующего ответа
            target.toggle_rbs_mode();
            
            if (config_.tracker.debug_mode && azimuth % 100 == 0) {
                std::cout << "  RBS '" << target.name << "' [" << (is_mode_a ? "MODE_A" : "MODE_C")
                          << "] alt=" << target.altitude_meters << "m\n";
            }
        }
    }
    
    // Генерация УВД целей
    for (auto& target : config_.uvd_targets) {
        if (!target.enabled) continue;

        if (target.use_linear_motion) {
            target.update_linear_position(time_delta_seconds);
        }
        
        // Отладочный вывод для проверки параметров
        if (config_.tracker.debug_mode && current_revolution_ % 10 == 0 && azimuth == 0) {
            std::cout << "[DEBUG] UVD target '" << target.name 
                    << "': enable_altitude=" << target.enable_altitude
                    << ", alternate_data_altitude=" << target.alternate_data_altitude
                    << ", altitude_meters=" << target.altitude_meters << "\n";
        }
        
        double current_az_deg_target = target.azimuth_deg + current_revolution_ * target.azimuth_speed_deg_per_rev;
        while (current_az_deg_target >= 360.0) current_az_deg_target -= 360.0;
        while (current_az_deg_target < 0) current_az_deg_target += 360.0;
        
        double current_range_km = target.range_km + current_revolution_ * target.range_speed_km_per_rev;
        if (current_range_km < 0) continue;
        
        if (is_in_beam(current_az_deg_target)) {
            uint16_t range_bins = static_cast<uint16_t>(current_range_km * 1000.0 / config_.radar.range_bin_uvd);
            
            uint32_t uvd_data = target.get_current_uvd_data();
            bool is_altitude_reply = (target.enable_altitude && target.alternate_data_altitude && target.uvd_current_mode == 1);
            
            // Отладочный вывод для сгенерированных данных
            if (config_.tracker.debug_mode && target.enable_altitude) {
                std::cout << "[DEBUG] UVD: mode=" << (is_altitude_reply ? "ALT" : "DATA")
                        << " alt=" << target.altitude_meters 
                        << "m, data=0x" << std::hex << uvd_data << std::dec << "\n";
            }
            
            auto uvd = simulator_->generate_uvd(azimuth, range_bins, uvd_data);
            scan.uvd_replies.push_back(uvd);
            
            // Запись в файл
            if (raw_reply_logging_enabled_ && raw_reply_file_.is_open()) {
                double sls_ratio = (uvd.ether_amplitudes_sls[0] > 0) ? 
                    static_cast<double>(uvd.ether_amplitudes_sls[0]) / uvd.ether_amplitudes[0] : 0;
                
                std::string reply_type = is_altitude_reply ? "UVD_ALT" : "UVD_DATA";
                
                raw_reply_file_ << current_revolution_ << ","
                            << azimuth << ","
                            << reply_type << ","
                            << range_bins << ","
                            << uvd_data << ","
                            << std::fixed << std::setprecision(3) << sls_ratio << ","
                            << (uvd.is_valid ? "1" : "0") << ","
                            << target.altitude_meters << "\n";
                raw_reply_file_.flush();
            }
            
            // Переключаем режим для следующего ответа
            target.toggle_uvd_mode();
            
            if (config_.tracker.debug_mode && azimuth % 100 == 0) {
                std::cout << "  UVD '" << target.name << "' [" << (is_altitude_reply ? "ALT" : "DATA")
                        << "] alt=" << target.altitude_meters << "m\n";
            }
        }
    }



    return scan;
}



// file: src/radar_system.cpp (добавить этот метод)

void RadarSystem::enable_raw_reply_logging(bool enable, const std::string& filename) {
    if (enable) {
        raw_reply_file_.open(filename);
        if (raw_reply_file_.is_open()) {
            raw_reply_file_ << "# Raw Radar Replies Log\n";
            raw_reply_file_ << "# Format: revolution,azimuth,type,range_bins,code/data,sls_ratio,is_valid\n";
            raw_reply_file_ << "# " << std::string(80, '-') << "\n";
            raw_reply_file_.flush();
            std::cout << "Raw reply logging enabled to: " << filename << "\n";
        } else {
            std::cerr << "Warning: Cannot open raw reply log file: " << filename << "\n";
        }
    } else {
        if (raw_reply_file_.is_open()) {
            raw_reply_file_.close();
            std::cout << "Raw reply logging disabled\n";
        }
    }
    raw_reply_logging_enabled_ = enable;
}



// file: src/radar_system.cpp - исправленные функции

void RadarSystem::end_of_revolution() {
    if (current_revolution_ > 0) {
        process_revolution_complete();
    }
    
    current_revolution_++;
    
    write_north_marker();  // Теперь без аргументов
    stats_.north_markers++;
    
    if (config_.tracker.debug_mode) {
        std::cout << "\n>>> Revolution " << current_revolution_ 
                  << " completed, " << revolution_targets_.size() << " targets <<<\n";
    }
    
    revolution_targets_.clear();
}

void RadarSystem::process_revolution_complete() {
    if (!tracking_enabled_ || !track_manager_) return;
    
    // Передаем цели в трекер
    track_manager_->process_targets(revolution_targets_, current_revolution_);
    
    // Обновляем статистику
    stats_.targets_reported += revolution_targets_.size();
    
    // Получаем треки
    auto tracks = track_manager_->get_active_tracks();
    stats_.active_tracks = tracks.size();
    
    // Выводим цели в файл
    for (const auto& target : revolution_targets_) {
        write_target_to_file(target);
    }
    
    // Выводим треки в файл
    for (const auto& track : tracks) {
        write_track_to_file(track);
    }
    
    // Выводим в консоль
    if (config_.tracker.debug_mode && !tracks.empty()) {
        std::cout << "Rev " << current_revolution_ << ": " << tracks.size() 
                  << " tracks, " << revolution_targets_.size() << " targets\n";
    }
}

void RadarSystem::write_track_to_file(const Track& track) {
    if (!output_file_.is_open()) return;
    
    output_file_ << "T," << current_revolution_ << ","
                 << track.id << ","
                 << static_cast<int>(track.state) << ","
                 << std::fixed << std::setprecision(2)
                 << track.x << "," << track.y << ","
                 << track.ground_speed << ","
                 << track.course_deg << ","
                 << "0x" << std::hex << track.mode3a_code << std::dec << ","
                 << track.altitude << ","
                 << track.confidence << ","
                 << track.hit_count << "\n";
}

void RadarSystem::write_north_marker() {
    if (!output_file_.is_open()) return;
    
    output_file_ << "N," << current_revolution_ << "," 
                 << last_azimuth_ << "\n";
    output_file_.flush();
}

void RadarSystem::write_target_to_file(const TargetReport& target) {
    if (!output_file_.is_open()) return;
    
    output_file_ << "R," << current_revolution_ << ","
                 << (target.type == TargetReport::SourceType::RBS ? "RBS" : "UVD") << ","
                 << std::fixed << std::setprecision(2)
                 << target.azimuth_deg << ","
                 << target.range_m << ","
                 << target.x << ","
                 << target.y << ",";
    
    if (target.type == TargetReport::SourceType::RBS) {
        output_file_ << "0x" << std::hex << target.rbs.mode3a_code << std::dec << ","
                     << target.rbs.modec_altitude;
    } else {
        output_file_ << "0x" << std::hex << target.uvd.raw_data20 << std::dec << ","
                     << target.uvd.altitude;
    }
    
    output_file_ << "\n";
}

} // namespace radar
