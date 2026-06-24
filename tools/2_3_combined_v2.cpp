// tools/2_3_combined_v2.cpp
// Combined tool for V2 architecture:
// - Clustering (DBSCAN/Legacy) with ClusterPool
// - Tracking with TrackManager V2
// - Plot-based output

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <memory>
#include <cstring>
#include <signal.h>

#include "vrl/radar/core/config.h"
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/core/replies.h"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/legacy_clusterer.h"
#include "vrl/radar/v2/track_manager.hpp"
#include "vrl/radar/v2/plot_pool.hpp"
#include "vrl/radar/v2/grid_config.hpp"
#include "vrl/radar/simulation/simulator.h"
#include "vrl/radar/utils/logger.h"
#include "vrl/radar/utils/utils.h"

using namespace vrl::radar;
using namespace vrl::radar::utils;
using namespace vrl::radar::v2;

// ============================================================================
// КОНСТАНТЫ
// ============================================================================

constexpr int AZIMUTH_BINS = 4096;
constexpr int NUM_SECTORS = 32;

// ============================================================================
// КОНФИГУРАЦИЯ
// ============================================================================

struct CombinedConfigV2 {
    // Входные данные
    std::string input_file{""};
    std::string config_file{"../config/radar.json"};
    std::string output_file{"targets_v2.txt"};
    std::string plots_output_file{"tracks_v2.txt"};
    
    // Режимы
    bool save_plots{false};
    bool verbose{false};
    bool debug{false};
    
    // Параметры кластеризации
    std::string clusterer_type{"dbscan"};
    
    // DBSCAN параметры
    int dbscan_max_range_gap{3};
    double dbscan_azimuth_gap_coefficient{1.2};
    
    // Legacy параметры
    int legacy_max_gap_azimuth{8};
    int legacy_range_window{30};
    
    // Параметры TrackManager V2
    double cell_size_km{5.0};
    double max_range_km{400.0};
    int rings_near{1};
    int rings_far{2};
    double far_threshold_km{150.0};
    double max_candidate_distance_km{10.0};
    double revolution_time_s{5.0};
    
    // Параметры строба
    double range_gate_bins_near{5.0};
    double range_gate_bins_mid{10.0};
    double range_gate_bins_far{20.0};
    double azimuth_gate_maia_near{10.0};
    double azimuth_gate_maia_mid{20.0};
    double azimuth_gate_maia_far{40.0};
    double coast_gate_expansion{1.5};
    
    // COASTING параметры
    int max_coast_revolutions{3};
};

// ============================================================================
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ
// ============================================================================

static std::atomic<bool> g_running{true};
static std::mutex g_output_mutex;

// ============================================================================
// ОБРАБОТЧИК СИГНАЛОВ
// ============================================================================

void signal_handler(int sig) {
    (void)sig;
    VRL_LOG_INFO(modules::MAIN, "Received interrupt signal, shutting down...");
    g_running = false;
}

// ============================================================================
// КЛАСС PROCESSOR V2
// ============================================================================

class RadarProcessorV2 {
public:
    RadarProcessorV2() = default;
    ~RadarProcessorV2() = default;
    
    bool init(const CombinedConfigV2& config);
    void run();
    void shutdown();
    
private:
    bool load_config();
    bool load_data();
    bool init_clusterer();
    bool init_track_manager();
    
    void process_loop();
    void process_scan(const ScanReplies& scan);
    void save_results();
    void print_stats();
    
    // Загрузка данных из файла
    bool load_scans_from_file(const std::string& filename);
    
    CombinedConfigV2 config_;
    SystemConfig system_config_;
    
    std::unique_ptr<IClusterer> clusterer_;
    std::unique_ptr<TrackManager> track_manager_;
    
    // Данные
    std::vector<ScanReplies> scans_;
    
    // Статистика
    struct Stats {
        uint32_t scans_processed{0};
        uint32_t replies_processed{0};
        uint32_t clusters_formed{0};
        uint32_t tracks_created{0};
        uint32_t tracks_confirmed{0};
        uint32_t tracks_updated{0};
        uint32_t tracks_coasted{0};
        uint32_t tracks_dropped{0};
        double processing_time_ms{0.0};
    } stats_;
    
    uint32_t revolution_counter_{0};
    uint64_t scan_counter_{0};
    
    // Сохранение результатов
    struct SavedReport {
        double azimuth_deg;
        double range_km;
        uint32_t code;
        std::string type;
        double confidence;
        uint32_t track_id;
        uint64_t time_maia;
    };
    std::vector<SavedReport> saved_reports_;
    std::vector<Track> saved_tracks_;
};

// ============================================================================
// РЕАЛИЗАЦИЯ
// ============================================================================

bool RadarProcessorV2::init(const CombinedConfigV2& config) {
    config_ = config;
    
    // Настройка логгера
    if (config_.verbose) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::DEBUG);
    } else if (config_.debug) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::TRACE);
    } else {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::INFO);
    }
    
    VRL_LOG_INFO(modules::MAIN, "=== VRL-Radar Processor V2 ===");
    VRL_LOG_INFO(modules::MAIN, "Clusterer type: " + config_.clusterer_type);
    VRL_LOG_INFO(modules::MAIN, "Input file: " + config_.input_file);
    
    // Загружаем конфигурацию
    if (!load_config()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to load config");
        return false;
    }
    
    // Инициализируем PointBuffer
    PointBuffer::instance().init(65536);
    VRL_LOG_INFO(modules::MAIN, "PointBuffer initialized");
    
    // Инициализируем ClusterPool
    ClusterPool::instance().init(65535);
    VRL_LOG_INFO(modules::MAIN, "ClusterPool initialized");
    
    // Инициализируем кластеризатор
    if (!init_clusterer()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize clusterer");
        return false;
    }
    
    // Инициализируем TrackManager V2
    if (!init_track_manager()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize TrackManager V2");
        return false;
    }
    
    // Загружаем данные
    if (!load_data()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to load data from: " + config_.input_file);
        return false;
    }
    
    VRL_LOG_INFO(modules::MAIN, "Processor V2 initialized successfully");
    return true;
}

bool RadarProcessorV2::load_config() {
    if (!config_.config_file.empty()) {
        ConfigLoader loader;
        if (loader.load(config_.config_file, system_config_)) {
            VRL_LOG_INFO(modules::MAIN, "Loaded config from: " + config_.config_file);
            return true;
        } else {
            VRL_LOG_ERROR(modules::MAIN, "Failed to load config: " + config_.config_file);
        }
    }
    
    // Конфигурация по умолчанию
    system_config_.radar.range_bin_rbs = 30.0;
    system_config_.radar.range_bin_uvd = 60.0;
    system_config_.radar.beamwidth_deg = 5.0;
    system_config_.radar.min_amplitude = 10;
    system_config_.beamwidth_deg = 5.0;
    system_config_.revolution_time = 5.0;
    
    VRL_LOG_INFO(modules::MAIN, "Using default configuration");
    return true;
}

bool RadarProcessorV2::init_clusterer() {
    RadarConfig radar_config;
    radar_config.range_bin_rbs = system_config_.radar.range_bin_rbs;
    radar_config.range_bin_uvd = system_config_.radar.range_bin_uvd;
    radar_config.beamwidth_deg = system_config_.radar.beamwidth_deg;
    radar_config.min_amplitude = system_config_.radar.min_amplitude;
    
    if (config_.clusterer_type == "dbscan") {
        VRL_LOG_INFO(modules::MAIN, "Creating DBSCANClusterer: range_gap=" + 
                     std::to_string(config_.dbscan_max_range_gap) +
                     ", az_coeff=" + std::to_string(config_.dbscan_azimuth_gap_coefficient));
        
        clusterer_ = std::make_unique<DBSCANClusterer>(
            radar_config,
            config_.dbscan_max_range_gap,
            config_.dbscan_azimuth_gap_coefficient
        );
        
        if (config_.debug) {
            auto* dbscan = dynamic_cast<DBSCANClusterer*>(clusterer_.get());
            if (dbscan) {
                dbscan->set_debug(true);
            }
        }
    } else {
        VRL_LOG_INFO(modules::MAIN, "Creating LegacyClusterer: gap=" + 
                     std::to_string(config_.legacy_max_gap_azimuth) +
                     ", window=" + std::to_string(config_.legacy_range_window));
        
        clusterer_ = std::make_unique<LegacyClusterer>(
            config_.legacy_max_gap_azimuth,
            config_.legacy_range_window
        );
    }
    
    return clusterer_ != nullptr;
}

bool RadarProcessorV2::init_track_manager() {
    // Настраиваем GridConfig
    GridConfig grid_config;
    grid_config.cell_size_km = config_.cell_size_km;
    grid_config.max_range_km = config_.max_range_km;
    grid_config.rings_near = config_.rings_near;
    grid_config.rings_far = config_.rings_far;
    grid_config.far_threshold_km = config_.far_threshold_km;
    grid_config.max_candidate_distance_km = config_.max_candidate_distance_km;
    grid_config.revolution_time_s = config_.revolution_time_s;
    
    // Параметры строба
    grid_config.range_gate_bins_near = config_.range_gate_bins_near;
    grid_config.range_gate_bins_mid = config_.range_gate_bins_mid;
    grid_config.range_gate_bins_far = config_.range_gate_bins_far;
    grid_config.azimuth_gate_maia_near = config_.azimuth_gate_maia_near;
    grid_config.azimuth_gate_maia_mid = config_.azimuth_gate_maia_mid;
    grid_config.azimuth_gate_maia_far = config_.azimuth_gate_maia_far;
    grid_config.coast_gate_expansion = config_.coast_gate_expansion;
    grid_config.max_coast_revolutions = config_.max_coast_revolutions;
    
    VRL_LOG_INFO(modules::MAIN, "TrackManager V2 config: cell_size=" + 
                 std::to_string(grid_config.cell_size_km) + "km, max_range=" +
                 std::to_string(grid_config.max_range_km) + "km");
    VRL_LOG_INFO(modules::MAIN, "  coast_max=" + 
                 std::to_string(grid_config.max_coast_revolutions) + " revs");
    
    track_manager_ = std::make_unique<TrackManager>();
    track_manager_->init(grid_config);
    
    VRL_LOG_INFO(modules::MAIN, "TrackManager V2 initialized");
    return true;
}

// ============================================================================
// ЗАГРУЗКА ДАННЫХ ИЗ ФАЙЛА
// ============================================================================

bool RadarProcessorV2::load_data() {
    if (config_.input_file.empty()) {
        VRL_LOG_ERROR(modules::MAIN, "No input file specified");
        return false;
    }
    
    return load_scans_from_file(config_.input_file);
}

bool RadarProcessorV2::load_scans_from_file(const std::string& filename) {
    VRL_LOG_INFO(modules::MAIN, "Loading scans from: " + filename);
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        VRL_LOG_ERROR(modules::MAIN, "Cannot open file: " + filename);
        return false;
    }
    
    scans_.clear();
    
    std::string line;
    int line_num = 0;
    uint32_t current_revolution = 0;
    uint32_t current_time_ms = 0;
    
    while (std::getline(file, line)) {
        line_num++;
        
        // Пропускаем комментарии и пустые строки
        if (line.empty() || line[0] == '#') continue;
        
        // Парсим строку: azimuth, rbs_count, uvd_count, [rbs_data...], [uvd_data...]
        std::stringstream ss(line);
        std::string token;
        
        std::vector<std::string> tokens;
        while (ss >> token) {
            tokens.push_back(token);
        }
        
        if (tokens.size() < 3) {
            VRL_LOG_WARN(modules::MAIN, "Line " + std::to_string(line_num) + 
                         ": invalid format, skipping");
            continue;
        }
        
        uint16_t azimuth = static_cast<uint16_t>(std::stoi(tokens[0]));
        int rbs_count = std::stoi(tokens[1]);
        int uvd_count = std::stoi(tokens[2]);
        
        ScanReplies scan(azimuth, current_time_ms);
        
        int idx = 3;
        
        // Читаем RBS ответы
        for (int i = 0; i < rbs_count && idx + 3 < static_cast<int>(tokens.size()); ++i) {
            uint16_t range_bin = static_cast<uint16_t>(std::stoi(tokens[idx++]));
            uint16_t code12 = static_cast<uint16_t>(std::stoi(tokens[idx++]));
            bool spi = (std::stoi(tokens[idx++]) != 0);
            
            RBSReply reply;
            reply.azimuth = azimuth;
            reply.range = range_bin;
            reply.code12 = code12;
            reply.spi = spi;
            reply.is_valid = true;
            
            // Заполняем амплитуды (упрощённо)
            reply.ether_amplitudes[0] = 100;   // F1
            reply.ether_amplitudes[14] = 100;  // F2
            for (int j = 0; j < 12; ++j) {
                if (code12 & (1 << j)) {
                    reply.ether_amplitudes[utils::bit_position(j)] = 100;
                }
            }
            
            scan.rbs_replies.push_back(reply);
        }
        
        // Читаем UVD ответы
        for (int i = 0; i < uvd_count && idx < static_cast<int>(tokens.size()); ++i) {
            uint16_t range_bin = static_cast<uint16_t>(std::stoi(tokens[idx++]));
            uint32_t data20 = static_cast<uint32_t>(std::stoul(tokens[idx++]));
            
            UVDReply reply;
            reply.azimuth = azimuth;
            reply.range = range_bin;
            reply.data20 = data20;
            reply.is_valid = true;
            
            scan.uvd_replies.push_back(reply);
        }
        
        scans_.push_back(scan);
        current_time_ms += 10;  // примерно 10 мс на сканирование
        
        // Определяем смену оборота
        if (azimuth < 100 && scans_.size() > 1) {
            current_revolution++;
        }
    }
    
    file.close();
    
    VRL_LOG_INFO(modules::MAIN, "Loaded " + std::to_string(scans_.size()) + 
                 " scans from " + filename);
    return true;
}

// ============================================================================
// ОСНОВНОЙ ЦИКЛ ОБРАБОТКИ
// ============================================================================

void RadarProcessorV2::run() {
    if (scans_.empty()) {
        VRL_LOG_ERROR(modules::MAIN, "No scans to process");
        return;
    }
    
    VRL_LOG_INFO(modules::MAIN, "Starting processing: " + std::to_string(scans_.size()) + " scans");
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (const auto& scan : scans_) {
        if (!g_running) break;
        
        process_scan(scan);
        scan_counter_++;
        
        if (config_.verbose && scan_counter_ % 100 == 0) {
            VRL_LOG_DEBUG(modules::MAIN, "Processed " + std::to_string(scan_counter_) + 
                          " scans");
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    stats_.processing_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time
    ).count();
    
    // Получаем финальные треки
    auto all_tracks = track_manager_->get_plots();
    for (const auto& plot : all_tracks) {
        Track track;
        track.id = 0;  // не используется
        track.x = plot.x;
        track.y = plot.y;
        track.azimuth_maia = plot.azimuth_maia;
        track.range_bins = plot.range_bins;
        track.mode3a_code = plot.mode3a_code;
        track.uvd_data20 = plot.uvd_data20;
        track.altitude = plot.altitude;
        track.spi = plot.spi;
        track.state = 1;  // ACTIVE
        saved_tracks_.push_back(track);
    }
    
    print_stats();
    save_results();
}

void RadarProcessorV2::process_scan(const ScanReplies& scan) {
    stats_.scans_processed++;
    stats_.replies_processed += scan.reply_count();
    
    // 1. Кластеризация (заполняет ClusterPool)
    clusterer_->process_scan(scan);
    
    // 2. Обработка треков (V2)
    track_manager_->process_azimuth(scan.azimuth);
    
    // 3. Получение обновлённых треков
    const auto& updated_ids = track_manager_->get_updated_tracks();
    
    for (uint64_t track_id : updated_ids) {
        const Plot* plot = track_manager_->get_plot(track_id);
        if (!plot) continue;
        
        stats_.tracks_updated++;
        
        // Сохраняем отчет
        SavedReport report;
        report.azimuth_deg = plot->azimuth_maia * 360.0 / 4096.0;
        report.range_km = plot->range_bins * 30.0 / 1000.0;  // упрощённо
        report.confidence = plot->confidence;
        report.track_id = static_cast<uint32_t>(track_id);
        report.time_maia = 0;  // TODO: добавить время
        
        if (plot->source_type == Plot::SourceType::RBS) {
            report.code = plot->mode3a_code;
            report.type = "RBS";
        } else {
            report.code = plot->uvd_data20;
            report.type = "UVD";
        }
        
        saved_reports_.push_back(report);
    }
    
    // Очищаем список после обработки
    track_manager_->clear_updated_tracks();
    
    // Статистика
    auto stats = track_manager_->get_stats();
    stats_.tracks_created = stats.active_tracks + stats.coasting_tracks;
    stats_.tracks_confirmed = stats.confirmed_tracks;
    stats_.tracks_coasted = stats.coasting_tracks;
}

// ============================================================================
// СОХРАНЕНИЕ РЕЗУЛЬТАТОВ
// ============================================================================

void RadarProcessorV2::save_results() {
    // Сохраняем отчеты (совместимый формат с 2_3_combined)
    if (!config_.output_file.empty()) {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        
        std::ofstream file(config_.output_file);
        if (!file.is_open()) {
            VRL_LOG_ERROR(modules::MAIN, "Failed to open output file: " + config_.output_file);
        } else {
            file << "# VRL-Radar Targets Output (V2)\n";
            file << "# Generated by 2_3_combined_v2\n";
            file << "# Clusterer: " << config_.clusterer_type << "\n";
            file << "# Format: track_id, azimuth_deg, range_km, code, type, confidence\n\n";
            
            for (const auto& report : saved_reports_) {
                file << report.track_id << ", ";
                file << std::fixed << std::setprecision(2);
                file << report.azimuth_deg << ", ";
                file << report.range_km << ", ";
                
                if (report.type == "RBS") {
                    file << std::oct << report.code << ", ";
                    file << "RBS, ";
                } else {
                    file << std::hex << report.code << ", ";
                    file << "UVD, ";
                }
                
                file << std::dec << std::fixed << std::setprecision(3);
                file << report.confidence << "\n";
            }
            
            file.close();
            VRL_LOG_INFO(modules::MAIN, "Saved " + std::to_string(saved_reports_.size()) + 
                         " reports to: " + config_.output_file);
        }
    }
    
    // Сохраняем треки (совместимый формат с 2_3_combined)
    if (!config_.plots_output_file.empty() && !saved_tracks_.empty()) {
        std::lock_guard<std::mutex> lock(g_output_mutex);
        
        std::ofstream file(config_.plots_output_file);
        if (!file.is_open()) {
            VRL_LOG_ERROR(modules::MAIN, "Failed to open tracks file: " + config_.plots_output_file);
        } else {
            file << "# VRL-Radar Tracks Output (V2)\n";
            file << "# Format: id, x, y, speed, course, state\n\n";
            
            for (const auto& track : saved_tracks_) {
                double speed = std::sqrt(track.vx * track.vx + track.vy * track.vy);
                double course = std::atan2(track.vx, track.vy) * 180.0 / M_PI;
                if (course < 0) course += 360.0;
                
                file << track.id << ", ";
                file << std::fixed << std::setprecision(1);
                file << track.x << ", " << track.y << ", ";
                file << speed << ", " << course << ", ";
                
                switch (track.state) {
                    case 0: file << "NEW"; break;
                    case 1: file << "ACTIVE"; break;
                    case 2: file << "COASTING"; break;
                    case 3: file << "DROPPED"; break;
                    default: file << "UNKNOWN"; break;
                }
                file << "\n";
            }
            
            file.close();
            VRL_LOG_INFO(modules::MAIN, "Saved " + std::to_string(saved_tracks_.size()) + 
                         " tracks to: " + config_.plots_output_file);
        }
    }
}

// ============================================================================
// СТАТИСТИКА
// ============================================================================

void RadarProcessorV2::print_stats() {
    std::stringstream ss;
    ss << "\n";
    ss << "=== Processing Statistics (V2) ===\n";
    ss << "Scans processed:       " << stats_.scans_processed << "\n";
    ss << "Replies processed:     " << stats_.replies_processed << "\n";
    ss << "Tracks created:        " << stats_.tracks_created << "\n";
    ss << "Tracks confirmed:      " << stats_.tracks_confirmed << "\n";
    ss << "Tracks updated:        " << stats_.tracks_updated << "\n";
    ss << "Tracks coasting:       " << stats_.tracks_coasted << "\n";
    ss << "Processing time:       " << stats_.processing_time_ms << " ms\n";
    ss << "================================\n";
    
    VRL_LOG_INFO(modules::MAIN, ss.str());
}

void RadarProcessorV2::shutdown() {
    VRL_LOG_INFO(modules::MAIN, "Shutting down processor V2...");
}

// ============================================================================
// ПАРСИНГ АРГУМЕНТОВ
// ============================================================================

CombinedConfigV2 parse_arguments(int argc, char* argv[]) {
    CombinedConfigV2 config;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--config" && i + 1 < argc) {
            config.config_file = argv[++i];
        } else if (arg == "--input" && i + 1 < argc) {
            config.input_file = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            config.output_file = argv[++i];
        } else if (arg == "--plots" && i + 1 < argc) {
            config.plots_output_file = argv[++i];
            config.save_plots = true;
        } else if (arg == "--clusterer" && i + 1 < argc) {
            config.clusterer_type = argv[++i];
        } else if (arg == "--dbscan-range-gap" && i + 1 < argc) {
            config.dbscan_max_range_gap = std::stoi(argv[++i]);
        } else if (arg == "--dbscan-az-coeff" && i + 1 < argc) {
            config.dbscan_azimuth_gap_coefficient = std::stod(argv[++i]);
        } else if (arg == "--legacy-gap" && i + 1 < argc) {
            config.legacy_max_gap_azimuth = std::stoi(argv[++i]);
        } else if (arg == "--legacy-window" && i + 1 < argc) {
            config.legacy_range_window = std::stoi(argv[++i]);
        } else if (arg == "--cell-size" && i + 1 < argc) {
            config.cell_size_km = std::stod(argv[++i]);
        } else if (arg == "--max-range" && i + 1 < argc) {
            config.max_range_km = std::stod(argv[++i]);
        } else if (arg == "--max-coast" && i + 1 < argc) {
            config.max_coast_revolutions = std::stoi(argv[++i]);
        } else if (arg == "--verbose" || arg == "-v") {
            config.verbose = true;
        } else if (arg == "--debug" || arg == "-d") {
            config.debug = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  --config FILE           Config file\n"
                      << "  --input FILE            Input data file (REQUIRED)\n"
                      << "  --output FILE           Output targets file\n"
                      << "  --plots FILE            Output tracks file\n\n"
                      << "  --clusterer TYPE        Clusterer type: dbscan or legacy (default: dbscan)\n\n"
                      << "  DBSCAN options:\n"
                      << "  --dbscan-range-gap N    Max range gap in bins (default: 3)\n"
                      << "  --dbscan-az-coeff N    Azimuth gap coefficient (default: 1.2)\n\n"
                      << "  Legacy options:\n"
                      << "  --legacy-gap N          Max gap azimuth (default: 8)\n"
                      << "  --legacy-window N       Range window (default: 30)\n\n"
                      << "  TrackManager V2 options:\n"
                      << "  --cell-size N           Grid cell size in km (default: 5)\n"
                      << "  --max-range N           Max range in km (default: 400)\n"
                      << "  --max-coast N           Max coast revolutions (default: 3)\n\n"
                      << "  General:\n"
                      << "  --verbose, -v           Verbose output\n"
                      << "  --debug, -d             Debug mode (trace logging)\n"
                      << "  --help, -h              Show this help\n";
            exit(0);
        }
    }
    
    if (config.input_file.empty()) {
        std::cerr << "Error: --input file is required\n";
        exit(1);
    }
    
    return config;
}

// ============================================================================
// ТОЧКА ВХОДА
// ============================================================================

int main(int argc, char* argv[]) {
    // Настройка обработчика сигналов
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Парсим аргументы
    CombinedConfigV2 config = parse_arguments(argc, argv);
    
    // Создаем и запускаем процессор
    RadarProcessorV2 processor;
    if (!processor.init(config)) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize processor V2");
        return 1;
    }
    
    processor.run();
    processor.shutdown();
    
    VRL_LOG_INFO(modules::MAIN, "Done.");
    return 0;
}
