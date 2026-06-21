// tools/2_3_combined.cpp
// Combined tool for:
// 2. Reply processing with clustering
// 3. Target tracking with Kalman filter

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <memory>
#include <cstring>
#include <signal.h>

#include "vrl/radar/core/config.h"
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/core/replies.h"
#include "vrl/radar/core/object_pool.hpp"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/simulation/simulator.h"
#include "vrl/radar/utils/logger.h"
#include "vrl/radar/utils/utils.h"

using namespace vrl::radar;
using namespace vrl::radar::utils;

// ============================================================================
// КОНСТАНТЫ
// ============================================================================

constexpr int AZIMUTH_BINS = 4096;
constexpr double DEG_TO_BIN = AZIMUTH_BINS / 360.0;

// ============================================================================
// КОНФИГУРАЦИЯ
// ============================================================================

struct CombinedConfig {
    // Входные данные
    std::string input_file{""};
    std::string config_file{"../config/radar.json"};
    std::string output_file{"targets.txt"};
    std::string plots_output_file{""};
    
    // Режимы
    bool real_time{true};
    bool save_plots{false};
    bool verbose{false};
    bool debug{false};
    
    // Параметры обработки
    int max_gap_azimuth{8};
    int range_window{30};
    int min_hits{2};
    int min_cluster_hits{2};
    double min_confidence{0.3};
    
    // Параметры трекера
    int min_hits_to_confirm{3};
    int max_coast_count{5};
    double max_gate_distance{150.0};
    double max_gate_azimuth{30.0};
    
    // ========================================================================
    // ПАРАМЕТРЫ КЛАСТЕРИЗАЦИИ
    // ========================================================================
    
    // Тип кластеризатора: "dbscan" или "legacy"
    std::string clusterer_type{"dbscan"};
    
    // Параметры для DBSCAN
    int dbscan_max_range_gap{3};
    int dbscan_min_points{2};
    double dbscan_azimuth_gap_coefficient{1.2};
    
    // Параметры для Legacy
    int legacy_max_gap_azimuth{8};
    int legacy_range_window{30};
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
// КЛАСС PROCESSOR
// ============================================================================

class RadarProcessor {
public:
    RadarProcessor() = default;
    ~RadarProcessor() = default;
    
    bool init(const CombinedConfig& config);
    void run();
    void shutdown();
    
private:
    bool load_config();
    bool load_data();
    bool init_tracker();
    bool init_simulator();
    
    void process_loop();
    void process_scan(const ScanReplies& scan);
    void save_results();
    void print_stats();
    
    // Генерация тестовых данных (если нет входного файла)
    void generate_test_data();
    
    CombinedConfig config_;
    SystemConfig system_config_;
    
    std::unique_ptr<ReplySimulator> simulator_;
    std::unique_ptr<ClusterTracker> cluster_tracker_;
    std::unique_ptr<TrackManager> track_manager_;
    
    // Данные
    std::vector<ScanReplies> scans_;
    std::vector<TargetReport> all_reports_;
    std::vector<Track> all_tracks_;
    
    // Статистика
    struct Stats {
        uint32_t scans_processed{0};
        uint32_t replies_processed{0};
        uint32_t clusters_formed{0};
        uint32_t reports_generated{0};
        uint32_t tracks_created{0};
        uint32_t tracks_confirmed{0};
        double processing_time_ms{0.0};
    } stats_;
    
    uint32_t revolution_counter_{0};
    uint64_t scan_counter_{0};
};

// ============================================================================
// РЕАЛИЗАЦИЯ
// ============================================================================


bool RadarProcessor::init(const CombinedConfig& config) {
    config_ = config;
    
    // Настройка логгера - ИСПРАВЛЕНО
    if (config_.verbose) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::DEBUG);
    } else if (config_.debug) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::TRACE);
    } else {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::INFO);
    }
    
    VRL_LOG_INFO(modules::MAIN, "=== VRL-Radar Processor (2_3_combined) ===");
    VRL_LOG_INFO(modules::MAIN, "Clusterer type: " + config_.clusterer_type);
    
    if (!load_config()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to load config");
        return false;
    }

    // ============================================================
    // ИНИЦИАЛИЗАЦИЯ POINT BUFFER
    // ============================================================
    size_t point_buffer_size = 65536;  // или из конфига
    PointBuffer::instance().init(point_buffer_size);
    VRL_LOG_INFO(modules::MAIN, "PointBuffer initialized with size: " + 
                 std::to_string(point_buffer_size));
    

    if (!init_tracker()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize tracker");
        return false;
    }
    
    if (!config_.input_file.empty()) {
        if (!load_data()) {
            VRL_LOG_ERROR(modules::MAIN, "Failed to load data from: " + config_.input_file);
            return false;
        }
    } else {
        VRL_LOG_INFO(modules::MAIN, "No input file specified, generating test data");
        if (!init_simulator()) {
            VRL_LOG_ERROR(modules::MAIN, "Failed to initialize simulator");
            return false;
        }
        generate_test_data();
    }
    
    VRL_LOG_INFO(modules::MAIN, "Processor initialized successfully");
    return true;
}


bool RadarProcessor::load_config() {
    if (!config_.config_file.empty()) {
        ConfigLoader loader;
        if (loader.load(config_.config_file, system_config_)) {
            VRL_LOG_INFO(modules::MAIN, "Loaded config from: " + config_.config_file);
            
            // Выводим информацию о кластеризаторе
            if (system_config_.clusterer.type == ClustererConfig::Type::DBSCAN) {
                VRL_LOG_INFO(modules::MAIN, "Clusterer: DBSCAN (range_gap=" + 
                             std::to_string(system_config_.clusterer.max_range_gap) +
                             ", az_coeff=" + 
                             std::to_string(system_config_.clusterer.azimuth_gap_coefficient) + ")");
            } else {
                VRL_LOG_INFO(modules::MAIN, "Clusterer: Legacy (gap=" + 
                             std::to_string(system_config_.clusterer.max_gap_azimuth) +
                             ", window=" + 
                             std::to_string(system_config_.clusterer.range_window) + ")");
            }
            
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
    
    system_config_.tracker.min_hits_to_confirm = config_.min_hits_to_confirm;
    system_config_.tracker.max_coast_count = config_.max_coast_count;
    system_config_.tracker.max_gate_distance = config_.max_gate_distance;
    system_config_.tracker.max_gate_azimuth = config_.max_gate_azimuth;
    system_config_.tracker.process_noise = 0.1;
    system_config_.tracker.measurement_noise = 1.0;
    system_config_.tracker.enable_rbs_tracking = true;
    system_config_.tracker.enable_uvd_tracking = true;
    
    return true;
}

bool RadarProcessor::init_simulator() {
    SimulatorConfig sim_config;
    sim_config.radar = system_config_.radar;
    sim_config.rbs.snr_db = 20.0;
    sim_config.rbs.amp_variation = 0.1;
    sim_config.uvd.snr_db = 20.0;
    sim_config.uvd.error_probability = 0.01;
    sim_config.sls.enabled = false;
    
    simulator_ = std::make_unique<ReplySimulator>(sim_config);
    VRL_LOG_INFO(modules::MAIN, "Simulator initialized");
    return true;
}

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ ТРЕКЕРА С ВЫБОРОМ АЛГОРИТМА
// ============================================================================

bool RadarProcessor::init_tracker() {
    ClustererConfig clusterer_config = system_config_.clusterer;
    
    if (clusterer_config.type == ClustererConfig::Type::DBSCAN) {
        VRL_LOG_INFO(modules::MAIN, "Using DBSCANClusterer: range_gap=" + 
                     std::to_string(clusterer_config.max_range_gap) +
                     ", azimuth_coeff=" + std::to_string(clusterer_config.azimuth_gap_coefficient));
    } else {
        VRL_LOG_INFO(modules::MAIN, "Using LegacyClusterer: gap=" + 
                     std::to_string(clusterer_config.max_gap_azimuth) +
                     ", window=" + std::to_string(clusterer_config.range_window));
    }
    
    cluster_tracker_ = std::make_unique<ClusterTracker>(clusterer_config);
    track_manager_ = std::make_unique<TrackManager>(system_config_.tracker);
    
    if (config_.debug) {
        auto* clusterer = cluster_tracker_->get_clusterer();
        if (clusterer) {
            auto* dbscan = dynamic_cast<DBSCANClusterer*>(clusterer);
            if (dbscan) {
                dbscan->set_debug(true);
            }
        }
    }
    
    VRL_LOG_INFO(modules::MAIN, "Tracker initialized with: " + 
                 cluster_tracker_->get_algorithm_name());
    return true;
}

bool RadarProcessor::load_data() {
    // TODO: Загрузка данных из файла (реализовать позже)
    VRL_LOG_WARN(modules::MAIN, "Data loading from file not implemented yet");
    return false;
}

void RadarProcessor::generate_test_data() {
    VRL_LOG_INFO(modules::MAIN, "Generating test data...");
    
    // Создаем тестовые цели
    std::vector<GeneratedTarget> targets;
    
    // Цель 1: RBS
    GeneratedTarget target1;
    target1.type = GeneratedTarget::Type::RBS;
    target1.name = "RBS_Target_1";
    target1.azimuth_deg = 45.0;
    target1.range_km = 20.0;
    target1.rbs_code_octal = 01234;  // 1234 в восьмеричной системе
    target1.spi = false;
    target1.enabled = true;
    targets.push_back(target1);
    
    // Цель 2: RBS с SPI
    GeneratedTarget target2;
    target2.type = GeneratedTarget::Type::RBS;
    target2.name = "RBS_Target_2";
    target2.azimuth_deg = 120.0;
    target2.range_km = 35.0;
    target2.rbs_code_octal = 05670;  // 5670 в восьмеричной системе
    target2.spi = true;
    target2.enabled = true;
    targets.push_back(target2);
    
    // Цель 3: UVD
    GeneratedTarget target3;
    target3.type = GeneratedTarget::Type::UVD;
    target3.name = "UVD_Target_3";
    target3.azimuth_deg = 200.0;
    target3.range_km = 50.0;
    target3.uvd_data_dec = 0x12345;
    target3.enabled = true;
    targets.push_back(target3);
    
    // Цель 4: UVD
    GeneratedTarget target4;
    target4.type = GeneratedTarget::Type::UVD;
    target4.name = "UVD_Target_4";
    target4.azimuth_deg = 300.0;
    target4.range_km = 25.0;
    target4.uvd_data_dec = 0x67890;
    target4.enabled = true;
    targets.push_back(target4);
    
    // Генерируем сканы на 5 оборотов
    const int REVOLUTIONS = 5;
    const int SCANS_PER_REV = 360;
    
    for (int rev = 0; rev < REVOLUTIONS; ++rev) {
        for (int deg = 0; deg < 360; deg += 1) {
            uint16_t azimuth = static_cast<uint16_t>(deg * DEG_TO_BIN);
            ScanReplies scan(azimuth, rev * 360 + deg);
            
            // Проверяем каждую цель
            for (const auto& target : targets) {
                if (!target.enabled) continue;
                
                int target_az_bin = static_cast<int>(target.azimuth_deg * DEG_TO_BIN);
                int az_diff = std::abs(static_cast<int>(azimuth) - target_az_bin);
                if (az_diff > AZIMUTH_BINS / 2) {
                    az_diff = AZIMUTH_BINS - az_diff;
                }
                
                // Цель видна в пределах +/- 2 градуса
                if (az_diff < static_cast<int>(2 * DEG_TO_BIN)) {
                    if (target.type == GeneratedTarget::Type::RBS) {
                        uint16_t range_bin = static_cast<uint16_t>(target.range_km * 1000.0 / 
                                                                   system_config_.radar.range_bin_rbs);
                        RBSReply reply = simulator_->generate_rbs(
                            azimuth,
                            range_bin,
                            target.get_rbs_code(),
                            target.spi
                        );
                        scan.rbs_replies.push_back(reply);
                    } else {
                        uint16_t range_bin = static_cast<uint16_t>(target.range_km * 1000.0 / 
                                                                   system_config_.radar.range_bin_uvd);
                        UVDReply reply = simulator_->generate_uvd(
                            azimuth,
                            range_bin,
                            target.get_current_uvd_data()
                        );
                        scan.uvd_replies.push_back(reply);
                    }
                }
            }
            
            scans_.push_back(scan);
        }
    }
    
    VRL_LOG_INFO(modules::MAIN, "Generated " + std::to_string(scans_.size()) + " scans");
}

// ============================================================================
// ОСНОВНОЙ ЦИКЛ ОБРАБОТКИ
// ============================================================================

void RadarProcessor::run() {
    if (scans_.empty()) {
        VRL_LOG_ERROR(modules::MAIN, "No scans to process");
        return;
    }
    
    VRL_LOG_INFO(modules::MAIN, "Starting processing: " + std::to_string(scans_.size()) + " scans");
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    for (const auto& scan : scans_) {
        if (!g_running) break;
        
        process_scan(scan);
        revolution_counter_++;
        
        if (config_.verbose && revolution_counter_ % 100 == 0) {
            VRL_LOG_DEBUG(modules::MAIN, "Processed " + std::to_string(revolution_counter_) + 
                          " scans, " + std::to_string(all_reports_.size()) + " reports");
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    stats_.processing_time_ms = std::chrono::duration<double, std::milli>(
        end_time - start_time
    ).count();
    
    // Получаем финальные треки
    auto tracks = track_manager_->get_active_tracks();
    all_tracks_.insert(all_tracks_.end(), tracks.begin(), tracks.end());
    
    print_stats();
    save_results();
}

void RadarProcessor::process_scan(const ScanReplies& scan) {
    stats_.scans_processed++;
    stats_.replies_processed += scan.reply_count();
    
    if (!scan.has_replies()) {
        // Пустой скан - все равно нужно обработать для закрытия кластеров
        cluster_tracker_->process_scan(scan);
        return;
    }
    
    // 1. Кластеризация
    cluster_tracker_->process_scan(scan);
    
    // 2. Получаем завершенные кластеры
    auto clusters = cluster_tracker_->get_completed_clusters();
    stats_.clusters_formed += clusters.size();
    
    if (clusters.empty()) {
        return;
    }
    
    // 3. Обработка кластеров -> отчеты
    ClusterProcessor processor(system_config_.radar);
    processor.set_min_hits(config_.min_hits);
    processor.set_confidence_threshold(config_.min_confidence);
    
    std::vector<TargetReport> reports;
    for (const auto& cluster : clusters) {
        auto cluster_reports = processor.process_cluster(cluster);
        reports.insert(reports.end(), cluster_reports.begin(), cluster_reports.end());
    }
    
    if (reports.empty()) {
        return;
    }
    
    stats_.reports_generated += reports.size();
    
    // 4. Трекинг
    uint32_t revolution = static_cast<uint32_t>(revolution_counter_ / 360);
    track_manager_->process_targets(reports, revolution);
    
    // 5. Сохраняем отчеты
    all_reports_.insert(all_reports_.end(), reports.begin(), reports.end());
    
    // 6. Получаем активные треки для статистики
    auto tracks = track_manager_->get_active_tracks();
    stats_.tracks_created = tracks.size();
    
    // Подсчет подтвержденных треков
    stats_.tracks_confirmed = 0;
    for (const auto& track : tracks) {
        if (track.is_confirmed()) {
            stats_.tracks_confirmed++;
        }
    }
}

// ============================================================================
// СОХРАНЕНИЕ РЕЗУЛЬТАТОВ
// ============================================================================

void RadarProcessor::save_results() {
    if (config_.output_file.empty()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(g_output_mutex);
    
    std::ofstream file(config_.output_file);
    if (!file.is_open()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to open output file: " + config_.output_file);
        return;
    }
    
    file << "# VRL-Radar Targets Output\n";
    file << "# Generated by 2_3_combined\n";
    file << "# Clusterer: " << config_.clusterer_type << "\n";
    file << "# Format: revolution, azimuth_deg, range_km, code, type, confidence\n\n";
    
    for (const auto& report : all_reports_) {
        file << std::fixed << std::setprecision(2);
        file << revolution_counter_ << ", ";
        file << report.azimuth_deg << ", ";
        file << report.range_m / 1000.0 << ", ";
        
        if (report.type == TargetReport::SourceType::RBS) {
            file << std::oct << report.rbs.mode3a_code << ", ";
            file << "RBS, ";
        } else {
            file << std::hex << report.uvd.raw_data20 << ", ";
            file << "UVD, ";
        }
        
        file << std::dec << static_cast<int>(report.signal_strength) << "\n";
    }
    
    file.close();
    VRL_LOG_INFO(modules::MAIN, "Saved " + std::to_string(all_reports_.size()) + 
                 " reports to: " + config_.output_file);
    
    // Сохраняем треки отдельно
    if (!config_.plots_output_file.empty()) {
        std::ofstream track_file(config_.plots_output_file);
        if (track_file.is_open()) {
            track_file << "# VRL-Radar Tracks Output\n";
            track_file << "# Format: id, x, y, speed, course, state\n\n";
            
            auto tracks = track_manager_->get_active_tracks();
            for (const auto& track : tracks) {
                track_file << track.id << ", ";
                track_file << std::fixed << std::setprecision(1);
                track_file << track.x << ", " << track.y << ", ";
                track_file << track.ground_speed << ", " << track.course_deg << ", ";
                
                switch (track.state) {
                    case TrackState::NEW: track_file << "NEW"; break;
                    case TrackState::ACTIVE: track_file << "ACTIVE"; break;
                    case TrackState::COASTING: track_file << "COASTING"; break;
                    case TrackState::DROPPED: track_file << "DROPPED"; break;
                }
                track_file << "\n";
            }
            track_file.close();
            VRL_LOG_INFO(modules::MAIN, "Saved " + std::to_string(tracks.size()) + 
                         " tracks to: " + config_.plots_output_file);
        }
    }
}

// ============================================================================
// СТАТИСТИКА
// ============================================================================

void RadarProcessor::print_stats() {
    std::stringstream ss;
    ss << "\n";
    ss << "=== Processing Statistics ===\n";
    ss << "Scans processed:       " << stats_.scans_processed << "\n";
    ss << "Replies processed:     " << stats_.replies_processed << "\n";
    ss << "Clusters formed:       " << stats_.clusters_formed << "\n";
    ss << "Reports generated:     " << stats_.reports_generated << "\n";
    ss << "Tracks created:        " << stats_.tracks_created << "\n";
    ss << "Tracks confirmed:      " << stats_.tracks_confirmed << "\n";
    ss << "Processing time:       " << stats_.processing_time_ms << " ms\n";
    ss << "================================\n";
    
    VRL_LOG_INFO(modules::MAIN, ss.str());
}

void RadarProcessor::shutdown() {
    VRL_LOG_INFO(modules::MAIN, "Shutting down processor...");
}

// ============================================================================
// ПАРСИНГ АРГУМЕНТОВ
// ============================================================================

CombinedConfig parse_arguments(int argc, char* argv[]) {
    CombinedConfig config;
    
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
        } else if (arg == "--dbscan-min-points" && i + 1 < argc) {
            config.dbscan_min_points = std::stoi(argv[++i]);
        } else if (arg == "--dbscan-az-coeff" && i + 1 < argc) {
            config.dbscan_azimuth_gap_coefficient = std::stod(argv[++i]);
        } else if (arg == "--legacy-gap" && i + 1 < argc) {
            config.legacy_max_gap_azimuth = std::stoi(argv[++i]);
        } else if (arg == "--legacy-window" && i + 1 < argc) {
            config.legacy_range_window = std::stoi(argv[++i]);
        } else if (arg == "--min-hits" && i + 1 < argc) {
            config.min_hits = std::stoi(argv[++i]);
        } else if (arg == "--min-confidence" && i + 1 < argc) {
            config.min_confidence = std::stod(argv[++i]);
        } else if (arg == "--min-hits-to-confirm" && i + 1 < argc) {
            config.min_hits_to_confirm = std::stoi(argv[++i]);
        } else if (arg == "--max-coast" && i + 1 < argc) {
            config.max_coast_count = std::stoi(argv[++i]);
        } else if (arg == "--gate-distance" && i + 1 < argc) {
            config.max_gate_distance = std::stod(argv[++i]);
        } else if (arg == "--gate-azimuth" && i + 1 < argc) {
            config.max_gate_azimuth = std::stod(argv[++i]);
        } else if (arg == "--verbose" || arg == "-v") {
            config.verbose = true;
        } else if (arg == "--debug" || arg == "-d") {
            config.debug = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  --config FILE           Config file\n"
                      << "  --input FILE            Input data file (optional)\n"
                      << "  --output FILE           Output targets file\n"
                      << "  --plots FILE            Output tracks file\n\n"
                      << "  --clusterer TYPE        Clusterer type: dbscan or legacy (default: dbscan)\n\n"
                      << "  DBSCAN options:\n"
                      << "  --dbscan-range-gap N    Max range gap in bins (default: 3)\n"
                      << "  --dbscan-min-points N   Min points for cluster (default: 2)\n"
                      << "  --dbscan-az-coeff N    Azimuth gap coefficient (default: 1.2)\n\n"
                      << "  Legacy options:\n"
                      << "  --legacy-gap N          Max gap azimuth (default: 8)\n"
                      << "  --legacy-window N       Range window (default: 30)\n\n"
                      << "  Processing:\n"
                      << "  --min-hits N            Min hits for report (default: 2)\n"
                      << "  --min-confidence N      Min confidence (default: 0.3)\n\n"
                      << "  Tracker:\n"
                      << "  --min-hits-to-confirm N Min hits to confirm track (default: 3)\n"
                      << "  --max-coast N           Max coast count (default: 5)\n"
                      << "  --gate-distance N       Max gate distance (default: 150)\n"
                      << "  --gate-azimuth N        Max gate azimuth (default: 30)\n\n"
                      << "  General:\n"
                      << "  --verbose, -v           Verbose output\n"
                      << "  --debug, -d             Debug mode (trace logging)\n"
                      << "  --help, -h              Show this help\n";
            exit(0);
        }
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
    CombinedConfig config = parse_arguments(argc, argv);
    
    // Создаем и запускаем процессор
    RadarProcessor processor;
    if (!processor.init(config)) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize processor");
        return 1;
    }
    
    processor.run();
    processor.shutdown();
    
    VRL_LOG_INFO(modules::MAIN, "Done.");
    return 0;
}
