// tools/2_3_combined_v2_manual.cpp
// Manual control tool for V2 architecture:
// - Full initialization with default config
// - No file I/O
// - Empty loop for manual control
// - All components ready for interactive debugging

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

struct ManualConfig {
    // Режимы
    bool verbose{true};
    bool debug{true};
    
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
// КЛАСС MANUAL PROCESSOR V2
// ============================================================================

class ManualProcessorV2 {
public:
    ManualProcessorV2() = default;
    ~ManualProcessorV2() = default;
    
    bool init(const ManualConfig& config);
    void run_loop();
    void shutdown();
    
    // === ПУБЛИЧНЫЕ МЕТОДЫ ДЛЯ РУЧНОГО УПРАВЛЕНИЯ ===
    
    /**
     * @brief Добавить RBS точку в буфер и обработать
     * @param azimuth азимут (0-4095)
     * @param range дальность (бины)
     * @param code12 код Mode 3/A (восьмеричный)
     * @param spi флаг SPI
     */
    void add_rbs_point(uint16_t azimuth, uint16_t range, uint16_t code12 = 01234, bool spi = false);
    
    /**
     * @brief Добавить UVD точку в буфер и обработать
     * @param azimuth азимут (0-4095)
     * @param range дальность (бины)
     * @param data20 данные UVD (20 бит)
     */
    void add_uvd_point(uint16_t azimuth, uint16_t range, uint32_t data20 = 12345);
    
    /**
     * @brief Обработать текущий азимут (вызвать кластеризацию и трекинг)
     * @param azimuth азимут (0-4095)
     */
    void process_azimuth(uint16_t azimuth);
    
    /**
     * @brief Получить текущие треки
     * @return вектор плотов
     */
    std::vector<Plot> get_tracks() const;
    
    /**
     * @brief Получить статистику TrackManager
     */
    TrackManager::Stats get_stats() const;
    
    /**
     * @brief Вывести статистику в консоль
     */
    void print_stats() const;
    
    /**
     * @brief Очистить все данные (сброс)
     */
    void reset_all();
    
    /**
     * @brief Получить доступ к TrackManager (для продвинутого использования)
     */
    TrackManager* get_track_manager() { return track_manager_.get(); }
    
    /**
     * @brief Получить доступ к Clusterer (для продвинутого использования)
     */
    IClusterer* get_clusterer() { return clusterer_.get(); }
    
private:
    bool load_default_config();
    bool init_clusterer();
    bool init_track_manager();
    void print_help() const;
    
    ManualConfig config_;
    SystemConfig system_config_;
    
    std::unique_ptr<IClusterer> clusterer_;
    std::unique_ptr<TrackManager> track_manager_;
    
    // Статистика
    struct Stats {
        uint32_t points_added{0};
        uint32_t scans_processed{0};
        uint32_t clusters_formed{0};
        uint32_t tracks_created{0};
        uint32_t tracks_confirmed{0};
        uint32_t tracks_updated{0};
        uint32_t tracks_coasted{0};
        uint32_t tracks_dropped{0};
        uint64_t current_maia{0};
    } stats_;
    
    uint32_t revolution_counter_{0};
    uint64_t scan_counter_{0};
    uint16_t previous_azimuth_{0};
    bool first_scan_{true};
};

// ============================================================================
// РЕАЛИЗАЦИЯ
// ============================================================================

bool ManualProcessorV2::init(const ManualConfig& config) {
    config_ = config;
    
    // Настройка логгера
    if (config_.verbose) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::DEBUG);
        Logger::instance().set_module_level(modules::TRACKER, LogLevel::DEBUG);
        Logger::instance().set_module_level(modules::CLUSTER, LogLevel::DEBUG);
    } else if (config_.debug) {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::TRACE);
        Logger::instance().set_module_level(modules::TRACKER, LogLevel::TRACE);
        Logger::instance().set_module_level(modules::CLUSTER, LogLevel::TRACE);
    } else {
        Logger::instance().set_module_level(modules::MAIN, LogLevel::INFO);
        Logger::instance().set_module_level(modules::TRACKER, LogLevel::INFO);
        Logger::instance().set_module_level(modules::CLUSTER, LogLevel::INFO);
    }
    
    VRL_LOG_INFO(modules::MAIN, "=== VRL-Radar Manual Processor V2 ===");
    VRL_LOG_INFO(modules::MAIN, "Clusterer type: " + config_.clusterer_type);
    VRL_LOG_INFO(modules::MAIN, "Revolution time: " + std::to_string(config_.revolution_time_s) + "s");
    VRL_LOG_INFO(modules::MAIN, "Max coast revolutions: " + std::to_string(config_.max_coast_revolutions));
    
    // Загружаем конфигурацию по умолчанию
    if (!load_default_config()) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to load default config");
        return false;
    }
    
    // Инициализируем PointBuffer
    PointBuffer::instance().init(65536);
    VRL_LOG_INFO(modules::MAIN, "PointBuffer initialized (65536 slots)");
    
    // Инициализируем ClusterPool
    ClusterPool::instance().init(65535);
    VRL_LOG_INFO(modules::MAIN, "ClusterPool initialized (65535 slots)");
    
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
    
    VRL_LOG_INFO(modules::MAIN, "Manual Processor V2 initialized successfully");
    print_help();
    
    return true;
}

bool ManualProcessorV2::load_default_config() {
    // Конфигурация по умолчанию
    system_config_.radar.range_bin_rbs = 30.0;
    system_config_.radar.range_bin_uvd = 60.0;
    system_config_.radar.beamwidth_deg = 5.0;
    system_config_.radar.min_amplitude = 10;
    system_config_.beamwidth_deg = 5.0;
    system_config_.revolution_time = config_.revolution_time_s;
    
    VRL_LOG_DEBUG(modules::MAIN, "Using default configuration");
    return true;
}

bool ManualProcessorV2::init_clusterer() {
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

bool ManualProcessorV2::init_track_manager() {
    GridConfig grid_config;
    grid_config.cell_size_km = config_.cell_size_km;
    grid_config.max_range_km = config_.max_range_km;
    grid_config.rings_near = config_.rings_near;
    grid_config.rings_far = config_.rings_far;
    grid_config.far_threshold_km = config_.far_threshold_km;
    grid_config.max_candidate_distance_km = config_.max_candidate_distance_km;
    grid_config.revolution_time_s = config_.revolution_time_s;
    
    grid_config.range_gate_bins_near = config_.range_gate_bins_near;
    grid_config.range_gate_bins_mid = config_.range_gate_bins_mid;
    grid_config.range_gate_bins_far = config_.range_gate_bins_far;
    grid_config.azimuth_gate_maia_near = config_.azimuth_gate_maia_near;
    grid_config.azimuth_gate_maia_mid = config_.azimuth_gate_maia_mid;
    grid_config.azimuth_gate_maia_far = config_.azimuth_gate_maia_far;
    grid_config.coast_gate_expansion = config_.coast_gate_expansion;
    grid_config.max_coast_revolutions = config_.max_coast_revolutions;
    
    VRL_LOG_INFO(modules::MAIN, "TrackManager V2 config:");
    VRL_LOG_INFO(modules::MAIN, "  cell_size=" + std::to_string(grid_config.cell_size_km) + "km");
    VRL_LOG_INFO(modules::MAIN, "  max_range=" + std::to_string(grid_config.max_range_km) + "km");
    VRL_LOG_INFO(modules::MAIN, "  coast_max=" + std::to_string(grid_config.max_coast_revolutions) + " revs");
    
    track_manager_ = std::make_unique<TrackManager>();
    track_manager_->init(grid_config);
    
    return true;
}

// ============================================================================
// РУЧНОЕ УПРАВЛЕНИЕ
// ============================================================================

void ManualProcessorV2::add_rbs_point(uint16_t azimuth, uint16_t range, 
                                       uint16_t code12, bool spi) {
    // Добавляем точку в буфер
    StoredPoint point;
    point.azimuth = azimuth;
    point.range = range;
    point.is_rbs = true;
    point.code12 = code12;
    point.spi = spi;
    point.amplitude = 100;
    
    PointBuffer::instance().add_point(point);
    stats_.points_added++;
    
    // Создаем RBSReply для кластеризатора
    RBSReply reply;
    reply.azimuth = azimuth;
    reply.range = range;
    reply.code12 = code12;
    reply.spi = spi;
    reply.is_valid = true;
    
    // Заполняем амплитуды
    reply.ether_amplitudes[0] = 100;   // F1
    reply.ether_amplitudes[14] = 100;  // F2
    for (int j = 0; j < 12; ++j) {
        if (code12 & (1 << j)) {
            reply.ether_amplitudes[utils::bit_position(j)] = 100;
        }
    }
    if (spi) {
        reply.ether_amplitudes[17] = 100;
    }
    
    // Создаем скан с одним ответом
    ScanReplies scan(azimuth, 0);
    scan.rbs_replies.push_back(reply);
    
    // Кластеризация
    clusterer_->process_scan(scan);
    
    // Трекинг
    process_azimuth(azimuth);
}

void ManualProcessorV2::add_uvd_point(uint16_t azimuth, uint16_t range, uint32_t data20) {
    // Добавляем точку в буфер
    StoredPoint point;
    point.azimuth = azimuth;
    point.range = range;
    point.is_rbs = false;
    point.data20 = data20;
    point.amplitude = 100;
    
    PointBuffer::instance().add_point(point);
    stats_.points_added++;
    
    // Создаем UVDReply для кластеризатора
    UVDReply reply;
    reply.azimuth = azimuth;
    reply.range = range;
    reply.data20 = data20;
    reply.is_valid = true;
    
    // Заполняем амплитуды (упрощённо)
    for (int i = 0; i < 20; ++i) {
        bool bit_value = (data20 >> i) & 1;
        if (bit_value) {
            reply.ether_amplitudes[i * 2 + 1] = 100;
            reply.ether_amplitudes[40 + i * 2 + 1] = 100;
        } else {
            reply.ether_amplitudes[i * 2] = 100;
            reply.ether_amplitudes[40 + i * 2] = 100;
        }
    }
    
    // Создаем скан с одним ответом
    ScanReplies scan(azimuth, 0);
    scan.uvd_replies.push_back(reply);
    
    // Кластеризация
    clusterer_->process_scan(scan);
    
    // Трекинг
    process_azimuth(azimuth);
}

void ManualProcessorV2::process_azimuth(uint16_t azimuth) {
    // Определяем смену оборота
    if (!first_scan_ && azimuth < previous_azimuth_) {
        revolution_counter_++;
        VRL_LOG_DEBUG(modules::MAIN, "=== Revolution " + std::to_string(revolution_counter_) + " ===");
    }
    previous_azimuth_ = azimuth;
    first_scan_ = false;
    
    stats_.scans_processed++;
    
    // Обработка треков
    track_manager_->process_azimuth(azimuth);
    
    // Обновляем статистику
    auto track_stats = track_manager_->get_stats();
    stats_.tracks_created = track_stats.total_tracks;
    stats_.tracks_confirmed = track_stats.confirmed_tracks;
    stats_.tracks_coasted = track_stats.coasting_tracks;
    
    // Получаем обновленные треки
    const auto& updated = track_manager_->get_updated_tracks();
    stats_.tracks_updated += updated.size();
    
    if (!updated.empty() && config_.verbose) {
        VRL_LOG_DEBUG(modules::MAIN, "Updated " + std::to_string(updated.size()) + " tracks");
    }
    
    // Очищаем список обновлений
    track_manager_->clear_updated_tracks();
    
    // Обновляем глобальный счетчик MAIA
    stats_.current_maia = track_manager_->get_global_maia_counter() + azimuth;
}

std::vector<Plot> ManualProcessorV2::get_tracks() const {
    return track_manager_->get_plots();
}

TrackManager::Stats ManualProcessorV2::get_stats() const {
    return track_manager_->get_stats();
}

void ManualProcessorV2::print_stats() const {
    auto track_stats = track_manager_->get_stats();
    
    std::stringstream ss;
    ss << "\n";
    ss << "=== Manual Processor Statistics ===\n";
    ss << "Points added:          " << stats_.points_added << "\n";
    ss << "Scans processed:       " << stats_.scans_processed << "\n";
    ss << "Revolutions:           " << revolution_counter_ << "\n";
    ss << "Current MAIA:          " << stats_.current_maia << "\n";
    ss << "Total tracks:          " << track_stats.total_tracks << "\n";
    ss << "Active tracks:         " << track_stats.active_tracks << "\n";
    ss << "Confirmed tracks:      " << track_stats.confirmed_tracks << "\n";
    ss << "Coasting tracks:       " << track_stats.coasting_tracks << "\n";
    ss << "Tracks updated:        " << stats_.tracks_updated << "\n";
    ss << "==================================\n";
    
    VRL_LOG_INFO(modules::MAIN, ss.str());
}

void ManualProcessorV2::reset_all() {
    track_manager_->reset();
    TrackPool::instance().clear();
    PlotPool::instance().clear();
    ClusterPool::instance().clear();
    PointBuffer::instance().init(65536);
    
    stats_ = Stats{};
    revolution_counter_ = 0;
    scan_counter_ = 0;
    previous_azimuth_ = 0;
    first_scan_ = true;
    
    VRL_LOG_INFO(modules::MAIN, "All components reset");
}

void ManualProcessorV2::print_help() const {
    std::cout << "\n";
    std::cout << "=== MANUAL PROCESSOR V2 - HELP ===\n";
    std::cout << "\n";
    std::cout << "Available methods:\n";
    std::cout << "  add_rbs_point(azimuth, range, code12=01234, spi=false)\n";
    std::cout << "    - Add RBS point and process\n";
    std::cout << "    - Example: processor.add_rbs_point(512, 100, 01234, false)\n";
    std::cout << "\n";
    std::cout << "  add_uvd_point(azimuth, range, data20=12345)\n";
    std::cout << "    - Add UVD point and process\n";
    std::cout << "    - Example: processor.add_uvd_point(512, 100, 12345)\n";
    std::cout << "\n";
    std::cout << "  process_azimuth(azimuth)\n";
    std::cout << "    - Process current azimuth (for manual scanning)\n";
    std::cout << "    - Example: processor.process_azimuth(512)\n";
    std::cout << "\n";
    std::cout << "  get_tracks()\n";
    std::cout << "    - Get current tracks as plots\n";
    std::cout << "    - Example: auto plots = processor.get_tracks()\n";
    std::cout << "\n";
    std::cout << "  get_stats()\n";
    std::cout << "    - Get TrackManager statistics\n";
    std::cout << "    - Example: auto stats = processor.get_stats()\n";
    std::cout << "\n";
    std::cout << "  print_stats()\n";
    std::cout << "    - Print statistics to console\n";
    std::cout << "    - Example: processor.print_stats()\n";
    std::cout << "\n";
    std::cout << "  reset_all()\n";
    std::cout << "    - Reset all components\n";
    std::cout << "    - Example: processor.reset_all()\n";
    std::cout << "\n";
    std::cout << "  get_track_manager()\n";
    std::cout << "    - Get TrackManager pointer (advanced)\n";
    std::cout << "    - Example: auto tm = processor.get_track_manager()\n";
    std::cout << "\n";
    std::cout << "  get_clusterer()\n";
    std::cout << "    - Get Clusterer pointer (advanced)\n";
    std::cout << "    - Example: auto cl = processor.get_clusterer()\n";
    std::cout << "\n";
    std::cout << "================================\n";
    std::cout << "\n";
    std::cout << "Type 'exit' to quit, or enter GDB commands.\n";
    std::cout << "\n";
}

// ============================================================================
// ОСНОВНОЙ ЦИКЛ (ПУСТОЙ, ДЛЯ РУЧНОГО УПРАВЛЕНИЯ)
// ============================================================================

void ManualProcessorV2::run_loop() {
    VRL_LOG_INFO(modules::MAIN, "Entering manual control loop...");
    VRL_LOG_INFO(modules::MAIN, "Type 'help' for available commands");
    VRL_LOG_INFO(modules::MAIN, "Use GDB to set breakpoints and call methods");
    
    // Пустой цикл - все управление через GDB/интерактивный отладчик
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ManualProcessorV2::shutdown() {
    VRL_LOG_INFO(modules::MAIN, "Shutting down manual processor...");
}

// ============================================================================
// СИГНАЛЫ
// ============================================================================

void signal_handler(int sig) {
    (void)sig;
    VRL_LOG_INFO(modules::MAIN, "Received interrupt signal, shutting down...");
    g_running = false;
}

// ============================================================================
// ТОЧКА ВХОДА
// ============================================================================

int main(int argc, char* argv[]) {
    // Настройка обработчика сигналов
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Создаем конфигурацию по умолчанию
    ManualConfig config;
    
    // Разбор аргументов командной строки (опционально)
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "--clusterer" && i + 1 < argc) {
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
        } else if (arg == "--rev-time" && i + 1 < argc) {
            config.revolution_time_s = std::stod(argv[++i]);
        } else if (arg == "--quiet" || arg == "-q") {
            config.verbose = false;
            config.debug = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n\n"
                      << "Options:\n"
                      << "  --clusterer TYPE        Clusterer: dbscan or legacy (default: dbscan)\n"
                      << "  --dbscan-range-gap N    Max range gap in bins (default: 3)\n"
                      << "  --dbscan-az-coeff N    Azimuth gap coefficient (default: 1.2)\n"
                      << "  --legacy-gap N          Max gap azimuth (default: 8)\n"
                      << "  --legacy-window N       Range window (default: 30)\n"
                      << "  --cell-size N           Grid cell size in km (default: 5)\n"
                      << "  --max-range N           Max range in km (default: 400)\n"
                      << "  --max-coast N           Max coast revolutions (default: 3)\n"
                      << "  --rev-time N            Revolution time in seconds (default: 5)\n"
                      << "  --quiet, -q             Quiet mode\n"
                      << "  --help, -h              Show this help\n"
                      << "\n"
                      << "This tool is designed for manual control via GDB.\n"
                      << "Set breakpoints and call methods on the processor object.\n";
            return 0;
        }
    }
    
    // Создаем и запускаем процессор
    ManualProcessorV2 processor;
    if (!processor.init(config)) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to initialize processor");
        return 1;
    }
    
    // Для удобства в GDB - сохраняем указатель
    std::cout << "Processor initialized. Use 'processor' in GDB.\n";
    std::cout << "Example: processor.add_rbs_point(512, 100, 01234, false)\n";
    std::cout << "Type Ctrl+C to exit.\n";
    
    processor.run_loop();
    processor.shutdown();
    
    VRL_LOG_INFO(modules::MAIN, "Done.");
    return 0;
}
