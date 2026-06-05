// file: include/radar/radar_system.h
#pragma once

#include "cluster_tracker.h"
#include "cluster_processor.h"
#include "simulator.h"
#include <string>
#include <fstream>
#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <algorithm>  // для std::sort

namespace radar {

// Конфигурация всей системы
struct SystemConfig {
    // Параметры радара
    RadarConfig radar;
    
    // Параметры имитатора (для тестирования)
    SimulatorConfig simulator;
    
    // Параметры обработки
    struct ProcessingConfig {
        int max_gap_azimuth{8};        // макс. пропуск по азимуту для кластера
        int range_window{30};           // окно по дальности для кластеризации
        uint16_t range_tolerance{5};    // допуск по дальности для группировки
        int min_hits{2};                 // мин. число попаданий для цели
        std::string output_file{"targets.txt"}; // файл для вывода результатов
    } processing;
    
    // Загрузить конфигурацию из файла
    static SystemConfig load_from_file(const std::string& filename);
    
    // Сохранить конфигурацию в файл
    void save_to_file(const std::string& filename) const;
};

// Класс, объединяющий всю систему
class RadarSystem {
public:
    explicit RadarSystem(const SystemConfig& config);
    
    // Инициализация системы
    bool initialize();
    
    // Обработка одного зондирования
    void process_scan(const ScanReplies& scan);
    
    // Завершение работы (сброс всех кластеров)
    void shutdown();
    
    // Генерация тестовых данных (если нет реального источника)
    ScanReplies generate_test_scan(uint16_t azimuth, uint32_t timestamp);
    
    // Установка callback для получения плотов
    using TargetCallback = std::function<void(const TargetReport&)>;
    void set_target_callback(TargetCallback callback) { target_callback_ = callback; }
    
    // Получить статистику
    struct Statistics {
        uint32_t scans_processed{0};
        uint32_t clusters_completed{0};
        uint32_t targets_reported{0};
        uint32_t rbs_targets{0};
        uint32_t uvd_targets{0};
        uint32_t garbled_targets{0};
        uint32_t north_markers{0};      // счётчик меток Север
        
        void print() const;
    };
    
    const Statistics& get_statistics() const { return stats_; }
    
private:
    // Обработка завершённого кластера
    void on_cluster_completed(const TargetCluster& cluster);
    
    // Вывод плота в файл
    void write_target_to_file(const TargetReport& target);
    
    // Вывод метки Север в файл
    void write_north_marker(uint32_t timestamp, uint16_t azimuth);
    
    // Сортировка и вывод накопленных плотов
    void sort_and_flush_pending_targets();
    
    SystemConfig config_;
    ClusterTracker tracker_;
    ClusterProcessor processor_;
    std::unique_ptr<ReplySimulator> simulator_;
    
    std::ofstream output_file_;
    TargetCallback target_callback_;
    
    Statistics stats_;
    uint32_t scan_counter_{0};
    uint16_t last_azimuth_{0};          // последний обработанный азимут
    
    std::vector<TargetReport> pending_targets_;  // буфер для сортировки
};

} // namespace radar