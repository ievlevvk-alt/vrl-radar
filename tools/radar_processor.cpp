// file: tools/radar_processor.cpp
#include "radar/radar_system.h"
#include "radar/simulator.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

using namespace radar;

// Глобальный указатель на систему для обработки сигналов
RadarSystem* g_system = nullptr;

void signal_handler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    if (g_system) {
        g_system->shutdown();
    }
    exit(signum);
}

void print_usage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n";
    std::cout << "Options:\n";
    std::cout << "  -c <file>    Configuration file (default: radar.conf)\n";
    std::cout << "  -o <file>    Output file (overrides config)\n";
    std::cout << "  -t <ms>      Scan interval in milliseconds (default: 100)\n";
    std::cout << "  -n <scans>   Number of scans to process (default: 5000)\n";
    std::cout << "  -r <revs>    Number of revolutions (default: 2, overrides -n)\n";
    std::cout << "  -g           Generate config file template\n";
    std::cout << "  -h           Show this help\n";
}

void generate_config_template(const std::string& filename) {
    SystemConfig config;
    config.save_to_file(filename);
    std::cout << "Configuration template saved to " << filename << "\n";
}

int main(int argc, char* argv[]) {
    std::string config_file = "radar.conf";
    std::string output_file;
    int scan_interval_ms = 100;
    int max_scans = 5000;  // увеличен лимит
    int num_revolutions = 0;
    bool generate_template = false;
    
    // Парсинг аргументов командной строки
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-c" && i + 1 < argc) {
            config_file = argv[++i];
        } else if (arg == "-o" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "-t" && i + 1 < argc) {
            scan_interval_ms = std::stoi(argv[++i]);
        } else if (arg == "-n" && i + 1 < argc) {
            max_scans = std::stoi(argv[++i]);
        } else if (arg == "-r" && i + 1 < argc) {
            num_revolutions = std::stoi(argv[++i]);
        } else if (arg == "-g") {
            generate_template = true;
        } else if (arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }
    
    if (generate_template) {
        generate_config_template(config_file);
        return 0;
    }
    
    // Загружаем конфигурацию
    std::cout << "Loading configuration from " << config_file << "\n";
    auto config = SystemConfig::load_from_file(config_file);
    
    // Устанавливаем параметры для тестирования
    config.processing.max_gap_azimuth = 8;
    
    // Переопределяем выходной файл, если указан в командной строке
    if (!output_file.empty()) {
        config.processing.output_file = output_file;
    }
    
    // Создаём систему
    RadarSystem system(config);
    g_system = &system;
    
    // Устанавливаем обработчик сигналов
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Инициализация
    if (!system.initialize()) {
        std::cerr << "Failed to initialize radar system\n";
        return 1;
    }
    
    // Устанавливаем callback для отображения плотов в реальном времени
    system.set_target_callback([](const TargetReport& target) {
        static int target_count = 0;
        target_count++;
        std::cout << "\rTargets: " << target_count << "    " << std::flush;
    });
    
    // Определяем количество сканов для обработки
    int scans_to_process = max_scans;
    if (num_revolutions > 0) {
        scans_to_process = num_revolutions * 2048;  // 2048 сканов на оборот (шаг 2)
        std::cout << "Processing " << num_revolutions << " revolutions (" 
                  << scans_to_process << " scans)\n";
    } else {
        std::cout << "Processing " << max_scans << " scans\n";
    }
    
    std::cout << "Scan interval: " << scan_interval_ms << " ms\n";
    std::cout << "Max gap azimuth: " << config.processing.max_gap_azimuth << "\n";
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Основной цикл обработки
    for (int scan_num = 0; scan_num < scans_to_process; scan_num++) {
        uint16_t azimuth = (scan_num * 2) % 4096;  // шаг 2 дискрета
        uint32_t timestamp = scan_num * scan_interval_ms;
        
        // Каждые 100 сканов создаём пропуск для закрытия кластеров
        if (scan_num > 0 && scan_num % 100 == 0) {
            int gap = config.processing.max_gap_azimuth + 2;
            std::cout << "\nCreating gap of " << gap << " scans at azimuth " << azimuth << "\n";
            for (int i = 0; i < gap; i++) {
                azimuth = (azimuth + 2) % 4096;
                timestamp += scan_interval_ms;
                ScanReplies empty_scan(azimuth, timestamp);
                system.process_scan(empty_scan);
                scan_num++;
                if (scan_num >= scans_to_process) break;
            }
        }
        
        if (scan_num < scans_to_process) {
            auto scan = system.generate_test_scan(azimuth, timestamp);
            system.process_scan(scan);
        }
        
        // Небольшая задержка для имитации реального времени
        if (scan_interval_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    // Принудительно закрываем все кластеры в конце
    std::cout << "\nFlushing remaining clusters...\n";
    system.shutdown();
    
    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    std::cout << "\nProcessing completed in " << duration.count() << " ms\n";
    
    return 0;
}