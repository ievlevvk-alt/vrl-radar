// file: examples/stream_processing_example.cpp
#include "radar/stream_grouper.h"
#include "radar/simulator.h"
#include <iostream>
#include <memory>

using namespace radar;

// Обработчик для RBS кластеров
void rbs_cluster_callback(const ProcessedCluster<RBSReply>& cluster) {
    std::cout << "\n=== RBS CLUSTER READY ===\n";
    std::cout << "Scan #" << cluster.replies[0].azimuth << "\n";
    std::cout << "Position: az=" << cluster.avg_azimuth 
              << " (" << cluster.avg_azimuth * 0.08789 << "°), "
              << "range=" << cluster.avg_range << " bins\n";
    std::cout << "Size: " << cluster.replies.size() << " replies\n";
    std::cout << "Span: az=" << cluster.azimuth_span 
              << " bins, range=" << cluster.range_span << " bins\n";
    
    if (cluster.has_overlaps) {
        std::cout << "⚠️  OVERLAP DETECTED! Need garbling processing\n";
        
        // Здесь будет вызов алгоритма разделения перекрытий
        for (const auto& reply : cluster.replies) {
            std::cout << "  Reply: az=" << reply.azimuth 
                      << ", range=" << reply.range 
                      << ", code=0x" << std::hex << reply.code12 << std::dec;
            
            // Проверяем SLS
            if (reply.ether_amplitudes_sls[0] > 0) {
                double ratio = static_cast<double>(reply.ether_amplitudes_sls[0]) / 
                              static_cast<double>(reply.ether_amplitudes[0]);
                if (ratio > 1.5) {
                    std::cout << " (SIDELOBE)";
                }
            }
            std::cout << "\n";
        }
    } else {
        // Чистый кластер - можно сразу формировать цель
        std::cout << "✓ Clean cluster - generating target report\n";
        // Берём ответ с максимальной амплитудой как основной
        const RBSReply* best = &cluster.replies[0];
        for (const auto& r : cluster.replies) {
            if (r.f1() + r.f2() > best->f1() + best->f2()) {
                best = &r;
            }
        }
        std::cout << "  Target: code=0x" << std::hex << best->code12 
                  << std::dec << ", SPI=" << best->spi << "\n";
    }
}

// Обработчик для UVD кластеров
void uvd_cluster_callback(const ProcessedCluster<UVDReply>& cluster) {
    std::cout << "\n=== UVD CLUSTER READY ===\n";
    std::cout << "Position: az=" << cluster.avg_azimuth << ", range=" << cluster.avg_range << "\n";
    std::cout << "Size: " << cluster.replies.size() << " replies\n";
    
    if (cluster.has_overlaps) {
        std::cout << "⚠️  UVD OVERLAP DETECTED\n";
    } else {
        // Декодируем УВД информацию
        const auto& reply = cluster.replies[0];
        uint32_t data = reply.data20;
        uint32_t error_mask = reply.error_mask;
        
        std::cout << "  UVD Data: 0x" << std::hex << data << std::dec << "\n";
        std::cout << "  Error mask: 0x" << std::hex << error_mask << std::dec << "\n";
        
        // Простейшее декодирование (реальное будет сложнее)
        if (error_mask == 0) {
            uint16_t altitude = data & 0x7FF;  // предположительно
            std::cout << "  Altitude: " << altitude * 100 << " ft\n";
        }
    }
}

int main() {
    std::cout << "Starting Radar Stream Processing Example\n";
    std::cout << "========================================\n";
    
    // Конфигурация радара
    RadarConfig radar_cfg;
    radar_cfg.range_bin_rbs = 30.0;        // 30 метров на дискрет
    radar_cfg.max_azimuth_diff_for_overlap = 2;
    radar_cfg.max_range_diff_for_overlap = 10;
    
    // Ширина диаграммы направленности в дискретах (40 ≈ 3.5°)
    double beamwidth_bins = 40.0;
    
    // Создаём группировщики для разных типов
    RBSStreamGrouper rbs_grouper(radar_cfg, beamwidth_bins, rbs_cluster_callback);
    UVDStreamGrouper uvd_grouper(radar_cfg, beamwidth_bins, uvd_cluster_callback);
    
    // Создаём имитатор для генерации тестовых данных
    SimulatorConfig sim_cfg;
    sim_cfg.radar = radar_cfg;
    sim_cfg.rbs.snr_db = 30.0;
    sim_cfg.sls.enabled = true;
    sim_cfg.sls.sidelobe_probability = 0.2;
    
    ReplySimulator simulator(sim_cfg);
    
    // Информация о сканировании
    ScanInfo scan;
    scan.scan_number = 1;
    
    std::cout << "\n--- Starting Scan #1 ---\n";
    
    // Симулируем вращение антенны с шагом 2 дискрета
    for (int az = 0; az < 4096; az += 2) {
        scan.azimuth = az;
        scan.scan_start = (az == 0);
        scan.scan_end = (az >= 4090);
        
        // Генерируем ответы в разных секторах
        
        // Сектор 100-200: одиночные RBS ответы
        if (az >= 100 && az <= 200 && az % 20 == 0) {
            auto reply = simulator.generate_rbs(az, 500 + (az % 100), 0xAAA, false);
            rbs_grouper.process_reply(reply, scan);
        }
        
        // Сектор 500-550: группа RBS с перекрытием
        if (az >= 500 && az <= 550 && az % 5 == 0) {
            // Генерируем два близких ответа
            auto r1 = simulator.generate_rbs(az, 800, 0x123, false);
            auto r2 = simulator.generate_rbs(az + 1, 805, 0x456, false);
            rbs_grouper.process_reply(r1, scan);
            rbs_grouper.process_reply(r2, scan);
        }
        
        // Сектор 1000-1100: UVD ответы
        if (az >= 1000 && az <= 1100 && az % 15 == 0) {
            auto reply = simulator.generate_uvd(az, 1200, 0x54321);
            uvd_grouper.process_reply(reply, scan);
        }
        
        // Сектор 3000-3100: ответы с SLS (боковые лепестки)
        if (az >= 3000 && az <= 3100 && az % 10 == 0) {
            auto reply = simulator.generate_rbs(az, 2000, 0x789, az % 30 == 0);
            rbs_grouper.process_reply(reply, scan);
        }
    }
    
    // Завершаем обработку
    scan.scan_end = true;
    rbs_grouper.flush();
    uvd_grouper.flush();
    
    std::cout << "\n========================================\n";
    std::cout << "Stream Processing Example Completed\n";
    
    return 0;
}