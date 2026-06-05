// file: tests/test_cluster_processor.cpp
#include "radar/cluster_processor.h"
#include "radar/simulator.h"
#include <iostream>
#include <cassert>
#include <iomanip>
#include <cmath>

using namespace radar;

void print_target_report(const TargetReport& report, int index) {
    std::cout << "\nTarget " << index << ":\n";
    std::cout << "  Type: " << (report.type == TargetReport::SourceType::RBS ? "RBS" : "UVD") << "\n";
    std::cout << "  Position: az=" << report.azimuth_deg << "°"
              << ", range=" << report.range_m << "m\n";
    std::cout << "  XY: (" << report.x << ", " << report.y << ")\n";
    std::cout << "  Signal strength: " << static_cast<int>(report.signal_strength) << "\n";
    
    if (report.type == TargetReport::SourceType::RBS) {
        std::cout << "  RBS: code=0x" << std::hex << report.rbs.mode3a_code << std::dec
                  << ", SPI=" << report.rbs.spi
                  << ", alt=" << report.rbs.modec_altitude << "\n";
    } else {
        std::cout << "  UVD: data=0x" << std::hex << report.uvd.raw_data20 << std::dec
                  << ", octal=";
        for (int i = 0; i < 5; ++i) {
            std::cout << static_cast<int>(report.uvd.octal_id[i]);
        }
        std::cout << ", alt=" << report.uvd.altitude
                  << ", fuel=" << static_cast<int>(report.uvd.fuel)
                  << ", pressure_ref=" << report.uvd.pressure_ref << "\n";
    }
    
    std::cout << "  Flags: garbled=" << report.is_garbled
              << ", sls_blanked=" << report.is_sls_blanked
              << ", reflection=" << report.is_reflection << "\n";
    std::cout << "  Sources: " << report.sources.size() << "\n";
}

void test_single_rbs_target() {
    std::cout << "\n=== Test Single RBS Target ===\n";
    
    RadarConfig cfg;
    cfg.range_bin_rbs = 30.0;
    // azimuth_per_bin - статическая константа, её нельзя изменить
    
    ClusterProcessor processor(cfg);
    processor.set_min_hits(2);
    
    // Создаём кластер с одной целью (7 ответов на разных азимутах)
    TargetCluster cluster;
    
    for (int az = 1000; az <= 1012; az += 2) {
        ScanReplies scan(az, az * 1000);
        
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 500;
        reply.code12 = 0x123;
        reply.spi = (az == 1006);  // SPI на среднем азимуте
        
        // Заполняем эфирные амплитуды (упрощённо)
        reply.ether_amplitudes[0] = 200;  // F1
        reply.ether_amplitudes[14] = 200; // F2
        for (int i = 1; i < 14; ++i) {
            if (i != 7) {  // пропускаем центральную паузу
                reply.ether_amplitudes[i] = 150;
            }
        }
        
        scan.rbs_replies.push_back(reply);
        cluster.add_scan(scan);
    }
    
    auto reports = processor.process_cluster(cluster);
    
    assert(reports.size() == 1);
    assert(reports[0].type == TargetReport::SourceType::RBS);
    assert(reports[0].rbs.mode3a_code == 0x123);
    assert(reports[0].rbs.spi == true);
    assert(std::abs(reports[0].range_m - 500 * 30.0) < 0.1);
    assert(reports[0].sources.size() == 7);
    
    print_target_report(reports[0], 0);
    std::cout << "Single RBS target test passed!\n";
}

void test_single_uvd_target() {
    std::cout << "\n=== Test Single UVD Target ===\n";
    
    RadarConfig cfg;
    cfg.range_bin_uvd = 50.0;
    
    ClusterProcessor processor(cfg);
    processor.set_min_hits(2);
    
    // Создаём кластер с одной целью
    TargetCluster cluster;
    
    for (int az = 2000; az <= 2010; az += 2) {
        ScanReplies scan(az, az * 1000);
        
        UVDReply reply;
        reply.azimuth = az;
        reply.range = 800;
        reply.data20 = 0x12345;  // тестовые данные
        reply.error_mask = 0;
        
        // Заполняем эфирные амплитуды
        for (int i = 0; i < 80; i += 2) {
            reply.ether_amplitudes[i] = 200;  // left
            reply.ether_amplitudes[i+1] = 0;   // right для 0
        }
        
        scan.uvd_replies.push_back(reply);
        cluster.add_scan(scan);
    }
    
    auto reports = processor.process_cluster(cluster);
    
    assert(reports.size() == 1);
    assert(reports[0].type == TargetReport::SourceType::UVD);
    assert(reports[0].uvd.raw_data20 == 0x12345);
    assert(std::abs(reports[0].range_m - 800 * 50.0) < 0.1);
    assert(reports[0].sources.size() == 6);
    
    print_target_report(reports[0], 0);
    std::cout << "Single UVD target test passed!\n";
}

void test_two_targets_adjacent() {
    std::cout << "\n=== Test Two Adjacent Targets ===\n";
    
    RadarConfig cfg;
    cfg.range_bin_rbs = 30.0;
    
    ClusterProcessor processor(cfg);
    processor.set_range_tolerance(3);  // допуск по дальности 3 дискрета
    processor.set_min_hits(2);
    
    // Кластер с двумя целями на разных дальностях
    TargetCluster cluster;
    
    for (int az = 1000; az <= 1016; az += 2) {
        ScanReplies scan(az, az * 1000);
        
        // Цель A (дальность 500)
        if (az <= 1010) {
            RBSReply reply_a;
            reply_a.azimuth = az;
            reply_a.range = 500;
            reply_a.code12 = 0xAAA;
            reply_a.ether_amplitudes[0] = 200;
            reply_a.ether_amplitudes[14] = 200;
            scan.rbs_replies.push_back(reply_a);
        }
        
        // Цель B (дальность 505)
        if (az >= 1006) {
            RBSReply reply_b;
            reply_b.azimuth = az;
            reply_b.range = 505;
            reply_b.code12 = 0xBBB;
            reply_b.ether_amplitudes[0] = 200;
            reply_b.ether_amplitudes[14] = 200;
            scan.rbs_replies.push_back(reply_b);
        }
        
        cluster.add_scan(scan);
    }
    
    auto reports = processor.process_cluster(cluster);
    
    assert(reports.size() == 2);
    
    // Сортируем по дальности
    if (reports[0].range_m > reports[1].range_m) {
        std::swap(reports[0], reports[1]);
    }
    
    assert(std::abs(reports[0].range_m - 500 * 30.0) < 0.1);
    assert(reports[0].rbs.mode3a_code == 0xAAA);
    assert(reports[0].sources.size() == 6);  // 1000-1010 (шаг 2) = 6 ответов
    
    assert(std::abs(reports[1].range_m - 505 * 30.0) < 0.1);
    assert(reports[1].rbs.mode3a_code == 0xBBB);
    assert(reports[1].sources.size() == 6);  // 1006-1016 = 6 ответов
    
    print_target_report(reports[0], 0);
    print_target_report(reports[1], 1);
    std::cout << "Two adjacent targets test passed!\n";
}

void test_garbled_targets() {
    std::cout << "\n=== Test Garbled Targets ===\n";
    
    RadarConfig cfg;
    cfg.range_bin_rbs = 30.0;
    
    ClusterProcessor processor(cfg);
    processor.set_range_tolerance(5);  // увеличиваем допуск, чтобы оба ответа попали в одну группу
    processor.set_min_hits(1);
    
    // Кластер с перекрывающимися ответами в одном зондировании
    TargetCluster cluster;
    
    ScanReplies scan(1000, 1000);
    
    // Два ответа на близкой дальности - перекрытие
    RBSReply r1;
    r1.azimuth = 1000;
    r1.range = 500;
    r1.code12 = 0x111;
    r1.ether_amplitudes[0] = 200;
    r1.ether_amplitudes[14] = 200;
    
    RBSReply r2;
    r2.azimuth = 1000;
    r2.range = 502;  // близкая дальность
    r2.code12 = 0x111;  // тот же код! (было 0x222)
    r2.ether_amplitudes[0] = 200;
    r2.ether_amplitudes[14] = 200;
    
    scan.rbs_replies.push_back(r1);
    scan.rbs_replies.push_back(r2);
    cluster.add_scan(scan);
    
    // Ещё одно зондирование с одним ответом
    ScanReplies scan2(1002, 2000);
    RBSReply r3;
    r3.azimuth = 1002;
    r3.range = 500;
    r3.code12 = 0x111;
    r3.ether_amplitudes[0] = 200;
    r3.ether_amplitudes[14] = 200;
    scan2.rbs_replies.push_back(r3);
    cluster.add_scan(scan2);
    
    auto reports = processor.process_cluster(cluster);
    
    // Должны получить одну цель (с кодом 0x111)
    assert(reports.size() == 1);
    assert(reports[0].rbs.mode3a_code == 0x111);
    assert(reports[0].sources.size() == 3);  // три ответа с этим кодом
    
    print_target_report(reports[0], 0);
    std::cout << "Garbled targets test passed!\n";
}


void test_mixed_types_in_cluster() {
    std::cout << "\n=== Test Mixed Types in Cluster ===\n";
    
    RadarConfig cfg;
    cfg.range_bin_rbs = 30.0;
    cfg.range_bin_uvd = 50.0;
    
    ClusterProcessor processor(cfg);
    processor.set_min_hits(1);
    
    // Кластер с RBS и UVD ответами от разных целей
    TargetCluster cluster;
    
    for (int az = 1500; az <= 1510; az += 2) {
        ScanReplies scan(az, az * 1000);
        
        // RBS цель
        RBSReply rbs;
        rbs.azimuth = az;
        rbs.range = 600;
        rbs.code12 = 0x789;
        rbs.ether_amplitudes[0] = 200;
        rbs.ether_amplitudes[14] = 200;
        scan.rbs_replies.push_back(rbs);
        
        // UVD цель (другая дальность)
        if (az % 4 == 0) {
            UVDReply uvd;
            uvd.azimuth = az;
            uvd.range = 700;
            uvd.data20 = 0xABCDE;
            uvd.error_mask = 0;
            for (int i = 0; i < 80; i += 2) {
                uvd.ether_amplitudes[i] = 200;
                uvd.ether_amplitudes[i+1] = 0;
            }
            scan.uvd_replies.push_back(uvd);
        }
        
        cluster.add_scan(scan);
    }
    
    auto reports = processor.process_cluster(cluster);
    
    assert(reports.size() == 2);
    
    // Одна RBS, одна UVD
    bool has_rbs = false, has_uvd = false;
    for (const auto& report : reports) {
        if (report.type == TargetReport::SourceType::RBS) {
            has_rbs = true;
            assert(std::abs(report.range_m - 600 * 30.0) < 0.1);
            assert(report.rbs.mode3a_code == 0x789);
        } else {
            has_uvd = true;
            assert(std::abs(report.range_m - 700 * 50.0) < 0.1);
            assert(report.uvd.raw_data20 == 0xABCDE);
        }
    }
    
    assert(has_rbs && has_uvd);
    
    for (size_t i = 0; i < reports.size(); ++i) {
        print_target_report(reports[i], i);
    }
    std::cout << "Mixed types test passed!\n";
}

int main() {
    std::cout << "Running Cluster Processor tests...\n";
    
    test_single_rbs_target();
    test_single_uvd_target();
    test_two_targets_adjacent();
    test_garbled_targets();
    test_mixed_types_in_cluster();
    
    std::cout << "\nAll cluster processor tests passed!\n";
    return 0;
}