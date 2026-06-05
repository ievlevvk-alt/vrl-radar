// file: tests/test_cluster_tracker.cpp
#include "radar/cluster_tracker.h"
#include "radar/simulator.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <map>
#include <set>  // Добавлено

using namespace radar;

void print_cluster(const TargetCluster& cluster, int index) {
    std::cout << "\nCluster " << index << ":\n";
    std::cout << "  Azimuth range: " << cluster.start_azimuth << " - " << cluster.last_reply_azimuth 
              << " (span=" << cluster.azimuth_span() << ")\n";
    std::cout << "  Total azimuth span (with gaps): " << cluster.total_azimuth_span() << "\n";
    std::cout << "  Range range: " << cluster.min_range << " - " << cluster.max_range 
              << " (span=" << cluster.range_span() << ")\n";
    std::cout << "  Scans: " << cluster.scans.size() << " total, "
              << cluster.reply_scans_count() << " with replies\n";
    std::cout << "  Total RBS replies: " << cluster.get_all_rbs().size() << "\n";
    std::cout << "  Total UVD replies: " << cluster.get_all_uvd().size() << "\n";
    
    // Детальная информация по азимутам
    if (!cluster.rbs_by_azimuth.empty() || !cluster.uvd_by_azimuth.empty()) {
        std::cout << "  Replies by azimuth:\n";
        
        // Собираем все уникальные азимуты
        std::set<uint16_t> all_azimuths;
        for (const auto& [az, _] : cluster.rbs_by_azimuth) all_azimuths.insert(az);
        for (const auto& [az, _] : cluster.uvd_by_azimuth) all_azimuths.insert(az);
        
        for (uint16_t az : all_azimuths) {
            std::cout << "    az=" << az << ": ";
            if (cluster.rbs_by_azimuth.count(az)) {
                std::cout << cluster.rbs_by_azimuth.at(az).size() << " RBS ";
            }
            if (cluster.uvd_by_azimuth.count(az)) {
                std::cout << cluster.uvd_by_azimuth.at(az).size() << " UVD";
            }
            std::cout << "\n";
        }
    }
}

void test_single_target() {
    std::cout << "\n=== Test Single Target ===\n";
    
    ClusterTracker tracker(4, 30);  // max_gap=4, range_window=30
    
    // Симулируем пролёт одной цели через луч
    std::vector<int> azimuths = {1000, 1002, 1004, 1006, 1008, 1010, 1012};
    
    for (size_t i = 0; i < azimuths.size(); ++i) {
        ScanReplies scan(azimuths[i], i * 1000);
        
        // Добавляем ответ от цели на каждом зондировании
        RBSReply reply;
        reply.azimuth = azimuths[i];
        reply.range = 500;
        reply.code12 = 0x123;
        
        scan.rbs_replies.push_back(reply);
        tracker.process_scan(scan);
        
        std::cout << "  Processed azimuth " << azimuths[i] << " with reply\n";
    }
    
    std::cout << "  After replies, active clusters: " << tracker.get_active_clusters().size() << "\n";
    
    // Добавляем пустые зондирования для закрытия кластера
    for (int az = 1020; az < 1040; az += 2) {
        ScanReplies scan(az, az * 10);
        tracker.process_scan(scan);
        std::cout << "  Processed empty azimuth " << az << "\n";
    }
    
    auto clusters = tracker.get_completed_clusters();
    std::cout << "Completed clusters: " << clusters.size() << "\n";
    
    assert(clusters.size() == 1);
    assert(clusters[0].azimuth_span() >= 12);  // от 1000 до 1012
    assert(clusters[0].get_all_rbs().size() == 7);
    assert(clusters[0].min_range == 500);
    assert(clusters[0].max_range == 500);
    assert(clusters[0].reply_scans_count() == 7);
    
    print_cluster(clusters[0], 0);
    std::cout << "Single target test passed!\n";
}

void test_two_targets_adjacent() {
    std::cout << "\n=== Test Two Targets Adjacent ===\n";
    
    ClusterTracker tracker(4, 30);
    
    // Две цели, идущие рядом
    // Цель A: азимуты 1000-1010, дальность 500
    // Цель B: азимуты 1006-1016, дальность 505
    
    std::vector<int> azimuths = {1000, 1002, 1004, 1006, 1008, 1010, 1012, 1014, 1016};
    
    for (size_t i = 0; i < azimuths.size(); ++i) {
        ScanReplies scan(azimuths[i], i * 1000);
        
        // Цель A (первые 6 зондирований)
        if (i < 6) {
            RBSReply reply_a;
            reply_a.azimuth = azimuths[i];
            reply_a.range = 500;
            reply_a.code12 = 0xAAA;
            scan.rbs_replies.push_back(reply_a);
        }
        
        // Цель B (последние 6 зондирований)
        if (i >= 3 && i < 9) {
            RBSReply reply_b;
            reply_b.azimuth = azimuths[i];
            reply_b.range = 505;
            reply_b.code12 = 0xBBB;
            scan.rbs_replies.push_back(reply_b);
        }
        
        tracker.process_scan(scan);
    }
    
    // Добавляем пустые зондирования для закрытия кластера
    for (int az = 1020; az < 1040; az += 2) {
        ScanReplies scan(az, az * 10);
        tracker.process_scan(scan);
    }
    
    auto clusters = tracker.get_completed_clusters();
    
    // Должен быть ОДИН кластер, так как цели перекрываются по азимуту
    assert(clusters.size() == 1);
    assert(clusters[0].azimuth_span() >= 16);  // от 1000 до 1016
    assert(clusters[0].get_all_rbs().size() == 12);  // 6 + 6 = 12
    assert(clusters[0].min_range == 500);
    assert(clusters[0].max_range == 505);
    assert(clusters[0].reply_scans_count() == 9);  // все 9 сканов с ответами
    
    print_cluster(clusters[0], 0);
    std::cout << "Two targets adjacent test passed!\n";
}

void test_two_targets_separate() {
    std::cout << "\n=== Test Two Targets Separate ===\n";
    
    ClusterTracker tracker(4, 30);
    
    // Две цели далеко друг от друга по азимуту
    // Цель A: азимуты 1000-1010, дальность 500
    // Цель B: азимуты 2000-2010, дальность 800
    
    // Цель A
    for (int az = 1000; az <= 1010; az += 2) {
        ScanReplies scan(az, az * 10);
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 500;
        reply.code12 = 0xAAA;
        scan.rbs_replies.push_back(reply);
        tracker.process_scan(scan);
    }
    
    // Пустые зондирования между целями
    for (int az = 1012; az < 2000; az += 10) {
        ScanReplies scan(az, az * 10);
        tracker.process_scan(scan);
    }
    
    // Цель B
    for (int az = 2000; az <= 2010; az += 2) {
        ScanReplies scan(az, az * 10);
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 800;
        reply.code12 = 0xBBB;
        scan.rbs_replies.push_back(reply);
        tracker.process_scan(scan);
    }
    
    // Пустые зондирования в конце
    for (int az = 2020; az < 2040; az += 10) {
        ScanReplies scan(az, az * 10);
        tracker.process_scan(scan);
    }
    
    auto clusters = tracker.get_completed_clusters();
    
    // Должно быть ДВА кластера
    assert(clusters.size() == 2);
    
    print_cluster(clusters[0], 0);
    print_cluster(clusters[1], 1);
    std::cout << "Two targets separate test passed!\n";
}

void test_garbling_detection() {
    std::cout << "\n=== Test Garbling Detection ===\n";
    
    ClusterTracker tracker(4, 30);
    
    // Одно зондирование с двумя перекрывающимися ответами
    ScanReplies scan(1000, 1000);
    
    RBSReply r1;
    r1.azimuth = 1000;
    r1.range = 500;
    r1.code12 = 0x123;
    
    RBSReply r2;
    r2.azimuth = 1000;  // тот же азимут
    r2.range = 505;      // близкая дальность
    r2.code12 = 0x456;
    
    scan.rbs_replies.push_back(r1);
    scan.rbs_replies.push_back(r2);
    
    tracker.process_scan(scan);
    
    // Пустые зондирования для закрытия кластера
    for (int az = 1010; az < 1040; az += 10) {
        ScanReplies empty_scan(az, az * 10);
        tracker.process_scan(empty_scan);
    }
    
    auto clusters = tracker.get_completed_clusters();
    assert(clusters.size() == 1);
    
    // Проверяем перекрытие в первом (и единственном) скане
    auto garbled = clusters[0].find_garbled_pairs_in_scan(0);
    assert(!garbled.empty());  // должно быть обнаружено перекрытие
    
    print_cluster(clusters[0], 0);
    std::cout << "  Garbled pairs: " << garbled.size() << "\n";
    std::cout << "Garbling detection test passed!\n";
}

void test_mixed_types() {
    std::cout << "\n=== Test Mixed Types ===\n";
    
    ClusterTracker tracker(4, 30);
    
    // Зондирование с RBS и UVD ответами
    ScanReplies scan(1500, 1500);
    
    RBSReply rbs;
    rbs.azimuth = 1500;
    rbs.range = 600;
    rbs.code12 = 0x789;
    
    UVDReply uvd;
    uvd.azimuth = 1500;
    uvd.range = 605;
    uvd.data20 = 0xABCDE;
    
    scan.rbs_replies.push_back(rbs);
    scan.uvd_replies.push_back(uvd);
    
    tracker.process_scan(scan);
    
    // Ещё несколько зондирований с теми же ответами
    for (int az = 1502; az <= 1510; az += 2) {
        ScanReplies next_scan(az, az * 10);
        
        RBSReply rbs2;
        rbs2.azimuth = az;
        rbs2.range = 600;
        rbs2.code12 = 0x789;
        
        UVDReply uvd2;
        uvd2.azimuth = az;
        uvd2.range = 605;
        uvd2.data20 = 0xABCDE;
        
        next_scan.rbs_replies.push_back(rbs2);
        next_scan.uvd_replies.push_back(uvd2);
        
        tracker.process_scan(next_scan);
    }
    
    // Закрываем кластер
    for (int az = 1520; az < 1550; az += 10) {
        ScanReplies empty_scan(az, az * 10);
        tracker.process_scan(empty_scan);
    }
    
    auto clusters = tracker.get_completed_clusters();
    assert(clusters.size() == 1);
    assert(clusters[0].get_all_rbs().size() == 6);  // 1 + 5 = 6
    assert(clusters[0].get_all_uvd().size() == 6);
    
    print_cluster(clusters[0], 0);
    std::cout << "Mixed types test passed!\n";
}

int main() {
    std::cout << "Running Cluster Tracker tests...\n";
    
    test_single_target();
    test_two_targets_adjacent();
    test_two_targets_separate();
    test_garbling_detection();
    test_mixed_types();
    
    std::cout << "\nAll cluster tracker tests passed!\n";
    return 0;
}