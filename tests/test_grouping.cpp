// file: tests/test_grouping.cpp
#include "radar/grouping.h"
#include "radar/simulator.h"
#include "radar/utils.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>

using namespace radar;

void print_cluster(const ReplyCluster& cluster, int index) {
    std::cout << "\nCluster " << index << ":\n";
    std::cout << "  Scan #: " << cluster.scan_number << "\n";
    std::cout << "  Azimuth range: " << cluster.start_azimuth << " - " << cluster.end_azimuth << "\n";
    std::cout << "  Center: az=" << cluster.avg_azimuth 
              << ", range=" << cluster.avg_range << "\n";
    std::cout << "  Span: az=" << cluster.azimuth_span 
              << " bins, range=" << cluster.range_span << " bins\n";
    std::cout << "  Replies: " << cluster.replies.size() << "\n";
    std::cout << "  Has overlap: " << cluster.has_overlap << "\n";
    
    for (size_t i = 0; i < cluster.replies.size(); ++i) {
        const auto& reply = cluster.replies[i];
        std::cout << "    " << i << ": ";
        if (reply.type == RawReply::Type::RBS) {
            std::cout << "RBS: az=" << reply.rbs.azimuth 
                      << ", range=" << reply.rbs.range 
                      << ", code=0x" << std::hex << reply.rbs.code12 << std::dec;
        } else {
            std::cout << "UVD: az=" << reply.uvd.azimuth 
                      << ", range=" << reply.uvd.range 
                      << ", data=0x" << std::hex << reply.uvd.data20 << std::dec;
        }
        std::cout << ", t=" << reply.timestamp_ms << "ms\n";
    }
    
    if (cluster.has_overlap) {
        std::cout << "  Overlapping pairs:\n";
        for (const auto& pair : cluster.overlapping_pairs) {
            std::cout << "    " << pair.first << " - " << pair.second << "\n";
        }
    }
}

// file: tests/test_grouping.cpp (фрагмент функции test_basic_grouping)
void test_basic_grouping() {
    std::cout << "\n=== Test Basic Grouping ===\n";
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;    // для перекрытия
    cfg.max_range_diff_for_overlap = 10;     // для перекрытия
    
    ScanPostProcessor processor(cfg);
    
    // Создаём несколько ответов с разными координатами
    RBSReply r1; r1.azimuth = 1000; r1.range = 500; r1.code12 = 0x123;
    RBSReply r2; r2.azimuth = 1001; r2.range = 505; r2.code12 = 0x456;  // близко к r1
    RBSReply r3; r3.azimuth = 2000; r3.range = 800; r3.code12 = 0x789;  // далеко (азимут +1000)
    RBSReply r4; r4.azimuth = 4095; r4.range = 300; r4.code12 = 0xABC;
    RBSReply r5; r5.azimuth = 0;    r5.range = 305; r5.code12 = 0xDEF;  // через 0 близко к r4
    
    uint32_t scan_num = 1;
    processor.add_reply(r1, scan_num, 1000);
    processor.add_reply(r2, scan_num, 1001);
    processor.add_reply(r3, scan_num, 2000);
    processor.add_reply(r4, scan_num, 3000);
    processor.add_reply(r5, scan_num, 3001);
    
    auto clusters = processor.finish_scan(scan_num);
    
    std::cout << "Number of clusters: " << clusters.size() << std::endl;  // Отладочный вывод
    
    // Должно быть 2 кластера: {r1,r2} и {r4,r5}
    // r3 должен быть отдельно, но из-за большого AZ_WINDOW может попасть в первый кластер
    assert(clusters.size() == 3);  // Ожидаем 3 кластера: {r1,r2}, {r3}, {r4,r5}
    
    // ... остальной код
}


void test_overlap_detection() {
    std::cout << "\n=== Test Overlap Detection ===\n";
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    ScanPostProcessor processor(cfg);
    
    // Создаём ответы с перекрытиями
    RBSReply r1; r1.azimuth = 1000; r1.range = 500;
    RBSReply r2; r2.azimuth = 1001; r2.range = 505;  // перекрывается с r1
    RBSReply r3; r3.azimuth = 1002; r3.range = 520;  // не перекрывается с r1/r2 (range далеко)
    
    uint32_t scan_num = 1;
    processor.add_reply(r1, scan_num, 1000);
    processor.add_reply(r2, scan_num, 1001);
    processor.add_reply(r3, scan_num, 1002);
    
    auto clusters = processor.finish_scan(scan_num);
    
    // Должен быть один кластер со всеми тремя (они близки по азимуту)
    assert(clusters.size() == 1);
    assert(clusters[0].replies.size() == 3);
    
    // Проверяем обнаружение перекрытий
    assert(clusters[0].has_overlap == true);
    assert(clusters[0].overlapping_pairs.size() == 1);  // только r1 и r2
    
    print_cluster(clusters[0], 0);
    std::cout << "Overlapping pairs: " << clusters[0].overlapping_pairs.size() << "\n";
    
    std::cout << "Overlap detection test passed!\n";
}

void test_separate_scans() {
    std::cout << "\n=== Test Separate Scans ===\n";
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    ScanPostProcessor processor(cfg);
    
    // Первый оборот
    RBSReply r1_scan1; r1_scan1.azimuth = 500; r1_scan1.range = 300; r1_scan1.code12 = 0x111;
    RBSReply r2_scan1; r2_scan1.azimuth = 501; r2_scan1.range = 305; r2_scan1.code12 = 0x222;
    
    processor.add_reply(r1_scan1, 1, 1000);
    processor.add_reply(r2_scan1, 1, 1001);
    
    auto clusters_scan1 = processor.finish_scan(1);
    assert(clusters_scan1.size() == 1);
    assert(clusters_scan1[0].replies.size() == 2);
    
    // Второй оборот
    RBSReply r1_scan2; r1_scan2.azimuth = 600; r1_scan2.range = 400; r1_scan2.code12 = 0x333;
    
    processor.add_reply(r1_scan2, 2, 2000);
    
    auto clusters_scan2 = processor.finish_scan(2);
    assert(clusters_scan2.size() == 1);
    assert(clusters_scan2[0].replies.size() == 1);
    assert(clusters_scan2[0].replies[0].rbs.code12 == 0x333);
    
    std::cout << "Separate scans test passed!\n";
}

void test_with_simulator() {
    std::cout << "\n=== Test With Simulator ===\n";
    
    // Настраиваем имитатор
    SimulatorConfig sim_cfg;
    sim_cfg.radar.range_bin_rbs = 30.0;
    sim_cfg.radar.max_azimuth_diff_for_overlap = 2;
    sim_cfg.radar.max_range_diff_for_overlap = 10;
    sim_cfg.rbs.snr_db = 30.0;
    
    ReplySimulator sim(sim_cfg);
    ScanPostProcessor processor(sim_cfg.radar);
    
    uint32_t scan_num = 42;
    uint32_t timestamp = 0;
    
    // Генерируем ответы в разных секторах
    std::vector<uint16_t> expected_codes;
    
    // Сектор 1: группа близких ответов
    for (int i = 0; i < 3; ++i) {
        uint16_t az = 1000 + i;
        uint16_t range = 500 + i * 2;
        uint16_t code = 0x100 + i;
        expected_codes.push_back(code);
        
        auto reply = sim.generate_rbs(az, range, code, false);
        processor.add_reply(reply, scan_num, timestamp);
        timestamp += 100;
    }
    
    // Сектор 2: одиночный ответ
    auto single = sim.generate_rbs(2000, 800, 0x200, true);
    processor.add_reply(single, scan_num, timestamp);
    expected_codes.push_back(0x200);
    timestamp += 100;
    
    // Сектор 3: группа с перекрытием
    auto o1 = sim.generate_rbs(3000, 1000, 0x301, false);
    auto o2 = sim.generate_rbs(3001, 1005, 0x302, false);
    processor.add_reply(o1, scan_num, timestamp);
    timestamp += 50;
    processor.add_reply(o2, scan_num, timestamp);
    expected_codes.push_back(0x301);
    expected_codes.push_back(0x302);
    
    auto clusters = processor.finish_scan(scan_num);
    
    std::cout << "Generated " << clusters.size() << " clusters from simulator data\n";
    
    // Должно быть 3 кластера
    assert(clusters.size() == 3);
    
    // Проверяем, что все коды присутствуют
    std::vector<uint16_t> found_codes;
    for (const auto& cluster : clusters) {
        static int cluster_idx = 0;
        print_cluster(cluster, cluster_idx++);
        for (const auto& reply : cluster.replies) {
            if (reply.type == RawReply::Type::RBS) {
                found_codes.push_back(reply.rbs.code12);
            }
        }
    }
    
    std::sort(expected_codes.begin(), expected_codes.end());
    std::sort(found_codes.begin(), found_codes.end());
    assert(expected_codes == found_codes);
    
    std::cout << "Simulator integration test passed!\n";
}

void test_timestamp_ordering() {
    std::cout << "\n=== Test Timestamp Ordering ===\n";
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 5;
    cfg.max_range_diff_for_overlap = 20;
    
    ScanPostProcessor processor(cfg);
    
    // Добавляем ответы в разном порядке по времени
    RBSReply r1; r1.azimuth = 1500; r1.range = 600; r1.code12 = 0x111;
    RBSReply r2; r2.azimuth = 1502; r2.range = 610; r2.code12 = 0x222;
    RBSReply r3; r3.azimuth = 1498; r3.range = 590; r3.code12 = 0x333;
    
    uint32_t scan_num = 1;
    processor.add_reply(r1, scan_num, 2000);  // среднее время
    processor.add_reply(r2, scan_num, 3000);  // позже всех
    processor.add_reply(r3, scan_num, 1000);  // раньше всех
    
    auto clusters = processor.finish_scan(scan_num);
    
    assert(clusters.size() == 1);
    assert(clusters[0].replies.size() == 3);
    
    // Проверяем, что ответы отсортированы по азимуту, а не по времени
    const auto& replies = clusters[0].replies;
    assert(replies[0].azimuth <= replies[1].azimuth);
    assert(replies[1].azimuth <= replies[2].azimuth);
    
    print_cluster(clusters[0], 0);
    
    std::cout << "Timestamp ordering test passed!\n";
}

void test_empty_scan() {
    std::cout << "\n=== Test Empty Scan ===\n";
    
    RadarConfig cfg;
    ScanPostProcessor processor(cfg);
    
    auto clusters = processor.finish_scan(1);
    assert(clusters.empty());
    
    clusters = processor.finish_scan(2);  // другой номер оборота
    assert(clusters.empty());
    
    std::cout << "Empty scan test passed!\n";
}

void test_reset() {
    std::cout << "\n=== Test Reset ===\n";
    
    RadarConfig cfg;
    ScanPostProcessor processor(cfg);
    
    RBSReply r1; r1.azimuth = 500; r1.range = 300; r1.code12 = 0x111;
    
    processor.add_reply(r1, 1, 1000);
    processor.reset();
    
    auto clusters = processor.finish_scan(1);
    assert(clusters.empty());  // после сброса данных нет
    
    std::cout << "Reset test passed!\n";
}

int main() {
    std::cout << "Running grouping tests...\n";
    
    test_basic_grouping();
    test_overlap_detection();
    test_separate_scans();
    test_with_simulator();
    test_timestamp_ordering();
    test_empty_scan();
    test_reset();
    
    std::cout << "\nAll grouping tests passed!\n";
    return 0;
}