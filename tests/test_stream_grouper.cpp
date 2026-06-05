// file: tests/test_stream_grouper.cpp
#include "radar/stream_grouper.h"
#include "radar/simulator.h"
#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace radar;

// Глобальные счётчики для простых тестов
int rbs_cluster_count = 0;
int uvd_cluster_count = 0;
std::vector<size_t> cluster_sizes;
std::vector<double> cluster_azimuths;

void test_rbs_handler(const ProcessedCluster<RBSReply>& cluster) {
    if (cluster.replies.empty()) return;
    
    rbs_cluster_count++;
    cluster_sizes.push_back(cluster.replies.size());
    cluster_azimuths.push_back(cluster.avg_azimuth);
    
    std::cout << "RBS Cluster #" << rbs_cluster_count << ":\n";
    std::cout << "  replies=" << cluster.replies.size()
              << " avg_az=" << cluster.avg_azimuth
              << " avg_range=" << cluster.avg_range
              << " overlaps=" << cluster.has_overlaps
              << " span_az=" << cluster.azimuth_span
              << " span_range=" << cluster.range_span << "\n";
    
    for (const auto& reply : cluster.replies) {
        std::cout << "    reply: az=" << reply.azimuth 
                  << " range=" << reply.range 
                  << " code=0x" << std::hex << reply.code12 << std::dec << "\n";
    }
}

void test_uvd_handler(const ProcessedCluster<UVDReply>& cluster) {
    if (cluster.replies.empty()) return;
    
    uvd_cluster_count++;
    
    std::cout << "UVD Cluster #" << uvd_cluster_count << ":\n";
    std::cout << "  replies=" << cluster.replies.size()
              << " avg_az=" << cluster.avg_azimuth
              << " avg_range=" << cluster.avg_range << "\n";
}

void reset_counters() {
    rbs_cluster_count = 0;
    uvd_cluster_count = 0;
    cluster_sizes.clear();
    cluster_azimuths.clear();
}

void test_basic_stream_grouping() {
    std::cout << "\n=== Test Basic Stream Grouping ===\n";
    reset_counters();
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    double beamwidth = 40.0;
    
    RBSStreamGrouper grouper(cfg, beamwidth, test_rbs_handler);
    
    ScanInfo scan;
    scan.scan_number = 1;
    
    // Первая группа ответов
    std::cout << "Sending first group of replies (az 100-128)...\n";
    for (int az = 100; az < 130; az += 2) {
        scan.azimuth = az;
        scan.scan_start = (az == 100);
        scan.scan_end = false;
        
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 500 + (az % 50);
        reply.code12 = 0x100 + (az % 16);
        
        grouper.process_reply(reply, scan);
    }
    
    // Вторая группа ответов
    std::cout << "Sending second group of replies (az 200-218)...\n";
    for (int az = 200; az < 220; az += 2) {
        scan.azimuth = az;
        
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 800;
        reply.code12 = 0x200;
        
        grouper.process_reply(reply, scan);
    }
    
    // Завершаем оборот
    scan.azimuth = 4095;
    scan.scan_end = true;
    grouper.flush();
    
    std::cout << "RBS clusters created: " << rbs_cluster_count << "\n";
    std::cout << "Cluster sizes: ";
    for (size_t s : cluster_sizes) std::cout << s << " ";
    std::cout << "\n";
    
    assert(rbs_cluster_count == 2);
    assert(cluster_sizes.size() == 2);
    assert(cluster_sizes[0] == 15);
    assert(cluster_sizes[1] == 10);
    
    std::cout << "Basic stream grouping test passed! Clusters: " << rbs_cluster_count << "\n";
}

void test_overlap_in_stream() {
    std::cout << "\n=== Test Overlap Detection in Stream ===\n";
    reset_counters();
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    double beamwidth = 40.0;
    bool overlap_detected = false;
    
    auto handler = [&overlap_detected](const ProcessedCluster<RBSReply>& cluster) {
        if (cluster.replies.empty()) return;
        rbs_cluster_count++;
        if (cluster.has_overlaps) {
            overlap_detected = true;
            std::cout << "  Overlap detected in cluster with " << cluster.replies.size() << " replies\n";
        }
    };
    
    RBSStreamGrouper grouper(cfg, beamwidth, handler);
    
    ScanInfo scan;
    scan.scan_number = 1;
    
    // Генерируем перекрывающиеся ответы
    std::cout << "Sending overlapping replies...\n";
    for (int az = 500; az < 510; az += 1) {
        scan.azimuth = az;
        
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 600 + (az % 5) * 2;
        reply.code12 = 0x300;
        
        grouper.process_reply(reply, scan);
    }
    
    // Завершаем оборот
    scan.azimuth = 4095;
    scan.scan_end = true;
    grouper.flush();
    
    std::cout << "RBS clusters: " << rbs_cluster_count << "\n";
    assert(rbs_cluster_count > 0);
    assert(overlap_detected);
    
    std::cout << "Overlap detection test passed! Overlap found: " << overlap_detected << "\n";
}

void test_azimuth_wraparound() {
    std::cout << "\n=== Test Azimuth Wraparound ===\n";
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 5;  // Умеренные параметры
    cfg.max_range_diff_for_overlap = 20;
    
    double beamwidth = 50.0;  // Умеренная ширина ДН
    
    // Счётчики для каждого оборота
    int first_scan_clusters = 0;
    int second_scan_clusters = 0;
    size_t first_scan_replies = 0;
    size_t second_scan_replies = 0;
    
    // Вектор для сбора всех ответов второго оборота
    std::vector<RBSReply> second_scan_all_replies;
    
    // Обработчик для первого оборота
    auto handler_first = [&first_scan_clusters, &first_scan_replies](const ProcessedCluster<RBSReply>& cluster) {
        if (cluster.replies.empty()) return;
        first_scan_clusters++;
        first_scan_replies = cluster.replies.size();
        std::cout << "First scan cluster #" << first_scan_clusters 
                  << " with " << cluster.replies.size() << " replies\n";
    };
    
    // Обработчик для второго оборота - собираем все ответы в один кластер
    auto handler_second = [&second_scan_clusters, &second_scan_replies, &second_scan_all_replies]
                         (const ProcessedCluster<RBSReply>& cluster) {
        if (cluster.replies.empty()) return;
        second_scan_clusters++;
        second_scan_replies += cluster.replies.size();
        
        // Сохраняем все ответы
        for (const auto& reply : cluster.replies) {
            second_scan_all_replies.push_back(reply);
        }
        
        std::cout << "Second scan cluster #" << second_scan_clusters 
                  << " with " << cluster.replies.size() << " replies\n";
        std::cout << "    cluster azimuth range: " << cluster.azimuth_span << "\n";
    };
    
    // Первый оборот
    RBSStreamGrouper grouper_first(cfg, beamwidth, handler_first);
    
    ScanInfo scan;
    scan.scan_number = 1;
    
    // Ответы в конце первого оборота
    std::cout << "Sending replies at end of scan (az 4050-4080 with step 2)...\n";
    for (int az = 4050; az < 4080; az += 2) {
        scan.azimuth = az;
        scan.scan_start = (az == 4050);
        scan.scan_end = false;
        
        RBSReply reply;
        reply.azimuth = az;
        reply.range = 400;
        reply.code12 = 0x400;
        
        grouper_first.process_reply(reply, scan);
    }
    
    // Завершаем первый оборот
    scan.azimuth = 4095;
    scan.scan_end = true;
    grouper_first.flush();
    
    std::cout << "Clusters in first scan: " << first_scan_clusters << "\n";
    assert(first_scan_clusters == 1);
    assert(first_scan_replies == 15);
    
    // Второй оборот
    RBSStreamGrouper grouper_second(cfg, beamwidth, handler_second);
    
    scan.scan_number = 2;
    scan.scan_start = true;
    scan.scan_end = false;
    
    // Отправляем все ответы быстро, пока луч не ушёл
    std::cout << "Sending all replies rapidly at same azimuth...\n";
    
    // Сначала отправляем все ответы при одном азимуте
    int start_az = 25;
    scan.azimuth = start_az;
    
    for (int i = 0; i < 15; i++) {
        RBSReply reply;
        reply.azimuth = start_az;
        reply.range = 450 + i * 2;
        reply.code12 = 0x500 + i;
        
        grouper_second.process_reply(reply, scan);
    }
    
    // Затем быстро продвигаем азимут, чтобы закрыть кластер
    // Но делаем это с маленьким шагом, чтобы не создать новые кластеры
    for (int az = start_az + 1; az < start_az + beamwidth; az += 5) {
        scan.azimuth = az;
        // Не отправляем ответы, просто двигаем азимут
    }
    
    // Завершаем оборот
    scan.azimuth = start_az + static_cast<int>(beamwidth) + 10;
    scan.scan_end = true;
    grouper_second.flush();
    
    std::cout << "Clusters in second scan: " << second_scan_clusters << "\n";
    std::cout << "Total replies in second scan: " << second_scan_replies << "\n";
    
    // Проверяем, что все ответы собраны
    assert(second_scan_replies == 15);
    assert(second_scan_all_replies.size() == 15);
    
    // Проверяем, что все ответы имеют правильные коды
    for (int i = 0; i < 15; i++) {
        assert(second_scan_all_replies[i].code12 == 0x500 + i);
    }
    
    std::cout << "Wraparound test passed! First scan: " << first_scan_clusters 
              << " cluster (" << first_scan_replies << " replies), "
              << "Second scan: " << second_scan_clusters 
              << " clusters (" << second_scan_replies << " total replies)\n";
}



void test_mixed_types_separate() {
    std::cout << "\n=== Test Mixed Types Are Separate ===\n";
    reset_counters();
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    double beamwidth = 40.0;
    
    RBSStreamGrouper rbs_grouper(cfg, beamwidth, test_rbs_handler);
    UVDStreamGrouper uvd_grouper(cfg, beamwidth, test_uvd_handler);
    
    ScanInfo scan;
    scan.scan_number = 1;
    
    // Добавляем RBS и UVD ответы в одном секторе
    std::cout << "Sending mixed RBS and UVD replies...\n";
    for (int az = 1000; az < 1020; az += 2) {
        scan.azimuth = az;
        
        RBSReply rbs;
        rbs.azimuth = az;
        rbs.range = 700;
        rbs.code12 = 0x600;
        
        UVDReply uvd;
        uvd.azimuth = az;
        uvd.range = 705;
        uvd.data20 = 0x12345;
        
        rbs_grouper.process_reply(rbs, scan);
        uvd_grouper.process_reply(uvd, scan);
    }
    
    scan.azimuth = 4095;
    scan.scan_end = true;
    rbs_grouper.flush();
    uvd_grouper.flush();
    
    std::cout << "RBS clusters: " << rbs_cluster_count << ", UVD clusters: " << uvd_cluster_count << "\n";
    assert(rbs_cluster_count == 1);
    assert(uvd_cluster_count == 1);
    
    std::cout << "Mixed types separate test passed!\n";
}

int main() {
    std::cout << "Running Stream Grouper tests...\n";
    
    test_basic_stream_grouping();
    test_overlap_in_stream();
    test_azimuth_wraparound();
    test_mixed_types_separate();
    
    std::cout << "\nAll stream grouper tests passed!\n";
    return 0;
}