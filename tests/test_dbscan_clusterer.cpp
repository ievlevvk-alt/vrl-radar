// tests/test_dbscan_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/config.h"
#include <vector>
#include <memory>
#include <iostream>

using namespace vrl::radar;

class DBSCANClustererTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(1000);
        ClusterPool::instance().clear();
        
        config.range_bin_rbs = 30.0;
        config.range_bin_uvd = 60.0;
        config.beamwidth_deg = 5.0;
        config.min_amplitude = 10;
        
        clusterer = std::make_unique<DBSCANClusterer>(config, 3, 1.2);
        clusterer->set_debug(true);
        
        std::cout << "\n=== DBSCANClustererTest::SetUp ===" << std::endl;
        std::cout << "max_azimuth_gap = " << clusterer->get_max_azimuth_gap() << std::endl;
        std::cout << "max_range_gap = " << clusterer->get_max_range_gap() << std::endl;
        std::cout << "====================================\n" << std::endl;
    }
    
    void TearDown() override {
        ClusterPool::instance().clear();
        clusterer.reset();
    }
    
    size_t count_active_clusters() const {
        return ClusterPool::instance().count_active_clusters();
    }
    
    size_t count_closed_clusters() const {
        return ClusterPool::instance().count_closed_clusters();
    }
    
    size_t count_total_clusters() const {
        return ClusterPool::instance().size();
    }
    
    std::vector<const Cluster*> get_closed_clusters() const {
        std::vector<const Cluster*> result;
        auto clusters = ClusterPool::instance().get_all_clusters();
        for (Cluster* cluster : clusters) {
            if (cluster && cluster->is_closed()) {
                result.push_back(cluster);
            }
        }
        return result;
    }
    
    void print_cluster_state(const std::string& label) {
        std::cout << "\n=== " << label << " ===" << std::endl;
        auto clusters = ClusterPool::instance().get_all_clusters();
        std::cout << "Total clusters: " << clusters.size() << std::endl;
        for (size_t i = 0; i < clusters.size(); ++i) {
            Cluster* cluster = clusters[i];
            if (cluster) {
                std::cout << "  Cluster[" << i << "]: size=" << cluster->size()
                          << ", closed=" << cluster->is_closed()
                          << ", min_range=" << cluster->get_min_range()
                          << ", max_range=" << cluster->get_max_range()
                          << ", last_az=" << cluster->get_last_azimuth()
                          << ", span=" << cluster->get_azimuth_span()
                          << std::endl;
            } else {
                std::cout << "  Cluster[" << i << "]: nullptr" << std::endl;
            }
        }
        std::cout << "==================\n" << std::endl;
    }
    
    RadarConfig config;
    std::unique_ptr<DBSCANClusterer> clusterer;
    
    static constexpr int AZIMUTH_BINS = 4096;
};

// ============================================================================
// ТЕСТЫ
// ============================================================================

TEST_F(DBSCANClustererTest, TwoPointsFormCluster) {
    std::cout << "\n=== TEST: TwoPointsFormCluster ===" << std::endl;
    
    ScanReplies scan1(100, 0);
    RBSReply reply1;
    reply1.azimuth = 100;
    reply1.range = 100;
    reply1.code12 = 0x1234;
    reply1.spi = false;
    reply1.ether_amplitudes[0] = 100;
    reply1.ether_amplitudes[14] = 100;
    scan1.rbs_replies.push_back(reply1);
    
    ScanReplies scan2(105, 0);
    RBSReply reply2;
    reply2.azimuth = 105;
    reply2.range = 102;
    reply2.code12 = 0x1234;
    reply2.spi = false;
    reply2.ether_amplitudes[0] = 100;
    reply2.ether_amplitudes[14] = 100;
    scan2.rbs_replies.push_back(reply2);
    
    clusterer->process_scan(scan1);
    print_cluster_state("After scan1");
    
    clusterer->process_scan(scan2);
    print_cluster_state("After scan2");
    
    EXPECT_EQ(count_active_clusters(), 1);
    
    clusterer->close_expired_clusters(200);
    print_cluster_state("After close");
    
    auto closed = get_closed_clusters();
    EXPECT_EQ(closed.size(), 1);
    if (!closed.empty()) {
        EXPECT_EQ(closed[0]->size(), 2);
    }
}

TEST_F(DBSCANClustererTest, ClusterMergeScenario) {
    std::cout << "\n=== TEST: ClusterMergeScenario ===" << std::endl;
    
    std::vector<std::pair<uint16_t, uint16_t>> points = {
        {100, 100},
        {105, 102},
        {110, 104}
    };
    
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& [az, range] = points[i];
        ScanReplies scan(az, 0);
        RBSReply reply;
        reply.azimuth = az;
        reply.range = range;
        reply.code12 = 0x1234;
        reply.spi = false;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        scan.rbs_replies.push_back(reply);
        clusterer->process_scan(scan);
        print_cluster_state("After point " + std::to_string(i+1));
    }
    
    EXPECT_EQ(count_active_clusters(), 1);
    
    clusterer->close_expired_clusters(200);
    print_cluster_state("After close");
    
    auto closed = get_closed_clusters();
    EXPECT_EQ(closed.size(), 1);
    if (!closed.empty()) {
        EXPECT_EQ(closed[0]->size(), 3);
    }
}

TEST_F(DBSCANClustererTest, DifferentCodesSameCluster) {
    std::cout << "\n=== TEST: DifferentCodesSameCluster ===" << std::endl;
    
    ScanReplies scan1(100, 0);
    RBSReply reply1;
    reply1.azimuth = 100;
    reply1.range = 100;
    reply1.code12 = 0x1234;
    reply1.spi = false;
    reply1.ether_amplitudes[0] = 100;
    reply1.ether_amplitudes[14] = 100;
    scan1.rbs_replies.push_back(reply1);
    
    ScanReplies scan2(105, 0);
    RBSReply reply2;
    reply2.azimuth = 105;
    reply2.range = 102;
    reply2.code12 = 0x5678;
    reply2.spi = false;
    reply2.ether_amplitudes[0] = 100;
    reply2.ether_amplitudes[14] = 100;
    scan2.rbs_replies.push_back(reply2);
    
    clusterer->process_scan(scan1);
    clusterer->process_scan(scan2);
    print_cluster_state("After both scans");
    
    EXPECT_EQ(count_active_clusters(), 1);
}

TEST_F(DBSCANClustererTest, AzimuthDistance) {
    std::cout << "\n=== TEST: AzimuthDistance ===" << std::endl;
    
    int az_gap = static_cast<int>(config.beamwidth_deg * 4096 / 360.0 * 1.2);
    std::cout << "az_gap (calculated) = " << az_gap << std::endl;
    std::cout << "max_azimuth_gap = " << clusterer->get_max_azimuth_gap() << std::endl;
    
    ScanReplies scan1(100, 0);
    RBSReply reply1;
    reply1.azimuth = 100;
    reply1.range = 100;
    reply1.code12 = 0x1234;
    reply1.spi = false;
    reply1.ether_amplitudes[0] = 100;
    reply1.ether_amplitudes[14] = 100;
    scan1.rbs_replies.push_back(reply1);
    
    std::cout << "Processing scan1 at az=100" << std::endl;
    clusterer->process_scan(scan1);
    print_cluster_state("After scan1");
    
    std::cout << "Closing clusters at az=200" << std::endl;
    clusterer->close_expired_clusters(200);
    print_cluster_state("After close_expired_clusters(200)");
    
    int gap = az_gap * 3;
    std::cout << "Second point gap = " << gap << " (azimuth = " << 100 + gap << ")" << std::endl;
    
    ScanReplies scan2(100 + gap, 0);
    RBSReply reply2;
    reply2.azimuth = 100 + gap;
    reply2.range = 102;
    reply2.code12 = 0x1234;
    reply2.spi = false;
    reply2.ether_amplitudes[0] = 100;
    reply2.ether_amplitudes[14] = 100;
    scan2.rbs_replies.push_back(reply2);
    
    std::cout << "Processing scan2 at az=" << 100 + gap << std::endl;
    clusterer->process_scan(scan2);
    print_cluster_state("After scan2");
    
    EXPECT_EQ(count_total_clusters(), 2);
    EXPECT_EQ(count_active_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
}

TEST_F(DBSCANClustererTest, SPIFlagPreserved) {
    std::cout << "\n=== TEST: SPIFlagPreserved ===" << std::endl;
    
    ScanReplies scan1(100, 0);
    RBSReply reply1;
    reply1.azimuth = 100;
    reply1.range = 100;
    reply1.code12 = 0x1234;
    reply1.spi = true;
    reply1.ether_amplitudes[0] = 100;
    reply1.ether_amplitudes[14] = 100;
    scan1.rbs_replies.push_back(reply1);
    
    ScanReplies scan2(105, 0);
    RBSReply reply2;
    reply2.azimuth = 105;
    reply2.range = 102;
    reply2.code12 = 0x1234;
    reply2.spi = false;
    reply2.ether_amplitudes[0] = 100;
    reply2.ether_amplitudes[14] = 100;
    scan2.rbs_replies.push_back(reply2);
    
    clusterer->process_scan(scan1);
    clusterer->process_scan(scan2);
    print_cluster_state("After both scans");
    
    clusterer->close_expired_clusters(200);
    print_cluster_state("After close");
    
    auto closed = get_closed_clusters();
    EXPECT_EQ(closed.size(), 1);
    if (!closed.empty()) {
        const auto& indices = closed[0]->get_indices();
        auto& buffer = PointBuffer::instance();
        bool has_spi = false;
        for (size_t idx : indices) {
            const auto& point = buffer.get_point(idx);
            if (point.spi) {
                has_spi = true;
                break;
            }
        }
        EXPECT_TRUE(has_spi);
    }
}

TEST_F(DBSCANClustererTest, AzimuthSpanNorth) {
    std::cout << "\n=== TEST: AzimuthSpanNorth ===" << std::endl;
    
    ScanReplies scan1(4095, 0);
    RBSReply reply1;
    reply1.azimuth = 4095;
    reply1.range = 100;
    reply1.code12 = 0x1234;
    reply1.spi = false;
    reply1.ether_amplitudes[0] = 100;
    reply1.ether_amplitudes[14] = 100;
    scan1.rbs_replies.push_back(reply1);
    
    ScanReplies scan2(10, 0);
    RBSReply reply2;
    reply2.azimuth = 10;
    reply2.range = 102;
    reply2.code12 = 0x1234;
    reply2.spi = false;
    reply2.ether_amplitudes[0] = 100;
    reply2.ether_amplitudes[14] = 100;
    scan2.rbs_replies.push_back(reply2);
    
    clusterer->process_scan(scan1);
    print_cluster_state("After scan1 (near north)");
    
    clusterer->process_scan(scan2);
    print_cluster_state("After scan2 (after north)");
    
    EXPECT_EQ(count_active_clusters(), 1);
    
    auto clusters = ClusterPool::instance().get_all_clusters();
    for (Cluster* cluster : clusters) {
        if (cluster && !cluster->is_closed()) {
            int span = cluster->get_azimuth_span();
            std::cout << "Cluster azimuth span: " << span << std::endl;
            EXPECT_LT(span, 100);
        }
    }
}

TEST_F(DBSCANClustererTest, NorthTransition) {
    std::cout << "\n=== TEST: NorthTransition ===" << std::endl;
    
    std::vector<std::pair<uint16_t, uint16_t>> points = {
        {4090, 100},
        {4095, 102},
        {5, 104},
        {10, 106}
    };
    
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& [az, range] = points[i];
        ScanReplies scan(az, 0);
        RBSReply reply;
        reply.azimuth = az;
        reply.range = range;
        reply.code12 = 0x1234;
        reply.spi = false;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        scan.rbs_replies.push_back(reply);
        clusterer->process_scan(scan);
        print_cluster_state("After point " + std::to_string(i+1));
    }
    
    EXPECT_EQ(count_active_clusters(), 1);
    
    clusterer->close_expired_clusters(200);
    print_cluster_state("After close");
    
    auto closed = get_closed_clusters();
    EXPECT_EQ(closed.size(), 1);
    if (!closed.empty()) {
        EXPECT_EQ(closed[0]->size(), 4);
    }
}

TEST_F(DBSCANClustererTest, Reset) {
    std::cout << "\n=== TEST: Reset ===" << std::endl;
    
    for (int i = 0; i < 5; ++i) {
        ScanReplies scan(100 + i * 10, 0);
        RBSReply reply;
        reply.azimuth = 100 + i * 10;
        reply.range = 100 + i * 5;
        reply.code12 = 0x1234;
        reply.spi = false;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        scan.rbs_replies.push_back(reply);
        clusterer->process_scan(scan);
    }
    print_cluster_state("Before reset");
    
    EXPECT_GT(count_active_clusters(), 0);
    
    clusterer->reset();
    print_cluster_state("After reset");
    
    EXPECT_EQ(count_active_clusters(), 0);
    EXPECT_EQ(count_closed_clusters(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
