// tests/test_integration.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/processing/dbscan_clusterer.h"
#include <vector>
#include <memory>
#include <iostream>

using namespace vrl::radar;

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(1000);
        ClusterPool::instance().clear();
        
        config.range_bin_rbs = 30.0;
        config.range_bin_uvd = 60.0;
        config.beamwidth_deg = 5.0;
        config.min_amplitude = 10;
        
        clusterer = std::make_unique<DBSCANClusterer>(config, 3, 1.2);
        clusterer->set_debug(false);
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
    
    RadarConfig config;
    std::unique_ptr<DBSCANClusterer> clusterer;
    
    static constexpr int AZIMUTH_BINS = 4096;
};

TEST_F(IntegrationTest, FullClusterLifecycle) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    std::vector<StoredPoint> points;
    for (int i = 0; i < 5; ++i) {
        StoredPoint point;
        point.azimuth = 100 + i * 5;
        point.range = 50 + i * 2;
        point.is_rbs = true;
        point.amplitude = 100;
        point.code12 = 0x1234;
        point.spi = (i == 2);
        points.push_back(point);
    }
    
    for (const auto& point : points) {
        ScanReplies scan(point.azimuth, 0);
        RBSReply reply;
        reply.azimuth = point.azimuth;
        reply.range = point.range;
        reply.code12 = point.code12;
        reply.spi = point.spi;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        scan.rbs_replies.push_back(reply);
        clusterer->process_scan(scan);
    }
    
    EXPECT_EQ(count_active_clusters(), 1);
    
    clusterer->close_expired_clusters(200);
    
    EXPECT_EQ(count_active_clusters(), 0);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    
    const Cluster* cluster = closed[0];
    EXPECT_EQ(cluster->size(), 5);
    EXPECT_TRUE(cluster->has_rbs());
    EXPECT_FALSE(cluster->has_uvd());
    EXPECT_EQ(cluster->get_min_range(), 50);
    EXPECT_EQ(cluster->get_max_range(), 58);
    EXPECT_TRUE(cluster->is_closed());
    
    const auto& indices = cluster->get_indices();
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

TEST_F(IntegrationTest, MultipleClusters) {
    std::vector<std::pair<uint16_t, uint16_t>> cluster1 = {
        {100, 50}, {105, 52}, {110, 54}
    };
    
    std::vector<std::pair<uint16_t, uint16_t>> cluster2 = {
        {300, 80}, {305, 82}, {310, 84}
    };
    
    for (const auto& [az, range] : cluster1) {
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
    }
    
    for (const auto& [az, range] : cluster2) {
        ScanReplies scan(az, 0);
        RBSReply reply;
        reply.azimuth = az;
        reply.range = range;
        reply.code12 = 0x5678;
        reply.spi = false;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        scan.rbs_replies.push_back(reply);
        clusterer->process_scan(scan);
    }
    
    EXPECT_EQ(count_total_clusters(), 2);
    EXPECT_EQ(count_active_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
}

TEST_F(IntegrationTest, MixedRBSAndUVD) {
    std::vector<std::pair<uint16_t, uint16_t>> points = {
        {100, 50}, {105, 52}, {110, 54}
    };
    
    for (const auto& [az, range] : points) {
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
    }
    
    for (const auto& [az, range] : points) {
        ScanReplies scan(az + 2, 0);
        UVDReply reply;
        reply.azimuth = az + 2;
        reply.range = range + 5;
        reply.data20 = 0x12345;
        reply.is_valid = true;
        reply.error_mask = 0;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[40] = 100;
        scan.uvd_replies.push_back(reply);
        clusterer->process_scan(scan);
    }
    
    EXPECT_EQ(count_total_clusters(), 2);
    EXPECT_EQ(count_active_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = get_closed_clusters();
    EXPECT_EQ(closed.size(), 1);
    if (!closed.empty()) {
        const Cluster* cluster = closed[0];
        EXPECT_TRUE(cluster->has_rbs());
        EXPECT_FALSE(cluster->has_uvd());
    }
    
    auto clusters = ClusterPool::instance().get_all_clusters();
    for (Cluster* cluster : clusters) {
        if (cluster && !cluster->is_closed()) {
            EXPECT_TRUE(cluster->has_uvd());
            EXPECT_FALSE(cluster->has_rbs());
        }
    }
}

TEST_F(IntegrationTest, ActiveAndClosedInPool) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    std::vector<uint64_t> ids;
    
    for (int i = 0; i < 5; ++i) {
        StoredPoint point;
        point.azimuth = 10 + i * 20;
        point.range = 50 + i * 10;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        ids.push_back(id);
        
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        
        if (i % 2 == 0) {
            pool.close_cluster(id, 0);
        }
    }
    
    EXPECT_EQ(count_total_clusters(), 5);
    EXPECT_EQ(count_active_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 3);
    
    auto closed_ids = pool.take_closed_clusters(0);
    EXPECT_EQ(closed_ids.size(), 3);
    
    EXPECT_EQ(count_closed_clusters(), 0);
    EXPECT_EQ(count_active_clusters(), 2);
    EXPECT_EQ(count_total_clusters(), 5);
}

TEST_F(IntegrationTest, CleanupAfterProcessing) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    std::vector<uint64_t> ids;
    
    for (int i = 0; i < 5; ++i) {
        StoredPoint point;
        point.azimuth = 10 + i * 20;
        point.range = 50 + i * 5;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        ids.push_back(id);
        
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        pool.close_cluster(id, 0);
    }
    
    EXPECT_EQ(count_total_clusters(), 5);
    EXPECT_EQ(count_closed_clusters(), 5);
    
    auto closed_ids = pool.take_closed_clusters(0);
    EXPECT_EQ(closed_ids.size(), 5);
    
    EXPECT_EQ(count_closed_clusters(), 0);
    EXPECT_EQ(count_total_clusters(), 5);
    
    size_t cleaned = pool.cleanup_delayed_clusters(0, 0.0);
    EXPECT_EQ(cleaned, 5);
    EXPECT_EQ(count_total_clusters(), 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
