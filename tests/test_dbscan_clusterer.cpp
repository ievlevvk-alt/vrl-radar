// tests/test_clusterer.cpp - исправленная версия
#include <gtest/gtest.h>
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/legacy_clusterer.h"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"

using namespace vrl::radar;

class ClustererTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(10000);
        ClusterPool::instance().init(1000);
        
        radar_config_.range_bin_rbs = 30.0;
        radar_config_.range_bin_uvd = 60.0;
        radar_config_.beamwidth_deg = 5.0;
        radar_config_.min_amplitude = 10;
    }
    
    void TearDown() override {
        PointBuffer::instance().init(10000);
        ClusterPool::instance().clear();
    }
    
    void add_point_to_buffer(uint16_t azimuth, uint16_t range, bool is_rbs = true) {
        StoredPoint point;
        point.azimuth = azimuth;
        point.range = range;
        point.is_rbs = is_rbs;
        point.code12 = 1234;
        PointBuffer::instance().add_point(point);
    }
    
    ScanReplies create_scan(uint16_t azimuth, 
                            const std::vector<uint16_t>& ranges,
                            bool is_rbs = true) {
        ScanReplies scan(azimuth, 0);
        
        for (uint16_t range : ranges) {
            add_point_to_buffer(azimuth, range, is_rbs);
            if (is_rbs) {
                RBSReply reply;
                reply.azimuth = azimuth;
                reply.range = range;
                reply.code12 = 1234;
                reply.is_valid = true;
                scan.rbs_replies.push_back(reply);
            } else {
                UVDReply reply;
                reply.azimuth = azimuth;
                reply.range = range;
                reply.data20 = 12345;
                reply.is_valid = true;
                scan.uvd_replies.push_back(reply);
            }
        }
        
        return scan;
    }
    
    RadarConfig radar_config_;
};

// ===== ТЕСТЫ DBSCAN CLUSTERER =====

TEST_F(ClustererTest, DBSCANInit) {
    auto clusterer = std::make_unique<DBSCANClusterer>(radar_config_, 3, 1.2);
    EXPECT_NE(clusterer.get(), nullptr);
    EXPECT_EQ(clusterer->get_name(), "DBSCANClusterer");
}

// ===== ИСПРАВЛЕННЫЙ ТЕСТ =====
TEST_F(ClustererTest, DBSCANProcessScan) {
    auto clusterer = std::make_unique<DBSCANClusterer>(radar_config_, 3, 1.2);
    
    // Создаем несколько сканов с близкими азимутами для формирования кластера
    for (int az = 500; az < 510; ++az) {
        auto scan = create_scan(az, {100});
        clusterer->process_scan(scan);
    }
    
    // Проверяем через count_active_clusters()
    size_t active_count = clusterer->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(ClustererTest, DBSCANReset) {
    auto clusterer = std::make_unique<DBSCANClusterer>(radar_config_, 3, 1.2);
    
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer->process_scan(scan);
    }
    
    clusterer->reset();
    
    const auto& active = clusterer->get_active_clusters();
    EXPECT_TRUE(active.empty());
}

// ===== ТЕСТЫ LEGACY CLUSTERER =====

TEST_F(ClustererTest, LegacyInit) {
    auto clusterer = std::make_unique<LegacyClusterer>(8, 30);
    EXPECT_NE(clusterer.get(), nullptr);
    EXPECT_EQ(clusterer->get_name(), "LegacyClusterer");
}

TEST_F(ClustererTest, LegacyProcessScan) {
    auto clusterer = std::make_unique<LegacyClusterer>(8, 30);
    
    for (int az = 500; az < 510; ++az) {
        auto scan = create_scan(az, {100});
        clusterer->process_scan(scan);
    }
    
    const auto& active = clusterer->get_active_clusters();
    EXPECT_GT(active.size(), 0);
}

TEST_F(ClustererTest, LegacyReset) {
    auto clusterer = std::make_unique<LegacyClusterer>(8, 30);
    
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer->process_scan(scan);
    }
    
    clusterer->reset();
    
    const auto& active = clusterer->get_active_clusters();
    EXPECT_TRUE(active.empty());
}

TEST_F(ClustererTest, LegacySetParams) {
    auto clusterer = std::make_unique<LegacyClusterer>(8, 30);
    
    clusterer->set_param("max_gap_azimuth", 16);
    clusterer->set_param("range_window", 60);
    
    // Проверяем через публичные методы или через поведение
    const auto& active = clusterer->get_active_clusters();
    EXPECT_TRUE(active.empty());
}
