// tests/test_dbscan_clusterer_full.cpp - ПОЛНАЯ ВЕРСИЯ

#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"

using namespace vrl::radar;

class DBSCANClustererFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(10000);
        ClusterPool::instance().init(1000);
        
        radar_config_.range_bin_rbs = 30.0;
        radar_config_.range_bin_uvd = 60.0;
        radar_config_.beamwidth_deg = 5.0;
        radar_config_.min_amplitude = 10;
        
        clusterer_ = std::make_unique<DBSCANClusterer>(
            radar_config_,
            3,    // max_range_gap
            1.2   // azimuth_gap_coefficient
        );
        
        clusterer_->set_debug(true);
    }
    
    void TearDown() override {
        clusterer_->reset();
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
    std::unique_ptr<DBSCANClusterer> clusterer_;
};

// ===== БАЗОВЫЕ ТЕСТЫ =====

TEST_F(DBSCANClustererFullTest, Init) {
    EXPECT_NE(clusterer_.get(), nullptr);
    EXPECT_EQ(clusterer_->get_name(), "DBSCANClusterer");
}

TEST_F(DBSCANClustererFullTest, ProcessSingleScan) {
    auto scan = create_scan(512, {100, 101, 102});
    clusterer_->process_scan(scan);
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, ProcessMultipleScans) {
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, ClusterCreation) {
    std::vector<int> azimuths = {512, 513, 514, 515, 516};
    for (int az : azimuths) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, ClusterMerging) {
    for (int az = 500; az < 505; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    for (int az = 505; az < 510; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_LE(active_count, 2);
}

TEST_F(DBSCANClustererFullTest, UVDClusters) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100}, false);
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, MixedClusters) {
    for (int az = 512; az < 516; ++az) {
        auto scan = create_scan(az, {100}, true);
        clusterer_->process_scan(scan);
    }
    
    for (int az = 514; az < 518; ++az) {
        auto scan = create_scan(az, {100}, false);
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, WideClusterDetection) {
    for (int az = 400; az < 500; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, Reset) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    clusterer_->reset();
    
    const auto& active = clusterer_->get_active_clusters();
    EXPECT_TRUE(active.empty());
}

TEST_F(DBSCANClustererFullTest, SetParameters) {
    clusterer_->set_param("max_range_gap", 5);
    EXPECT_EQ(clusterer_->get_max_range_gap(), 5);
    
    clusterer_->set_param("azimuth_gap_coefficient", 1.5);
    EXPECT_EQ(clusterer_->get_max_azimuth_gap(), 
              static_cast<int>(5.0 * 4096 / 360 * 1.5));
}

TEST_F(DBSCANClustererFullTest, SetWideClusterThreshold) {
    clusterer_->set_wide_cluster_threshold(100);
    EXPECT_EQ(clusterer_->get_wide_cluster_threshold(), 100);
}

TEST_F(DBSCANClustererFullTest, GetStats) {
    size_t active = 0;
    size_t completed = 0;
    
    clusterer_->get_stats(active, completed);
    
    EXPECT_EQ(active, 0);
    EXPECT_EQ(completed, 0);
}

TEST_F(DBSCANClustererFullTest, LargeCluster) {
    for (int az = 400; az < 500; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_EQ(active_count, 1);
}

TEST_F(DBSCANClustererFullTest, MultipleRangeGaps) {
    std::vector<int> ranges = {100, 102, 104, 106, 108, 200, 202, 204};
    for (int range : ranges) {
        auto scan = create_scan(512, {static_cast<uint16_t>(range)});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GE(active_count, 2);
}

// ===== ИСПРАВЛЕННЫЕ ТЕСТЫ =====

TEST_F(DBSCANClustererFullTest, MultipleClusters) {
    // Кластер 1: азимуты 100-109, дальность 100
    for (int az = 100; az < 110; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Кластер 2: азимуты 1500-1509, дальность 500 (очень далеко)
    for (int az = 1500; az < 1510; ++az) {
        auto scan = create_scan(az, {500});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GE(active_count, 2);
}

TEST_F(DBSCANClustererFullTest, RangeGap) {
    // Точки с дальностью 100 (азимуты 200-209)
    for (int az = 200; az < 210; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Точки с дальностью 500 (азимуты 1800-1809)
    for (int az = 1800; az < 1810; ++az) {
        auto scan = create_scan(az, {500});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GE(active_count, 2);
}

TEST_F(DBSCANClustererFullTest, ClusterClosure) {
    // Создаем широкий кластер (азимутальный размах > 68)
    for (int az = 300; az < 400; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Закрываем устаревшие кластеры
    clusterer_->close_expired_clusters(500);
    
    auto completed = clusterer_->get_completed_clusters();
    EXPECT_GT(completed.size(), 0);
}

TEST_F(DBSCANClustererFullTest, FinishAll) {
    // Создаем два широких кластера
    // Кластер 1: 400-460, range=100
    for (int az = 400; az < 460; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Кластер 2: 1000-1060, range=500
    for (int az = 1000; az < 1060; ++az) {
        auto scan = create_scan(az, {500});
        clusterer_->process_scan(scan);
    }
    
    // Проверяем, что есть активные кластеры
    size_t active_before = clusterer_->count_active_clusters();
    EXPECT_GT(active_before, 0);
    
    // Завершаем все кластеры
    auto completed = clusterer_->finish_all();
    
    // Проверяем, что активных кластеров не осталось
    const auto& active = clusterer_->get_active_clusters();
    EXPECT_TRUE(active.empty());
}
