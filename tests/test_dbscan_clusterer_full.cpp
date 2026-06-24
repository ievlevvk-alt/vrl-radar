// tests/test_dbscan_clusterer_full.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/cluster.h"  // <-- ДОБАВЛЯЕМ для TargetCluster
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

// ===== ТЕСТЫ БАЗОВОЙ ФУНКЦИОНАЛЬНОСТИ =====

TEST_F(DBSCANClustererFullTest, Init) {
    EXPECT_NE(clusterer_.get(), nullptr);
    EXPECT_EQ(clusterer_->get_name(), "DBSCANClusterer");
}

TEST_F(DBSCANClustererFullTest, ProcessSingleScan) {
    auto scan = create_scan(512, {100, 101, 102});
    clusterer_->process_scan(scan);
    
    const auto& active = clusterer_->get_active_clusters();
    EXPECT_GT(active.size(), 0);
}

TEST_F(DBSCANClustererFullTest, ProcessMultipleScans) {
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    const auto& active = clusterer_->get_active_clusters();
    EXPECT_GT(active.size(), 0);
}

// ===== ТЕСТЫ КЛАСТЕРИЗАЦИИ =====

TEST_F(DBSCANClustererFullTest, ClusterCreation) {
    std::vector<int> azimuths = {512, 513, 514, 515, 516};
    for (int az : azimuths) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, MultipleClusters) {
    // Кластер 1: 500-504
    for (int az = 500; az < 505; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Кластер 2: 600-604
    for (int az = 600; az < 605; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GE(active_count, 2);
}

TEST_F(DBSCANClustererFullTest, ClusterMerging) {
    // Кластер A: 500-504
    for (int az = 500; az < 505; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Кластер B: 505-509 (перекрывается)
    for (int az = 505; az < 510; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_LE(active_count, 2);
}

TEST_F(DBSCANClustererFullTest, RangeGap) {
    // Точки с дальностью 100
    for (int az = 512; az < 516; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    // Точки с дальностью 200 (разрыв больше max_range_gap=3)
    for (int az = 512; az < 516; ++az) {
        auto scan = create_scan(az, {200});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GE(active_count, 2);
}

// ===== ТЕСТЫ UVD =====

TEST_F(DBSCANClustererFullTest, UVDClusters) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100}, false);
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

TEST_F(DBSCANClustererFullTest, MixedClusters) {
    // RBS точки
    for (int az = 512; az < 516; ++az) {
        auto scan = create_scan(az, {100}, true);
        clusterer_->process_scan(scan);
    }
    
    // UVD точки (пересекаются)
    for (int az = 514; az < 518; ++az) {
        auto scan = create_scan(az, {100}, false);
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

// ===== ТЕСТЫ ЗАКРЫТИЯ КЛАСТЕРОВ =====

TEST_F(DBSCANClustererFullTest, ClusterClosure) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    clusterer_->close_expired_clusters(600);
    
    auto completed = clusterer_->get_completed_clusters();
    EXPECT_GT(completed.size(), 0);
}

TEST_F(DBSCANClustererFullTest, WideClusterDetection) {
    for (int az = 400; az < 500; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    size_t active_count = clusterer_->count_active_clusters();
    EXPECT_GT(active_count, 0);
}

// ===== ТЕСТЫ ОЧИСТКИ =====

TEST_F(DBSCANClustererFullTest, Reset) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    clusterer_->reset();
    
    const auto& active = clusterer_->get_active_clusters();
    EXPECT_TRUE(active.empty());
}

TEST_F(DBSCANClustererFullTest, FinishAll) {
    for (int az = 512; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
    }
    
    auto completed = clusterer_->finish_all();
    EXPECT_GT(completed.size(), 0);
}

// ===== ТЕСТЫ ПАРАМЕТРОВ =====

TEST_F(DBSCANClustererFullTest, SetParameters) {
    clusterer_->set_param("max_range_gap", 5);
    EXPECT_EQ(clusterer_->get_max_range_gap(), 5);
    
    clusterer_->set_param("azimuth_gap_coefficient", 1.5);
    // Проверяем через публичный метод
    EXPECT_EQ(clusterer_->get_max_azimuth_gap(), 
              static_cast<int>(5.0 * 4096 / 360 * 1.5));
}

TEST_F(DBSCANClustererFullTest, SetWideClusterThreshold) {
    clusterer_->set_wide_cluster_threshold(100);
    EXPECT_EQ(clusterer_->get_wide_cluster_threshold(), 100);
}

// ===== ТЕСТЫ СТАТИСТИКИ =====

TEST_F(DBSCANClustererFullTest, GetStats) {
    size_t active = 0;
    size_t completed = 0;
    
    clusterer_->get_stats(active, completed);
    
    EXPECT_EQ(active, 0);
    EXPECT_EQ(completed, 0);
}

// ===== ТЕСТЫ С МНОЖЕСТВОМ ТОЧЕК =====

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
