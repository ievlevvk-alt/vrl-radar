// tests/test_dbscan_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/processing/cluster.h"
#include <memory>
#include <cmath>
#include <iostream>

using namespace vrl::radar;

class DBSCANClustererTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.beamwidth_deg = 5.0;
        config_.range_bin_rbs = 30.0;
        config_.range_bin_uvd = 60.0;
        config_.min_amplitude = 10;
        
        PointBuffer::instance().init(1000);
        ClusterPool::instance().clear();
        
        // ИСПРАВЛЕНО: 3 аргумента
        clusterer_ = std::make_unique<DBSCANClusterer>(config_, 3, 1.2);
        clusterer_->set_debug(false);
    }
    
    void TearDown() override {
        ClusterPool::instance().clear();
    }
    
    RBSReply create_rbs_reply(uint16_t azimuth, uint16_t range, 
                              uint16_t code = 1234, bool spi = false) {
        RBSReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.code12 = code;
        reply.spi = spi;
        reply.is_valid = true;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        reply.ether_amplitudes[17] = spi ? 100 : 0;
        return reply;
    }
    
    UVDReply create_uvd_reply(uint16_t azimuth, uint16_t range, 
                              uint32_t data = 12345) {
        UVDReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.data20 = data;
        reply.is_valid = true;
        for (auto& amp : reply.ether_amplitudes) {
            amp = 50;
        }
        return reply;
    }
    
    ScanReplies create_scan(uint16_t azimuth) {
        return ScanReplies(azimuth, 0);
    }
    
    size_t count_clusters() const {
        return ClusterPool::instance().size();
    }
    
    size_t count_active_clusters() const {
        return ClusterPool::instance().get_active_clusters().size();
    }
    
    size_t count_closed_clusters() const {
        return ClusterPool::instance().get_closed_clusters().size();
    }
    
    RadarConfig config_;
    std::unique_ptr<DBSCANClusterer> clusterer_;
};

// ========================================================================
// ТЕСТ 1: Базовое создание кластера из двух точек
// ========================================================================

TEST_F(DBSCANClustererTest, TwoPointsFormCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->size(), 2);
}

// ========================================================================
// ТЕСТ 2: Две точки слишком далеко - два кластера
// ========================================================================

TEST_F(DBSCANClustererTest, PointsTooFarTwoClusters) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(200, 52));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 3: Точки далеко по дальности - два кластера
// ========================================================================

TEST_F(DBSCANClustererTest, PointsTooFarRangeTwoClusters) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 100));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 4: RBS и UVD не смешиваются
// ========================================================================

TEST_F(DBSCANClustererTest, RBSAndUVDSeparateClusters) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.uvd_replies.push_back(create_uvd_reply(102, 52));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 5: Объединение кластеров
// ========================================================================

TEST_F(DBSCANClustererTest, ClusterMergeScenario) {
    clusterer_->set_debug(true);  // ← добавить
    
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 100));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(101);
    scan2.rbs_replies.push_back(create_rbs_reply(101, 101));
    scan2.rbs_replies.push_back(create_rbs_reply(101, 105));
    clusterer_->process_scan(scan2);
    
    ScanReplies scan3 = create_scan(102);
    scan3.rbs_replies.push_back(create_rbs_reply(102, 103));
    scan3.rbs_replies.push_back(create_rbs_reply(102, 105));
    clusterer_->process_scan(scan3);
    
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->size(), 5);
}

// ========================================================================
// ТЕСТ 6: Закрытие кластера при разрыве
// ========================================================================

TEST_F(DBSCANClustererTest, ClusterClosesOnGap) {
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan1.rbs_replies.push_back(create_rbs_reply(102, 52));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(200);
    scan2.rbs_replies.push_back(create_rbs_reply(200, 55));
    clusterer_->process_scan(scan2);
    
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 7: Переход через Север
// ========================================================================

TEST_F(DBSCANClustererTest, NorthTransition) {
    ScanReplies scan1 = create_scan(4094);
    scan1.rbs_replies.push_back(create_rbs_reply(4094, 50));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(0);
    scan2.rbs_replies.push_back(create_rbs_reply(0, 52));
    clusterer_->process_scan(scan2);
    
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->get_azimuth_span(), 2);
}

// ========================================================================
// ТЕСТ 8: SPI флаг сохраняется
// ========================================================================

TEST_F(DBSCANClustererTest, SPIFlagPreserved) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234, true));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52, 1234, false));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    
    const auto& indices = closed[0]->get_indices();
    bool found_spi = false;
    for (size_t idx : indices) {
        const auto& point = PointBuffer::instance().get_point(idx);
        if (point.spi) found_spi = true;
    }
    EXPECT_TRUE(found_spi);
}

// ========================================================================
// ТЕСТ 9: Несколько ответов на одном азимуте
// ========================================================================

TEST_F(DBSCANClustererTest, MultipleRepliesSameAzimuth) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(100, 52));
    scan.rbs_replies.push_back(create_rbs_reply(100, 60));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 10: Сброс и статистика
// ========================================================================

TEST_F(DBSCANClustererTest, ResetAndStats) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52));
    clusterer_->process_scan(scan);
    
    size_t active, completed;
    clusterer_->get_stats(active, completed);
    
    EXPECT_TRUE(active > 0 || completed > 0);
    
    clusterer_->reset();
    clusterer_->get_stats(active, completed);
    
    EXPECT_EQ(active, 0);
    EXPECT_EQ(completed, 0);
}

// ========================================================================
// ТЕСТ 11: Параметры через set_param
// ========================================================================

TEST_F(DBSCANClustererTest, SetParams) {
    int default_gap = clusterer_->get_max_azimuth_gap();
    EXPECT_EQ(default_gap, 68);
    
    clusterer_->set_param("max_azimuth_gap", 50);
    clusterer_->set_param("max_range_gap", 5);
    
    EXPECT_EQ(clusterer_->get_max_azimuth_gap(), 50);
    EXPECT_EQ(clusterer_->get_max_range_gap(), 5);
}

// ========================================================================
// ТЕСТ 12: Вычисление азимутального размаха
// ========================================================================

TEST_F(DBSCANClustererTest, AzimuthSpanNorth) {
    ScanReplies scan1 = create_scan(4094);
    scan1.rbs_replies.push_back(create_rbs_reply(4094, 50));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(0);
    scan2.rbs_replies.push_back(create_rbs_reply(0, 52));
    clusterer_->process_scan(scan2);
    
    ScanReplies scan3 = create_scan(2);
    scan3.rbs_replies.push_back(create_rbs_reply(2, 54));
    clusterer_->process_scan(scan3);
    
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->size(), 3);
}

// ========================================================================
// ТЕСТ 13: Точки на границе допуска по дальности
// ========================================================================

TEST_F(DBSCANClustererTest, RangeBoundary) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 53));
    scan.rbs_replies.push_back(create_rbs_reply(104, 57));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 2);
}

// ========================================================================
// ТЕСТ 14: Разные коды RBS в одном кластере
// ========================================================================

TEST_F(DBSCANClustererTest, DifferentCodesSameCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52, 5678));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->size(), 2);
}

// ========================================================================
// ТЕСТ 15: Проверка azimuth_distance
// ========================================================================

TEST_F(DBSCANClustererTest, AzimuthDistance) {
    ScanReplies scan1 = create_scan(4094);
    scan1.rbs_replies.push_back(create_rbs_reply(4094, 50));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(0);
    scan2.rbs_replies.push_back(create_rbs_reply(0, 52));
    clusterer_->process_scan(scan2);
    
    auto completed = clusterer_->finish_all();
    
    EXPECT_EQ(count_clusters(), 1);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0]->size(), 2);
    EXPECT_EQ(closed[0]->get_azimuth_span(), 2);
}
