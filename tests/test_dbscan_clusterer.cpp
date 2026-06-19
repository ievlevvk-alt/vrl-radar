// tests/test_dbscan_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
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
        
        clusterer_ = std::make_unique<DBSCANClusterer>(config_, 3, 1, 1.2);
        clusterer_->set_debug(false);
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
    
    ASSERT_EQ(completed.size(), 1) << "Should have 1 cluster";
    EXPECT_GE(completed[0].scans.size(), 1) << "Cluster should have scans";
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
    
    ASSERT_EQ(completed.size(), 2) << "Should have 2 clusters (too far)";
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
    
    ASSERT_EQ(completed.size(), 2) << "Should have 2 clusters (range too far)";
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
    
    ASSERT_EQ(completed.size(), 2) << "RBS and UVD should be separate clusters";
}

// ========================================================================
// ТЕСТ 5: Объединение кластеров
// ========================================================================

TEST_F(DBSCANClustererTest, ClusterMergeScenario) {
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
    
    ASSERT_EQ(completed.size(), 1) << "Should have 1 merged cluster";
    EXPECT_GE(completed[0].scans.size(), 3) << "Cluster should have 3 azimuths";
    
    int total_replies = 0;
    for (const auto& scan : completed[0].scans) {
        total_replies += scan.rbs_replies.size();
    }
    EXPECT_EQ(total_replies, 5) << "Should have 5 RBS replies total";
}

// ========================================================================
// ТЕСТ 6: Минимальное количество точек
// ========================================================================

TEST_F(DBSCANClustererTest, MinPointsFilter) {
    clusterer_->set_min_points(3);
    
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    EXPECT_TRUE(completed.empty()) << "Cluster with 2 points should be discarded";
}

// ========================================================================
// ТЕСТ 7: Закрытие кластера при разрыве
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
    
    ASSERT_EQ(completed.size(), 2) << "Should have 2 clusters (gap closed first)";
}

// ========================================================================
// ТЕСТ 8: Переход через Север
// ========================================================================

TEST_F(DBSCANClustererTest, NorthTransition) {
    ScanReplies scan1 = create_scan(4094);
    scan1.rbs_replies.push_back(create_rbs_reply(4094, 50));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(0);
    scan2.rbs_replies.push_back(create_rbs_reply(0, 52));
    clusterer_->process_scan(scan2);
    
    auto completed = clusterer_->finish_all();
    
    ASSERT_EQ(completed.size(), 1) << "Should merge across North transition";
    
    bool has_4094 = false, has_0 = false;
    for (const auto& scan : completed[0].scans) {
        if (scan.azimuth == 4094) has_4094 = true;
        if (scan.azimuth == 0) has_0 = true;
    }
    EXPECT_TRUE(has_4094 && has_0) << "Should contain both azimuths 4094 and 0";
}

// ========================================================================
// ТЕСТ 9: SPI флаг сохраняется
// ========================================================================

TEST_F(DBSCANClustererTest, SPIFlagPreserved) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234, true));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52, 1234, false));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    ASSERT_EQ(completed.size(), 1) << "Should have 1 cluster";
    
    bool found_spi = false;
    for (const auto& scan_replies : completed[0].scans) {
        for (const auto& reply : scan_replies.rbs_replies) {
            if (reply.spi) found_spi = true;
        }
    }
    EXPECT_TRUE(found_spi) << "SPI flag should be preserved in replies";
}

// ========================================================================
// ТЕСТ 10: Несколько ответов на одном азимуте
// ========================================================================

TEST_F(DBSCANClustererTest, MultipleRepliesSameAzimuth) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(100, 52));
    scan.rbs_replies.push_back(create_rbs_reply(100, 60));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    ASSERT_EQ(completed.size(), 2) << "Should have 2 clusters (range gap)";
}

// ========================================================================
// ТЕСТ 11: Сброс и статистика
// ========================================================================

TEST_F(DBSCANClustererTest, ResetAndStats) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52));
    clusterer_->process_scan(scan);
    
    // Проверяем, что после обработки есть кластеры (активные или завершенные)
    size_t active, completed;
    clusterer_->get_stats(active, completed);
    EXPECT_TRUE(active > 0 || completed > 0) 
        << "Should have clusters (active=" << active << ", completed=" << completed << ")";
    
    // Сбрасываем
    clusterer_->reset();
    clusterer_->get_stats(active, completed);
    
    EXPECT_EQ(active, 0) << "After reset active should be 0";
    EXPECT_EQ(completed, 0) << "After reset completed should be 0";
}

// ========================================================================
// ТЕСТ 12: Параметры через set_param
// ========================================================================

TEST_F(DBSCANClustererTest, SetParams) {
    int default_gap = clusterer_->get_max_azimuth_gap();
    EXPECT_EQ(default_gap, 68) << "Default gap should be 68";
    
    clusterer_->set_param("max_azimuth_gap", 50);
    clusterer_->set_param("max_range_gap", 5);
    clusterer_->set_param("min_points", 4);
    
    EXPECT_EQ(clusterer_->get_max_azimuth_gap(), 50);
    EXPECT_EQ(clusterer_->get_max_range_gap(), 5);
    EXPECT_EQ(clusterer_->get_min_points(), 4);
}

// ========================================================================
// ТЕСТ 13: Вычисление азимутального размаха
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
    
    ASSERT_EQ(completed.size(), 1) << "Should have 1 cluster across North";
    
    bool has_4094 = false, has_0 = false, has_2 = false;
    for (const auto& scan : completed[0].scans) {
        if (scan.azimuth == 4094) has_4094 = true;
        if (scan.azimuth == 0) has_0 = true;
        if (scan.azimuth == 2) has_2 = true;
    }
    EXPECT_TRUE(has_4094 && has_0 && has_2) << "Should contain all three azimuths";
}

// ========================================================================
// ТЕСТ 14: Точки на границе допуска по дальности
// ========================================================================

TEST_F(DBSCANClustererTest, RangeBoundary) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50));
    scan.rbs_replies.push_back(create_rbs_reply(102, 53));
    scan.rbs_replies.push_back(create_rbs_reply(104, 57));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    ASSERT_EQ(completed.size(), 2) << "Should have 2 clusters (one at boundary, one outside)";
}

// ========================================================================
// ТЕСТ 15: Разные коды RBS в одном кластере
// ========================================================================

TEST_F(DBSCANClustererTest, DifferentCodesSameCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(102, 52, 5678));
    
    clusterer_->process_scan(scan);
    auto completed = clusterer_->finish_all();
    
    ASSERT_EQ(completed.size(), 1) << "Should have 1 cluster despite different codes";
    
    bool has_1234 = false, has_5678 = false;
    for (const auto& scan_replies : completed[0].scans) {
        for (const auto& reply : scan_replies.rbs_replies) {
            if (reply.code12 == 1234) has_1234 = true;
            if (reply.code12 == 5678) has_5678 = true;
        }
    }
    EXPECT_TRUE(has_1234 && has_5678) << "Both codes should be preserved";
}

// ========================================================================
// ТЕСТ 16: Проверка azimuth_distance
// ========================================================================

TEST_F(DBSCANClustererTest, AzimuthDistance) {
    ScanReplies scan1 = create_scan(4094);
    scan1.rbs_replies.push_back(create_rbs_reply(4094, 50));
    clusterer_->process_scan(scan1);
    
    ScanReplies scan2 = create_scan(0);
    scan2.rbs_replies.push_back(create_rbs_reply(0, 52));
    clusterer_->process_scan(scan2);
    
    auto completed = clusterer_->finish_all();
    ASSERT_EQ(completed.size(), 1) << "Should merge across North";
    
    bool has_4094 = false, has_0 = false;
    for (const auto& scan : completed[0].scans) {
        if (scan.azimuth == 4094) has_4094 = true;
        if (scan.azimuth == 0) has_0 = true;
    }
    EXPECT_TRUE(has_4094 && has_0);
}

