// tests/test_cluster_tracker.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/cluster.h"
#include <memory>

using namespace vrl::radar;

class ClusterTrackerTest : public ::testing::Test {
protected:
    void SetUp() override {
        tracker_ = std::make_unique<ClusterTracker>(8, 30);
    }
    
    RBSReply create_rbs_reply(uint16_t azimuth, uint16_t range, uint16_t code) {
        RBSReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.code12 = code;
        reply.is_valid = true;
        
        // Устанавливаем фреймовые импульсы для валидности
        reply.ether_amplitudes[0] = 100;   // F1
        reply.ether_amplitudes[14] = 100;  // F2
        
        return reply;
    }
    
    UVDReply create_uvd_reply(uint16_t azimuth, uint16_t range, uint32_t data) {
        UVDReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.data20 = data;
        reply.is_valid = true;
        return reply;
    }
    
    ScanReplies create_scan(uint16_t azimuth, uint32_t timestamp = 0) {
        return ScanReplies(azimuth, timestamp);
    }
    
    std::unique_ptr<ClusterTracker> tracker_;
};

TEST_F(ClusterTrackerTest, EmptyScanDoesNotCreateCluster) {
    ScanReplies scan = create_scan(100);
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 0);
    auto completed = tracker_->get_completed_clusters();
    EXPECT_TRUE(completed.empty());
}

TEST_F(ClusterTrackerTest, SingleReplyCreatesCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
}

TEST_F(ClusterTrackerTest, MultipleRepliesSameCluster) {
    // Создаем несколько сканов с близкими азимутами и дальностями
    for (int i = 0; i < 5; ++i) {
        ScanReplies scan = create_scan(100 + i);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, 50 + i, 1234));
        tracker_->process_scan(scan);
    }
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    // Проверяем, что все реплики в одном кластере
    const auto& clusters = tracker_->get_active_clusters();
    EXPECT_EQ(clusters[0].rbs_by_azimuth.size(), 5);
}

TEST_F(ClusterTrackerTest, DifferentRangesCreateSeparateClusters) {
    // Первая цель на дальности 50
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    tracker_->process_scan(scan1);
    
    // Вторая цель на дальности 200 (далеко от первой)
    ScanReplies scan2 = create_scan(102);
    scan2.rbs_replies.push_back(create_rbs_reply(102, 200, 5678));
    tracker_->process_scan(scan2);
    
    // Должно быть два кластера
    EXPECT_EQ(tracker_->get_active_clusters().size(), 2);
}

TEST_F(ClusterTrackerTest, ClusterCompletesAfterGap) {
    // Создаем кластер с несколькими репликами
    for (int i = 0; i < 5; ++i) {
        ScanReplies scan = create_scan(100 + i);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, 50, 1234));
        tracker_->process_scan(scan);
    }
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    // Пропускаем несколько сканов (больше чем max_gap_azimuth)
    for (int i = 0; i < 10; ++i) {
        ScanReplies empty_scan = create_scan(105 + i);
        tracker_->process_scan(empty_scan);
    }
    
    auto completed = tracker_->get_completed_clusters();
    EXPECT_FALSE(completed.empty());
    EXPECT_EQ(tracker_->get_active_clusters().size(), 0);
}

TEST_F(ClusterTrackerTest, DifferentCodesInSameCluster) {
    // Разные коды, но близкие по дальности и азимуту
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(101, 52, 5678));
    
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
}

TEST_F(ClusterTrackerTest, UVDAndRBSInSameCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.uvd_replies.push_back(create_uvd_reply(101, 52, 12345));
    
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    const auto& clusters = tracker_->get_active_clusters();
    EXPECT_EQ(clusters[0].rbs_by_azimuth.size(), 1);
    EXPECT_EQ(clusters[0].uvd_by_azimuth.size(), 1);
}

TEST_F(ClusterTrackerTest, ResetClearsClusters) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    tracker_->reset();
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 0);
    auto completed = tracker_->get_completed_clusters();
    EXPECT_TRUE(completed.empty());
}

TEST_F(ClusterTrackerTest, DifferentAzimuthThresholds) {
    ClusterTracker tracker_small_gap(2, 30);
    ClusterTracker tracker_large_gap(20, 30);
    
    // Создаем сканы с разрывом 5 азимутов
    for (int i = 0; i < 3; ++i) {
        ScanReplies scan = create_scan(100 + i * 3);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i * 3, 50, 1234));
        tracker_small_gap.process_scan(scan);
        tracker_large_gap.process_scan(scan);
    }
    
    // С маленьким разрывом (2) кластер должен закрыться
    auto completed_small = tracker_small_gap.get_completed_clusters();
    EXPECT_FALSE(completed_small.empty());
    
    // С большим разрывом (20) кластер должен остаться активным
    EXPECT_EQ(tracker_large_gap.get_active_clusters().size(), 1);
}
