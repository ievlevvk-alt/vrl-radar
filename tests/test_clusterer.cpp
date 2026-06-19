// tests/test_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/processing/legacy_clusterer.h"
#include <memory>
#include <iostream>

using namespace vrl::radar;

class ClustererTest : public ::testing::Test {
protected:
    RBSReply create_rbs_reply(uint16_t azimuth, uint16_t range, uint16_t code) {
        RBSReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.code12 = code;
        reply.is_valid = true;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        return reply;
    }
    
    ScanReplies create_scan(uint16_t azimuth, uint32_t timestamp = 0) {
        return ScanReplies(azimuth, timestamp);
    }
};

TEST_F(ClustererTest, DefaultLegacyClusterer) {
    ClusterTracker tracker;
    EXPECT_EQ(tracker.get_algorithm_name(), "LegacyClusterer");
    EXPECT_NE(tracker.get_clusterer(), nullptr);
}

TEST_F(ClustererTest, CustomClusterer) {
    auto clusterer = std::make_unique<LegacyClusterer>(5, 20);
    ClusterTracker tracker(std::move(clusterer));
    
    EXPECT_EQ(tracker.get_algorithm_name(), "LegacyClusterer");
    EXPECT_NE(tracker.get_clusterer(), nullptr);
}

TEST_F(ClustererTest, ChangeClusterer) {
    ClusterTracker tracker;
    EXPECT_EQ(tracker.get_algorithm_name(), "LegacyClusterer");
    
    auto new_clusterer = std::make_unique<LegacyClusterer>(10, 50);
    tracker.set_clusterer(std::move(new_clusterer));
    
    EXPECT_EQ(tracker.get_algorithm_name(), "LegacyClusterer");
}

TEST_F(ClustererTest, ProcessScanCreatesCluster) {
    ClusterTracker tracker;
    tracker.set_max_revolutions_no_update(10);
    
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    
    tracker.process_scan(scan);
    
    const auto& clusters = tracker.get_active_clusters();
    EXPECT_EQ(clusters.size(), 1);
}

TEST_F(ClustererTest, ResetClearsClusters) {
    ClusterTracker tracker;
    tracker.set_max_revolutions_no_update(10);
    
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    
    tracker.process_scan(scan);
    EXPECT_EQ(tracker.get_active_clusters().size(), 1);
    
    tracker.reset();
    EXPECT_EQ(tracker.get_active_clusters().size(), 0);
}

TEST_F(ClustererTest, LegacyClustererParameters) {
    auto clusterer = std::make_unique<LegacyClusterer>(3, 30);
    
    // Меняем параметры
    clusterer->set_param("max_gap_azimuth", 15);
    clusterer->set_param("range_window", 50);
    
    // Создаем трекер
    ClusterTracker tracker(std::move(clusterer));
    tracker.set_max_revolutions_no_update(20);
    
    // ВАЖНО: устанавливаем параметры в трекере!
    tracker.set_max_gap_azimuth(15);   // <-- ЭТО ВАЖНО!
    tracker.set_range_window(50);
    
    // Создаем кластер
    for (int i = 0; i < 3; ++i) {
        ScanReplies scan = create_scan(100 + i);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, 50 + i, 1234));
        tracker.process_scan(scan);
    }
    
    EXPECT_EQ(tracker.get_active_clusters().size(), 1);
    
    // Пропускаем 10 пустых сканов
    for (int i = 0; i < 10; ++i) {
        ScanReplies empty_scan = create_scan(103 + i);
        tracker.process_scan(empty_scan);
    }
    
    EXPECT_EQ(tracker.get_active_clusters().size(), 1);
}
