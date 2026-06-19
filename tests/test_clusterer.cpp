// tests/test_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/cluster.h"
#include "vrl/radar/processing/legacy_clusterer.h"
#include <memory>

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
    
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    
    tracker.process_scan(scan);
    
    const auto& clusters = tracker.get_active_clusters();
    EXPECT_EQ(clusters.size(), 1);
}

TEST_F(ClustererTest, ResetClearsClusters) {
    ClusterTracker tracker;
    
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    
    tracker.process_scan(scan);
    EXPECT_EQ(tracker.get_active_clusters().size(), 1);
    
    tracker.reset();
    EXPECT_EQ(tracker.get_active_clusters().size(), 0);
}

TEST_F(ClustererTest, LegacyClustererParameters) {
    auto clusterer = std::make_unique<LegacyClusterer>(5, 20);
    
    // Проверяем, что параметры можно изменить через set_param
    clusterer->set_param("max_gap_azimuth", 15);
    clusterer->set_param("range_window", 40);
    
    // Создаем трекер с этим кластеризатором
    ClusterTracker tracker(std::move(clusterer));
    
    // Проверяем, что кластеризатор работает
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    tracker.process_scan(scan1);
    
    // Пропускаем 10 сканов (больше чем gap=5, но меньше чем 15)
    for (int i = 0; i < 10; ++i) {
        ScanReplies empty_scan = create_scan(101 + i);
        tracker.process_scan(empty_scan);
    }
    
    // Кластер должен быть еще активен (gap=15)
    EXPECT_EQ(tracker.get_active_clusters().size(), 1);
}
