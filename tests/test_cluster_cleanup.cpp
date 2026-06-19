// tests/test_cluster_cleanup.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/cluster.h"
#include <memory>

using namespace vrl::radar;

class ClusterCleanupTest : public ::testing::Test {
protected:
    void SetUp() override {
        tracker_ = std::make_unique<ClusterTracker>(8, 30);
        tracker_->set_max_revolutions_no_update(2);
        tracker_->set_max_active_clusters(5);
    }
    
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
    
    std::unique_ptr<ClusterTracker> tracker_;
};

TEST_F(ClusterCleanupTest, ClusterExpiresAfterRevolutions) {
    // Создаем кластер
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    tracker_->process_scan(scan);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    // Пропускаем обороты
    for (int i = 0; i < 15; ++i) {
        ScanReplies empty_scan = create_scan(100 + i + 1);
        tracker_->process_scan(empty_scan);
    }
    
    // Кластер должен быть завершен
    auto completed = tracker_->get_completed_clusters();
    size_t active = tracker_->get_active_clusters().size();
    
    // Проверяем, что кластер больше не активен (завершен или очищен)
    EXPECT_EQ(active, 0);
}

TEST_F(ClusterCleanupTest, ClusterNotExpiredWithUpdates) {
    // Создаем кластер с постоянными обновлениями
    for (int i = 0; i < 10; ++i) {
        ScanReplies scan = create_scan(100 + i);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, 50 + i, 1234));
        tracker_->process_scan(scan);
    }
    
    // Кластер активен
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    // Пропускаем немного оборотов без обновлений
    for (int i = 0; i < 3; ++i) {
        ScanReplies empty_scan = create_scan(110 + i);
        tracker_->process_scan(empty_scan);
    }
    
    // Кластер все еще активен (не прошло достаточно оборотов для завершения)
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
}

TEST_F(ClusterCleanupTest, MaxActiveClustersLimit) {
    // Создаем много кластеров
    for (int cluster = 0; cluster < 10; ++cluster) {
        ScanReplies scan = create_scan(100 + cluster * 10);
        for (int reply = 0; reply < 3; ++reply) {
            scan.rbs_replies.push_back(
                create_rbs_reply(100 + cluster * 10 + reply, 
                                 50 + cluster * 10, 
                                 1234 + cluster)
            );
        }
        tracker_->process_scan(scan);
    }
    
    // Должно быть не более max_active_clusters
    const auto& active = tracker_->get_active_clusters();
    EXPECT_LE(active.size(), tracker_->get_max_active_clusters());
}

TEST_F(ClusterCleanupTest, ManualCleanup) {
    // Создаем кластер
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    tracker_->process_scan(scan1);
    
    EXPECT_EQ(tracker_->get_active_clusters().size(), 1);
    
    // Пропускаем обороты
    for (int i = 0; i < 15; ++i) {
        ScanReplies empty_scan = create_scan(101 + i);
        tracker_->process_scan(empty_scan);
    }
    
    // Кластер должен быть завершен
    auto completed = tracker_->get_completed_clusters();
    EXPECT_EQ(tracker_->get_active_clusters().size(), 0);
}

TEST_F(ClusterCleanupTest, Statistics) {
    // Создаем несколько кластеров
    for (int cluster = 0; cluster < 3; ++cluster) {
        ScanReplies scan = create_scan(100 + cluster * 20);
        for (int reply = 0; reply < 3; ++reply) {
            scan.rbs_replies.push_back(
                create_rbs_reply(100 + cluster * 20 + reply, 
                                 50 + cluster * 20, 
                                 1234 + cluster)
            );
        }
        tracker_->process_scan(scan);
    }
    
    // Проверяем статистику
    auto stats = tracker_->get_stats();
    EXPECT_GT(stats.active_count, 0);
}
