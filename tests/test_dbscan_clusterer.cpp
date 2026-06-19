// tests/test_dbscan_clusterer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/processing/cluster.h"
#include <memory>
#include <cmath>

using namespace vrl::radar;

class DBSCANClustererTest : public ::testing::Test {
protected:
    void SetUp() override {
        // min_pts = 1 для упрощения тестов (любая точка с соседом - кластер)
        clusterer_ = std::make_unique<DBSCANClusterer>(150.0, 1.0, 1, 30.0);
    }
    
    RBSReply create_rbs_reply(uint16_t azimuth, uint16_t range, double x, double y, uint16_t code = 1234) {
        RBSReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.x = x;
        reply.y = y;
        reply.code12 = code;
        reply.is_valid = true;
        reply.ether_amplitudes[0] = 100;
        reply.ether_amplitudes[14] = 100;
        return reply;
    }
    
    UVDReply create_uvd_reply(uint16_t azimuth, uint16_t range, double x, double y, uint32_t data = 12345) {
        UVDReply reply;
        reply.azimuth = azimuth;
        reply.range = range;
        reply.x = x;
        reply.y = y;
        reply.data20 = data;
        reply.is_valid = true;
        return reply;
    }
    
    ScanReplies create_scan(uint16_t azimuth, uint32_t timestamp = 0) {
        return ScanReplies(azimuth, timestamp);
    }
    
    std::unique_ptr<DBSCANClusterer> clusterer_;
};

TEST_F(DBSCANClustererTest, DefaultParameters) {
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_range(), 150.0);
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_azimuth_deg(), 1.0);
    EXPECT_EQ(clusterer_->get_min_pts(), 1);
    EXPECT_DOUBLE_EQ(clusterer_->get_range_bin(), 30.0);
    EXPECT_EQ(clusterer_->get_name(), "DBSCANClusterer");
}

TEST_F(DBSCANClustererTest, EmptyScan) {
    ScanReplies scan = create_scan(100);
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 0);
    auto completed = clusterer_->get_completed_clusters();
    EXPECT_TRUE(completed.empty());
}

TEST_F(DBSCANClustererTest, SinglePointNoCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    clusterer_->process_scan(scan);
    
    // Одна точка не образует кластер (нет соседей)
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 0);
}

TEST_F(DBSCANClustererTest, TwoPointsFormCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan.rbs_replies.push_back(create_rbs_reply(101, 52, 120.0, 110.0));
    clusterer_->process_scan(scan);
    
    // Должен быть 1 кластер
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 1);
}

TEST_F(DBSCANClustererTest, PointsTooFarNoCluster) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan.rbs_replies.push_back(create_rbs_reply(110, 80, 400.0, 400.0));
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 0);
}

TEST_F(DBSCANClustererTest, MultipleClusters) {
    ScanReplies scan = create_scan(100);
    
    // Кластер 1
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan.rbs_replies.push_back(create_rbs_reply(101, 52, 120.0, 110.0));
    
    // Кластер 2 (далеко от первого)
    scan.rbs_replies.push_back(create_rbs_reply(103, 70, 500.0, 500.0));
    scan.rbs_replies.push_back(create_rbs_reply(104, 72, 520.0, 510.0));
    
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 2);
}

TEST_F(DBSCANClustererTest, RBSAndUVDInSameCluster) {
    ScanReplies scan = create_scan(100);
    
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan.uvd_replies.push_back(create_uvd_reply(101, 52, 120.0, 110.0));
    
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 1);
}

TEST_F(DBSCANClustererTest, SetParameters) {
    clusterer_->set_eps_range(200.0);
    clusterer_->set_eps_azimuth_deg(0.5);
    clusterer_->set_min_pts(5);
    clusterer_->set_range_bin(60.0);
    
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_range(), 200.0);
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_azimuth_deg(), 0.5);
    EXPECT_EQ(clusterer_->get_min_pts(), 5);
    EXPECT_DOUBLE_EQ(clusterer_->get_range_bin(), 60.0);
}

TEST_F(DBSCANClustererTest, SetParamMethods) {
    clusterer_->set_param("eps_range", 180.0);
    clusterer_->set_param("eps_azimuth_deg", 2.0);
    clusterer_->set_param("min_pts", 4);
    clusterer_->set_param("range_bin", 45.0);
    
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_range(), 180.0);
    EXPECT_DOUBLE_EQ(clusterer_->get_eps_azimuth_deg(), 2.0);
    EXPECT_EQ(clusterer_->get_min_pts(), 4);
    EXPECT_DOUBLE_EQ(clusterer_->get_range_bin(), 45.0);
}

TEST_F(DBSCANClustererTest, Reset) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan.rbs_replies.push_back(create_rbs_reply(101, 52, 120.0, 110.0));
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 1);
    
    clusterer_->reset();
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 0);
}

TEST_F(DBSCANClustererTest, Clone) {
    clusterer_->set_eps_range(200.0);
    clusterer_->set_eps_azimuth_deg(0.5);
    clusterer_->set_min_pts(3);
    
    auto clone = clusterer_->clone();
    EXPECT_EQ(clone->get_name(), "DBSCANClusterer");
    
    auto* dbscan_clone = dynamic_cast<DBSCANClusterer*>(clone.get());
    ASSERT_NE(dbscan_clone, nullptr);
    EXPECT_DOUBLE_EQ(dbscan_clone->get_eps_range(), 200.0);
    EXPECT_DOUBLE_EQ(dbscan_clone->get_eps_azimuth_deg(), 0.5);
    EXPECT_EQ(dbscan_clone->get_min_pts(), 3);
}

TEST_F(DBSCANClustererTest, MahalanobisDistance) {
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 0.0));
    scan.rbs_replies.push_back(create_rbs_reply(100, 55, 130.0, 0.0));
    clusterer_->process_scan(scan);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 1);
}

TEST_F(DBSCANClustererTest, ClusterCompletesAfterGap) {
    ScanReplies scan1 = create_scan(100);
    scan1.rbs_replies.push_back(create_rbs_reply(100, 50, 100.0, 100.0));
    scan1.rbs_replies.push_back(create_rbs_reply(101, 52, 120.0, 110.0));
    clusterer_->process_scan(scan1);
    
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 1);
    
    for (int i = 0; i < 15; ++i) {
        ScanReplies empty_scan = create_scan(102 + i);
        clusterer_->process_scan(empty_scan);
    }
    
    auto completed = clusterer_->get_completed_clusters();
    EXPECT_FALSE(completed.empty());
    EXPECT_EQ(clusterer_->get_active_clusters().size(), 0);
}
