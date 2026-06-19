// tests/test_range_grouper.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/range_grouper.h"
#include "vrl/radar/processing/cluster.h"
#include <memory>

using namespace vrl::radar;

class RangeGrouperTest : public ::testing::Test {
protected:
    void SetUp() override {
        grouper_ = std::make_unique<RangeGrouper>(5);  // 5 bins tolerance
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
    
    std::unique_ptr<RangeGrouper> grouper_;
};

TEST_F(RangeGrouperTest, EmptyCluster) {
    TargetCluster cluster;
    auto groups = grouper_->group(cluster);
    EXPECT_TRUE(groups.empty());
}

TEST_F(RangeGrouperTest, SingleRBSReply) {
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 1);
    EXPECT_EQ(groups[0].total_replies(), 1);
    EXPECT_TRUE(groups[0].has_rbs());
    EXPECT_FALSE(groups[0].has_uvd());
    EXPECT_EQ(groups[0].nominal_range, 50);
}

TEST_F(RangeGrouperTest, SingleUVDReply) {
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.uvd_replies.push_back(create_uvd_reply(100, 50, 12345));
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 1);
    EXPECT_EQ(groups[0].total_replies(), 1);
    EXPECT_FALSE(groups[0].has_rbs());
    EXPECT_TRUE(groups[0].has_uvd());
    EXPECT_EQ(groups[0].nominal_range, 50);
}

TEST_F(RangeGrouperTest, MultipleRBSRepliesSameRange) {
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(101, 52, 5678));
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 1);
    EXPECT_EQ(groups[0].total_replies(), 2);
    EXPECT_TRUE(groups[0].has_rbs());
    EXPECT_EQ(groups[0].nominal_range, 50);
}

TEST_F(RangeGrouperTest, MultipleRBSRepliesDifferentRanges) {
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(101, 100, 5678));
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 2);  // Должно быть 2 группы, т.к. разница 50 > tolerance 5
    EXPECT_EQ(groups[0].total_replies(), 1);
    EXPECT_EQ(groups[1].total_replies(), 1);
}

TEST_F(RangeGrouperTest, RBSAndUVDInSameGroup) {
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.uvd_replies.push_back(create_uvd_reply(101, 52, 12345));
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 2);  // RBS и UVD группируются отдельно
    EXPECT_EQ(groups[0].total_replies(), 1);
    EXPECT_EQ(groups[1].total_replies(), 1);
    EXPECT_TRUE(groups[0].has_rbs());
    EXPECT_TRUE(groups[1].has_uvd());
}

TEST_F(RangeGrouperTest, MultipleScans) {
    TargetCluster cluster;
    
    for (int i = 0; i < 5; ++i) {
        ScanReplies scan = create_scan(100 + i);
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, 50 + i, 1234));
        cluster.add_scan(scan, i);
    }
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 1);
    EXPECT_EQ(groups[0].total_replies(), 5);
    EXPECT_EQ(groups[0].nominal_range, 50);
}

TEST_F(RangeGrouperTest, LargeDataSet) {
    TargetCluster cluster;
    
    // Создаем 100 ответов с разными дальностями
    for (int i = 0; i < 100; ++i) {
        ScanReplies scan = create_scan(100 + i);
        uint16_t range = 50 + (i % 20);  // Дальности от 50 до 69
        scan.rbs_replies.push_back(create_rbs_reply(100 + i, range, 1234));
        cluster.add_scan(scan, i);
    }
    
    auto groups = grouper_->group(cluster);
    
    // Должно быть ~4 группы (50-54, 55-59, 60-64, 65-69)
    EXPECT_GT(groups.size(), 1);
    EXPECT_LT(groups.size(), 10);
    
    // Проверяем, что все ответы распределены по группам
    size_t total = 0;
    for (const auto& group : groups) {
        total += group.total_replies();
    }
    EXPECT_EQ(total, 100);
}

TEST_F(RangeGrouperTest, SetTolerance) {
    grouper_->set_tolerance(10);
    EXPECT_EQ(grouper_->get_tolerance(), 10);
    
    TargetCluster cluster;
    ScanReplies scan = create_scan(100);
    scan.rbs_replies.push_back(create_rbs_reply(100, 50, 1234));
    scan.rbs_replies.push_back(create_rbs_reply(101, 58, 5678));  // Разница 8 < 10
    cluster.add_scan(scan, 0);
    
    auto groups = grouper_->group(cluster);
    EXPECT_EQ(groups.size(), 1);  // Одна группа, т.к. разница 8 < tolerance 10
    EXPECT_EQ(groups[0].total_replies(), 2);
}
