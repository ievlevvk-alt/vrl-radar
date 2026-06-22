// tests/test_cluster_pool.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include <iostream>

using namespace vrl::radar;

class ClusterPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(1000);
        ClusterPool::instance().clear();
    }
    
    void TearDown() override {
        ClusterPool::instance().clear();
    }
    
    size_t count_active_clusters() const {
        return ClusterPool::instance().count_active_clusters();
    }
    
    size_t count_closed_clusters() const {
        return ClusterPool::instance().count_closed_clusters();
    }
    
    size_t count_total_clusters() const {
        return ClusterPool::instance().size();
    }
};

TEST_F(ClusterPoolTest, CreateCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    
    EXPECT_NE(id, 0);
    ASSERT_NE(cluster, nullptr);
    EXPECT_TRUE(cluster->is_empty());
    EXPECT_FALSE(cluster->is_closed());
    EXPECT_EQ(pool.size(), 1);
}

TEST_F(ClusterPoolTest, AddPointToCluster) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    StoredPoint point;
    point.azimuth = 100;
    point.range = 50;
    point.is_rbs = true;
    point.amplitude = 100;
    size_t idx = buffer.add_point(point);
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    ASSERT_NE(cluster, nullptr);
    cluster->add_point(idx);
    
    EXPECT_EQ(cluster->size(), 1);
    EXPECT_EQ(cluster->get_min_range(), 50);
    EXPECT_EQ(cluster->get_max_range(), 50);
    EXPECT_TRUE(cluster->has_rbs());
    EXPECT_FALSE(cluster->has_uvd());
}

TEST_F(ClusterPoolTest, CloseCluster) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    StoredPoint point;
    point.azimuth = 100;
    point.range = 50;
    point.is_rbs = true;
    point.amplitude = 100;
    size_t idx = buffer.add_point(point);
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    ASSERT_NE(cluster, nullptr);
    cluster->add_point(idx);
    
    int sector = 0;
    pool.close_cluster(id, sector);
    
    EXPECT_TRUE(cluster->is_closed());
    EXPECT_EQ(count_active_clusters(), 0);
    EXPECT_EQ(count_closed_clusters(), 1);
}

TEST_F(ClusterPoolTest, ActiveAndClosedClusters) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    std::vector<uint64_t> ids;
    for (int i = 0; i < 3; ++i) {
        StoredPoint point;
        point.azimuth = 100 + i * 10;
        point.range = 50 + i * 5;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        ids.push_back(id);
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
    }
    
    pool.close_cluster(ids[0], 0);
    
    EXPECT_EQ(count_active_clusters(), 2);
    EXPECT_EQ(count_closed_clusters(), 1);
    
    pool.close_cluster(ids[1], 0);
    pool.close_cluster(ids[2], 0);
    
    EXPECT_EQ(count_active_clusters(), 0);
    EXPECT_EQ(count_closed_clusters(), 3);
}

TEST_F(ClusterPoolTest, RemoveCluster) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    StoredPoint point;
    point.azimuth = 100;
    point.range = 50;
    point.is_rbs = true;
    point.amplitude = 100;
    size_t idx = buffer.add_point(point);
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    ASSERT_NE(cluster, nullptr);
    cluster->add_point(idx);
    
    EXPECT_EQ(pool.size(), 1);
    
    pool.remove_cluster(id);
    
    EXPECT_EQ(pool.size(), 0);
}

TEST_F(ClusterPoolTest, MarkAsWide) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    ASSERT_NE(cluster, nullptr);
    
    for (int i = 0; i < 10; ++i) {
        StoredPoint point;
        point.azimuth = 100 + i * 5;
        point.range = 50 + i * 2;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        cluster->add_point(idx);
    }
    
    EXPECT_EQ(pool.size(), 1);
    EXPECT_EQ(count_active_clusters(), 1);
    
    pool.mark_as_wide(id);
    
    // Широкий кластер всё ещё активен!
    EXPECT_EQ(count_active_clusters(), 1);
    EXPECT_EQ(pool.size(), 1);
}

TEST_F(ClusterPoolTest, TakeClosedClusters) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    for (int s = 0; s < 3; ++s) {
        StoredPoint point;
        point.azimuth = s * ClusterPool::SECTOR_SIZE + 10;
        point.range = 50 + s * 10;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        pool.close_cluster(id, s);
    }
    
    auto clusters = pool.take_closed_clusters(1);
    EXPECT_EQ(clusters.size(), 1);
    
    EXPECT_FALSE(pool.has_closed_clusters(1));
    EXPECT_TRUE(pool.has_closed_clusters(0));
    EXPECT_TRUE(pool.has_closed_clusters(2));
}

TEST_F(ClusterPoolTest, Clear) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    for (int i = 0; i < 5; ++i) {
        StoredPoint point;
        point.azimuth = 100 + i * 10;
        point.range = 50 + i * 5;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
    }
    
    EXPECT_EQ(pool.size(), 5);
    
    pool.clear();
    
    EXPECT_EQ(pool.size(), 0);
    EXPECT_EQ(count_active_clusters(), 0);
    EXPECT_EQ(count_closed_clusters(), 0);
}

TEST_F(ClusterPoolTest, GetCluster) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    StoredPoint point;
    point.azimuth = 100;
    point.range = 50;
    point.is_rbs = true;
    point.amplitude = 100;
    size_t idx = buffer.add_point(point);
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster1 = pool.get_cluster(id);
    ASSERT_NE(cluster1, nullptr);
    cluster1->add_point(idx);
    
    Cluster* cluster2 = pool.get_cluster(id);
    
    EXPECT_EQ(cluster1, cluster2);
}

TEST_F(ClusterPoolTest, GetStats) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    for (int s = 0; s < 3; ++s) {
        StoredPoint point;
        point.azimuth = s * ClusterPool::SECTOR_SIZE + 10;
        point.range = 50 + s * 10;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        
        if (s % 2 == 0) {
            pool.close_cluster(id, s);
        }
    }
    
    auto stats = pool.get_stats();
    EXPECT_EQ(stats.total_clusters, 3);
    EXPECT_EQ(stats.active_clusters, 1);
    EXPECT_EQ(stats.closed_clusters, 2);
}

TEST_F(ClusterPoolTest, MultipleSectors) {
    auto& pool = ClusterPool::instance();
    auto& buffer = PointBuffer::instance();
    
    // Создаём кластеры в разных секторах
    for (int s = 0; s < ClusterPool::NUM_SECTORS; ++s) {
        StoredPoint point;
        point.azimuth = s * ClusterPool::SECTOR_SIZE + 10;
        point.range = 50;
        point.is_rbs = true;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        pool.close_cluster(id, s);
    }
    
    // Проверяем, что все сектора имеют кластеры
    for (int s = 0; s < ClusterPool::NUM_SECTORS; ++s) {
        EXPECT_TRUE(pool.has_closed_clusters(s));
    }
    
    // Забираем кластеры из половины секторов
    for (int s = 0; s < ClusterPool::NUM_SECTORS / 2; ++s) {
        auto clusters = pool.take_closed_clusters(s);
        EXPECT_EQ(clusters.size(), 1);
        EXPECT_FALSE(pool.has_closed_clusters(s));
    }
    
    // Проверяем, что в другой половине ещё есть
    for (int s = ClusterPool::NUM_SECTORS / 2; s < ClusterPool::NUM_SECTORS; ++s) {
        EXPECT_TRUE(pool.has_closed_clusters(s));
    }
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
