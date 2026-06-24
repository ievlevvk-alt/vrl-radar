// tests/test_cluster_pool_full.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"

using namespace vrl::radar;

class ClusterPoolFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(1000);
        ClusterPool::instance().init(100);
    }
    
    void TearDown() override {
        ClusterPool::instance().clear();
        PointBuffer::instance().init(1000);
    }
    
    void add_point_to_buffer(uint16_t azimuth, uint16_t range) {
        StoredPoint point;
        point.azimuth = azimuth;
        point.range = range;
        point.is_rbs = true;
        PointBuffer::instance().add_point(point);
    }
    
    uint64_t create_cluster_with_points(const std::vector<std::pair<uint16_t, uint16_t>>& points) {
        auto& pool = ClusterPool::instance();
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        
        for (const auto& [az, range] : points) {
            add_point_to_buffer(az, range);
            cluster->add_point(PointBuffer::instance().size() - 1);
        }
        
        return id;
    }
};

TEST_F(ClusterPoolFullTest, Init) {
    auto& pool = ClusterPool::instance();
    EXPECT_TRUE(pool.is_initialized());
    EXPECT_EQ(pool.size(), 0);
}

TEST_F(ClusterPoolFullTest, CreateCluster) {
    auto& pool = ClusterPool::instance();
    uint64_t id = pool.create_cluster();
    
    EXPECT_GT(id, 0);
    EXPECT_TRUE(pool.exists(id));
}

TEST_F(ClusterPoolFullTest, CreateMultipleClusters) {
    auto& pool = ClusterPool::instance();
    std::vector<uint64_t> ids;
    
    for (int i = 0; i < 10; ++i) {
        ids.push_back(pool.create_cluster());
    }
    
    for (uint64_t id : ids) {
        EXPECT_TRUE(pool.exists(id));
    }
}

TEST_F(ClusterPoolFullTest, GetCluster) {
    auto& pool = ClusterPool::instance();
    uint64_t id = pool.create_cluster();
    
    Cluster* cluster = pool.get_cluster(id);
    ASSERT_NE(cluster, nullptr);
    EXPECT_EQ(cluster->get_id(), id);
}

TEST_F(ClusterPoolFullTest, GetClusterInvalid) {
    auto& pool = ClusterPool::instance();
    Cluster* cluster = pool.get_cluster(999);
    EXPECT_EQ(cluster, nullptr);
}

TEST_F(ClusterPoolFullTest, CloseCluster) {
    auto& pool = ClusterPool::instance();
    uint64_t id = create_cluster_with_points({{512, 100}, {513, 101}});
    
    pool.close_cluster(id, 0);
    
    Cluster* cluster = pool.get_cluster(id);
    EXPECT_TRUE(cluster->is_closed());
}

TEST_F(ClusterPoolFullTest, AddToActive) {
    auto& pool = ClusterPool::instance();
    uint64_t id = pool.create_cluster();
    
    pool.add_to_active(id);
    
    auto active = pool.get_active_clusters();
    bool found = false;
    for (auto* cluster : active) {
        if (cluster->get_id() == id) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}

TEST_F(ClusterPoolFullTest, AddToDelayed) {
    auto& pool = ClusterPool::instance();
    uint64_t id = create_cluster_with_points({{512, 100}, {513, 101}});
    
    pool.add_to_delayed(id, 0);
    
    EXPECT_TRUE(pool.has_delayed_clusters());
}

TEST_F(ClusterPoolFullTest, TakeDelayedClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_points({{512 + i, 100}, {513 + i, 101}});
        pool.add_to_delayed(id, 0);
    }
    
    auto delayed = pool.take_delayed_clusters(0);
    EXPECT_EQ(delayed.size(), 5);
    EXPECT_FALSE(pool.has_delayed_clusters());
}

TEST_F(ClusterPoolFullTest, AddToWide) {
    auto& pool = ClusterPool::instance();
    uint64_t id = pool.create_cluster();
    
    pool.add_to_wide(id);
    
    EXPECT_TRUE(pool.has_wide_clusters());
}

TEST_F(ClusterPoolFullTest, TakeWideClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = pool.create_cluster();
        pool.add_to_wide(id);
    }
    
    auto wide = pool.take_wide_clusters();
    EXPECT_EQ(wide.size(), 5);
    EXPECT_FALSE(pool.has_wide_clusters());
}

TEST_F(ClusterPoolFullTest, TakeClosedClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_points({{512 + i, 100}, {513 + i, 101}});
        pool.close_cluster(id, 0);
    }
    
    auto closed = pool.take_closed_clusters(0);
    EXPECT_EQ(closed.size(), 5);
    EXPECT_FALSE(pool.has_closed_clusters(0));
}

TEST_F(ClusterPoolFullTest, GetClosedClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_points({{512 + i, 100}, {513 + i, 101}});
        pool.close_cluster(id, i % 32);
    }
    
    auto closed = pool.get_closed_clusters();
    EXPECT_EQ(closed.size(), 5);
}

TEST_F(ClusterPoolFullTest, MergeClusters) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id1 = create_cluster_with_points({{512, 100}, {513, 101}});
    uint64_t id2 = create_cluster_with_points({{514, 102}, {515, 103}});
    
    pool.merge_clusters(id1, id2);
    
    Cluster* cluster = pool.get_cluster(id1);
    EXPECT_EQ(cluster->size(), 4);
    EXPECT_FALSE(pool.exists(id2));
}

TEST_F(ClusterPoolFullTest, CleanupDelayed) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_points({{512 + i, 100}, {513 + i, 101}});
        pool.add_to_delayed(id, 0);
    }
    
    size_t cleaned = pool.cleanup_delayed_clusters(10, 0.0);
    EXPECT_EQ(cleaned, 5);
}

TEST_F(ClusterPoolFullTest, GetStats) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id1 = create_cluster_with_points({{512, 100}, {513, 101}});
    uint64_t id2 = create_cluster_with_points({{514, 102}, {515, 103}});
    
    pool.close_cluster(id1, 0);
    pool.add_to_delayed(id2, 0);
    
    auto stats = pool.get_stats();
    EXPECT_GT(stats.total_clusters, 0);
}

TEST_F(ClusterPoolFullTest, Clear) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 10; ++i) {
        uint64_t id = create_cluster_with_points({{512 + i, 100}, {513 + i, 101}});
        pool.close_cluster(id, i % 32);
    }
    
    pool.clear();
    
    EXPECT_TRUE(pool.is_empty());
    EXPECT_EQ(pool.size(), 0);
}

TEST_F(ClusterPoolFullTest, CountMethods) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id1 = create_cluster_with_points({{512, 100}, {513, 101}});
    uint64_t id2 = create_cluster_with_points({{514, 102}, {515, 103}});
    
    pool.close_cluster(id1, 0);
    pool.add_to_delayed(id2, 0);
    
    EXPECT_EQ(pool.count_active_clusters(), 1);
    EXPECT_EQ(pool.count_closed_clusters(), 1);
    EXPECT_EQ(pool.count_delayed_clusters(), 1);
}
