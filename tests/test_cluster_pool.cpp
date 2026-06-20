// tests/test_cluster_pool.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"

using namespace vrl::radar;

class ClusterPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(100);
        ClusterPool::instance().clear();
    }
    
    size_t add_point(uint16_t azimuth, uint16_t range, bool is_rbs = true) {
        StoredPoint p;
        p.azimuth = azimuth;
        p.range = range;
        p.is_rbs = is_rbs;
        p.amplitude = 100;
        return PointBuffer::instance().add_point(p);
    }
};

// ============================================================
// ТЕСТ 1: Создание кластера
// ============================================================

TEST_F(ClusterPoolTest, CreateCluster) {
    Cluster& cluster = ClusterPool::instance().create_cluster();
    
    EXPECT_EQ(ClusterPool::instance().size(), 1);
    EXPECT_TRUE(cluster.is_empty());
    EXPECT_FALSE(cluster.is_closed());
}

// ============================================================
// ТЕСТ 2: Множественные кластеры
// ============================================================

TEST_F(ClusterPoolTest, MultipleClusters) {
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    Cluster& c3 = ClusterPool::instance().create_cluster();
    
    EXPECT_EQ(ClusterPool::instance().size(), 3);
}

// ============================================================
// ТЕСТ 3: Добавление точек в кластер через пул
// ============================================================

TEST_F(ClusterPoolTest, AddPointsToCluster) {
    Cluster& cluster = ClusterPool::instance().create_cluster();
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(102, 52);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    EXPECT_EQ(cluster.size(), 2);
}

// ============================================================
// ТЕСТ 4: Получение кластера по индексу
// ============================================================

TEST_F(ClusterPoolTest, GetCluster) {
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(200, 60);
    
    c1.add_point(idx1);
    c2.add_point(idx2);
    
    const Cluster& retrieved1 = ClusterPool::instance().get_cluster(0);
    const Cluster& retrieved2 = ClusterPool::instance().get_cluster(1);
    
    EXPECT_EQ(retrieved1.size(), 1);
    EXPECT_EQ(retrieved2.size(), 1);
}

// ============================================================
// ТЕСТ 5: Активные и закрытые кластеры
// ============================================================

TEST_F(ClusterPoolTest, ActiveAndClosedClusters) {
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    Cluster& c3 = ClusterPool::instance().create_cluster();
    
    c1.close();
    c3.close();
    
    auto active = ClusterPool::instance().get_active_clusters();
    auto closed = ClusterPool::instance().get_closed_clusters();
    
    EXPECT_EQ(active.size(), 1);   // только c2
    EXPECT_EQ(closed.size(), 2);   // c1 и c3
}

// ============================================================
// ТЕСТ 6: Удаление кластера
// ============================================================

TEST_F(ClusterPoolTest, RemoveCluster) {
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    
    EXPECT_EQ(ClusterPool::instance().size(), 2);
    
    ClusterPool::instance().remove_cluster(0);
    
    EXPECT_EQ(ClusterPool::instance().size(), 1);
    
    const Cluster& remaining = ClusterPool::instance().get_cluster(0);
    // Проверяем, что остался второй кластер (c2)
}

// ============================================================
// ТЕСТ 7: Очистка пула
// ============================================================

TEST_F(ClusterPoolTest, ClearPool) {
    ClusterPool::instance().create_cluster();
    ClusterPool::instance().create_cluster();
    ClusterPool::instance().create_cluster();
    
    EXPECT_EQ(ClusterPool::instance().size(), 3);
    
    ClusterPool::instance().clear();
    
    EXPECT_EQ(ClusterPool::instance().size(), 0);
    EXPECT_TRUE(ClusterPool::instance().is_empty());
}
