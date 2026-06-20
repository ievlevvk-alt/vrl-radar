// tests/test_integration.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster.hpp"
#include "vrl/radar/core/cluster_pool.hpp"

using namespace vrl::radar;

class IntegrationTest : public ::testing::Test {
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
// ТЕСТ 1: Полный цикл жизни кластера
// ============================================================

TEST_F(IntegrationTest, FullClusterLifecycle) {
    // 1. Добавляем точки в буфер
    size_t idx1 = add_point(100, 50, true);   // RBS
    size_t idx2 = add_point(102, 52, true);   // RBS
    size_t idx3 = add_point(105, 55, false);  // UVD
    
    // 2. Создаем кластер через пул
    Cluster& cluster = ClusterPool::instance().create_cluster();
    EXPECT_EQ(ClusterPool::instance().size(), 1);
    EXPECT_TRUE(cluster.is_empty());
    EXPECT_FALSE(cluster.is_closed());
    
    // 3. Добавляем точки в кластер
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    cluster.add_point(idx3);
    
    EXPECT_EQ(cluster.size(), 3);
    EXPECT_FALSE(cluster.is_empty());
    
    // 4. Проверяем статистику
    EXPECT_EQ(cluster.get_min_range(), 50);
    EXPECT_EQ(cluster.get_max_range(), 55);
    EXPECT_EQ(cluster.get_azimuth_span(), 5);  // 105 - 100 = 5
    EXPECT_TRUE(cluster.has_rbs());
    EXPECT_TRUE(cluster.has_uvd());
    EXPECT_TRUE(cluster.is_mixed());
    
    // 5. Проверяем порядок точек
    const auto& indices = cluster.get_indices();
    ASSERT_EQ(indices.size(), 3);
    EXPECT_EQ(indices[0], idx1);
    EXPECT_EQ(indices[1], idx2);
    EXPECT_EQ(indices[2], idx3);
    
    // 6. Закрываем кластер
    cluster.close();
    EXPECT_TRUE(cluster.is_closed());
    
    // 7. Проверяем, что кластер в списке закрытых
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 1);
    EXPECT_EQ(closed[0], &cluster);
}

// ============================================================
// ТЕСТ 2: Несколько кластеров
// ============================================================

TEST_F(IntegrationTest, MultipleClusters) {
    // Создаем точки
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(102, 52);
    size_t idx3 = add_point(200, 60);
    size_t idx4 = add_point(202, 62);
    
    // Кластер 1 (RBS)
    Cluster& c1 = ClusterPool::instance().create_cluster();
    c1.add_point(idx1);
    c1.add_point(idx2);
    c1.close();
    
    // Кластер 2 (UVD)
    Cluster& c2 = ClusterPool::instance().create_cluster();
    c2.add_point(idx3);
    c2.add_point(idx4);
    c2.close();
    
    EXPECT_EQ(ClusterPool::instance().size(), 2);
    
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 2);
    
    EXPECT_EQ(closed[0]->get_min_range(), 50);
    EXPECT_EQ(closed[1]->get_min_range(), 60);
}

// ============================================================
// ТЕСТ 3: Удаление точек из кластера
// ============================================================

TEST_F(IntegrationTest, RemovePointsFromCluster) {
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(102, 52);
    size_t idx3 = add_point(104, 54);
    
    Cluster& cluster = ClusterPool::instance().create_cluster();
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    cluster.add_point(idx3);
    
    EXPECT_EQ(cluster.size(), 3);
    EXPECT_EQ(cluster.get_min_range(), 50);
    EXPECT_EQ(cluster.get_max_range(), 54);
    EXPECT_EQ(cluster.get_azimuth_span(), 4);
    
    // Удаляем точку по позиции 1 (idx2)
    cluster.remove_points({1});
    
    EXPECT_EQ(cluster.size(), 2);
    EXPECT_EQ(cluster.get_min_range(), 50);
    EXPECT_EQ(cluster.get_max_range(), 54);
    EXPECT_EQ(cluster.get_azimuth_span(), 4);
    
    const auto& indices = cluster.get_indices();
    ASSERT_EQ(indices.size(), 2);
    EXPECT_EQ(indices[0], idx1);
    EXPECT_EQ(indices[1], idx3);
}

// ============================================================
// ТЕСТ 4: Переход через Север
// ============================================================

TEST_F(IntegrationTest, NorthTransition) {
    size_t idx1 = add_point(4094, 50);
    size_t idx2 = add_point(0, 55);
    
    Cluster& cluster = ClusterPool::instance().create_cluster();
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    EXPECT_EQ(cluster.get_azimuth_span(), 2);
}

// ============================================================
// ТЕСТ 5: Активные и закрытые кластеры в пуле
// ============================================================

TEST_F(IntegrationTest, ActiveAndClosedInPool) {
    // Создаем 3 кластера
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    Cluster& c3 = ClusterPool::instance().create_cluster();
    
    // Добавляем точки
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(200, 60);
    size_t idx3 = add_point(300, 70);
    
    c1.add_point(idx1);
    c2.add_point(idx2);
    c3.add_point(idx3);
    
    // Закрываем c1 и c3
    c1.close();
    c3.close();
    
    // Проверяем активные
    auto active = ClusterPool::instance().get_active_clusters();
    ASSERT_EQ(active.size(), 1);
    EXPECT_EQ(active[0], &c2);
    EXPECT_FALSE(active[0]->is_closed());
    
    // Проверяем закрытые
    auto closed = ClusterPool::instance().get_closed_clusters();
    ASSERT_EQ(closed.size(), 2);
    EXPECT_TRUE(closed[0]->is_closed());
    EXPECT_TRUE(closed[1]->is_closed());
}

// ============================================================
// ТЕСТ 6: Очистка пула с кластерами
// ============================================================

TEST_F(IntegrationTest, ClearPoolWithClusters) {
    Cluster& c1 = ClusterPool::instance().create_cluster();
    Cluster& c2 = ClusterPool::instance().create_cluster();
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(200, 60);
    
    c1.add_point(idx1);
    c2.add_point(idx2);
    
    EXPECT_EQ(ClusterPool::instance().size(), 2);
    
    ClusterPool::instance().clear();
    
    EXPECT_EQ(ClusterPool::instance().size(), 0);
    EXPECT_TRUE(ClusterPool::instance().is_empty());
}
