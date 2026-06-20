// tests/test_cluster.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster.hpp"
#include "vrl/radar/core/point_buffer.hpp"

using namespace vrl::radar;

class ClusterTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(100);
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
// ТЕСТ 1: Добавление точек
// ============================================================

TEST_F(ClusterTest, AddPoints) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(102, 52);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    EXPECT_EQ(cluster.size(), 2);
    EXPECT_FALSE(cluster.is_empty());
    EXPECT_EQ(cluster.get_point_index(0), idx1);
    EXPECT_EQ(cluster.get_point_index(1), idx2);
}

// ============================================================
// ТЕСТ 2: Статистика кластера
// ============================================================

TEST_F(ClusterTest, Statistics) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(105, 55);
    size_t idx3 = add_point(110, 60);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    cluster.add_point(idx3);
    
    EXPECT_EQ(cluster.get_min_range(), 50);
    EXPECT_EQ(cluster.get_max_range(), 60);
    EXPECT_EQ(cluster.get_azimuth_span(), 10);  // 110 - 100 = 10
    EXPECT_TRUE(cluster.has_rbs());
    EXPECT_FALSE(cluster.has_uvd());
}

// ============================================================
// ТЕСТ 3: Переход через Север
// ============================================================

TEST_F(ClusterTest, NorthTransition) {
    Cluster cluster;
    
    size_t idx1 = add_point(4094, 50);
    size_t idx2 = add_point(0, 55);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    // Отладочный вывод
    const auto& indices = cluster.get_indices();
    std::cout << "=== Debug NorthTransition ===" << std::endl;
    std::cout << "Cluster size: " << cluster.size() << std::endl;
    for (size_t i = 0; i < indices.size(); ++i) {
        const auto& point = PointBuffer::instance().get_point(indices[i]);
        std::cout << "  Point " << i << ": azimuth=" << point.azimuth 
                  << ", range=" << point.range << std::endl;
    }
    std::cout << "azimuth_span = " << cluster.get_azimuth_span() << std::endl;
    std::cout << "Expected: " << ((4096 - 4094) + 0) << std::endl;
    
    EXPECT_EQ(cluster.get_azimuth_span(), (4096 - 4094) + 0);
}



// ============================================================
// ТЕСТ 4: RBS и UVD
// ============================================================

TEST_F(ClusterTest, RBSAndUVD) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50, true);   // RBS
    size_t idx2 = add_point(102, 52, false);  // UVD
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    EXPECT_TRUE(cluster.has_rbs());
    EXPECT_TRUE(cluster.has_uvd());
    EXPECT_TRUE(cluster.is_mixed());
}

// ============================================================
// ТЕСТ 5: Удаление точек
// ============================================================

TEST_F(ClusterTest, RemovePoints) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(105, 55);
    size_t idx3 = add_point(110, 60);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    cluster.add_point(idx3);
    
    EXPECT_EQ(cluster.size(), 3);
    
    // Удаляем точки на позициях 0 и 2
    cluster.remove_points({0, 2});
    
    EXPECT_EQ(cluster.size(), 1);
    EXPECT_EQ(cluster.get_point_index(0), idx2);
    EXPECT_EQ(cluster.get_min_range(), 55);
    EXPECT_EQ(cluster.get_max_range(), 55);
}

// ============================================================
// ТЕСТ 6: Закрытие кластера
// ============================================================

TEST_F(ClusterTest, CloseCluster) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    cluster.add_point(idx1);
    
    EXPECT_FALSE(cluster.is_closed());
    
    cluster.close();
    
    EXPECT_TRUE(cluster.is_closed());
    
    // Попытка добавить точку в закрытый кластер
    size_t idx2 = add_point(102, 52);
    cluster.add_point(idx2);
    
    EXPECT_EQ(cluster.size(), 1);  // Точка не добавилась
}

// ============================================================
// ТЕСТ 7: Очистка кластера
// ============================================================

TEST_F(ClusterTest, ClearCluster) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(102, 52);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    
    EXPECT_EQ(cluster.size(), 2);
    
    cluster.clear();
    
    EXPECT_TRUE(cluster.is_empty());
    EXPECT_EQ(cluster.size(), 0);
    EXPECT_FALSE(cluster.is_closed());  // После очистки кластер снова открыт
}

// ============================================================
// ТЕСТ 8: Порядок добавления
// ============================================================

TEST_F(ClusterTest, OrderPreserved) {
    Cluster cluster;
    
    size_t idx1 = add_point(100, 50);
    size_t idx2 = add_point(105, 55);
    size_t idx3 = add_point(110, 60);
    
    cluster.add_point(idx1);
    cluster.add_point(idx2);
    cluster.add_point(idx3);
    
    const auto& indices = cluster.get_indices();
    ASSERT_EQ(indices.size(), 3);
    EXPECT_EQ(indices[0], idx1);
    EXPECT_EQ(indices[1], idx2);
    EXPECT_EQ(indices[2], idx3);
}

// tests/test_cluster.cpp
// Замените тест NorthTransition на этот:

