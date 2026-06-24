// tests/test_cluster_pool_full.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/cluster_pool.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace vrl::radar;

class ClusterPoolFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(1000);
        ClusterPool::instance().init(65535);  // <-- ДОБАВЛЯЕМ
    }
    
    void TearDown() override {
        ClusterPool::instance().clear();
    }
    
    // Вспомогательные методы
    uint64_t create_cluster_with_point(uint16_t azimuth, uint16_t range, bool is_rbs = true) {
        auto& pool = ClusterPool::instance();
        auto& buffer = PointBuffer::instance();
        
        StoredPoint point;
        point.azimuth = azimuth;
        point.range = range;
        point.is_rbs = is_rbs;
        point.amplitude = 100;
        size_t idx = buffer.add_point(point);
        
        uint64_t id = pool.create_cluster();
        Cluster* cluster = pool.get_cluster(id);
        cluster->add_point(idx);
        
        return id;
    }
    
    size_t count_active() const {
        return ClusterPool::instance().count_active_clusters();
    }
    
    size_t count_closed() const {
        return ClusterPool::instance().count_closed_clusters();
    }
    
    size_t count_delayed() const {
        return ClusterPool::instance().count_delayed_clusters();
    }
    
    size_t count_wide() const {
        return ClusterPool::instance().count_wide_clusters();
    }
    
    size_t count_total() const {
        return ClusterPool::instance().size();
    }
    
    void print_stats(const std::string& label) {
        std::cout << "\n=== " << label << " ===" << std::endl;
        std::cout << "Total: " << count_total() << std::endl;
        std::cout << "Active: " << count_active() << std::endl;
        std::cout << "Closed: " << count_closed() << std::endl;
        std::cout << "Wide: " << count_wide() << std::endl;
        std::cout << "Delayed: " << count_delayed() << std::endl;
        std::cout << "==================\n" << std::endl;
    }
};

// ============================================================================
// 1. БАЗОВЫЕ ОПЕРАЦИИ
// ============================================================================

TEST_F(ClusterPoolFullTest, CreateCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = pool.create_cluster();
    Cluster* cluster = pool.get_cluster(id);
    
    EXPECT_NE(id, 0);
    ASSERT_NE(cluster, nullptr);
    EXPECT_TRUE(cluster->is_empty());
    EXPECT_FALSE(cluster->is_closed());
    EXPECT_EQ(count_total(), 0);   // <-- ИЗМЕНЕНО: пустой кластер не считается
    EXPECT_EQ(count_active(), 0);  // <-- ИЗМЕНЕНО: пустой кластер не в active_ids_
}

TEST_F(ClusterPoolFullTest, AddPointToCluster) {
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
    cluster->add_point(idx);
    
    EXPECT_EQ(cluster->size(), 1);
    EXPECT_EQ(cluster->get_min_range(), 50);
    EXPECT_EQ(cluster->get_max_range(), 50);
    EXPECT_TRUE(cluster->has_rbs());
    EXPECT_FALSE(cluster->has_uvd());
}

TEST_F(ClusterPoolFullTest, GetCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id1 = pool.create_cluster();
    uint64_t id2 = pool.create_cluster();
    
    Cluster* cluster1 = pool.get_cluster(id1);
    Cluster* cluster2 = pool.get_cluster(id2);
    
    ASSERT_NE(cluster1, nullptr);
    ASSERT_NE(cluster2, nullptr);
    EXPECT_NE(cluster1, cluster2);
    
    const Cluster* const_cluster = pool.get_cluster(id1);
    EXPECT_EQ(const_cluster, cluster1);
}

TEST_F(ClusterPoolFullTest, Clear) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        create_cluster_with_point(100 + i * 10, 50 + i * 5);
    }
    
    EXPECT_EQ(count_total(), 5);
    
    pool.clear();
    
    EXPECT_EQ(count_total(), 0);
    EXPECT_EQ(count_active(), 0);
    EXPECT_EQ(count_closed(), 0);
    EXPECT_EQ(count_wide(), 0);
    EXPECT_EQ(count_delayed(), 0);
}

// ============================================================================
// 2. СОСТОЯНИЯ КЛАСТЕРОВ
// ============================================================================

TEST_F(ClusterPoolFullTest, ActiveCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    
    EXPECT_TRUE(pool.exists(id));
    EXPECT_TRUE(pool.get_cluster(id)->is_empty() == false);
    EXPECT_FALSE(pool.get_cluster(id)->is_closed());
    EXPECT_EQ(count_active(), 1);
    EXPECT_EQ(count_closed(), 0);
}

TEST_F(ClusterPoolFullTest, WideCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    EXPECT_EQ(count_active(), 1);
    
    pool.add_to_wide(id);
    
    EXPECT_EQ(count_active(), 1);  // Широкий всё ещё активен!
    EXPECT_EQ(count_wide(), 1);
    EXPECT_EQ(count_closed(), 0);
}

TEST_F(ClusterPoolFullTest, ClosedCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    EXPECT_EQ(count_active(), 1);
    
    pool.close_cluster(id, 0);
    
    EXPECT_EQ(count_active(), 0);
    EXPECT_EQ(count_closed(), 1);
    EXPECT_TRUE(pool.get_cluster(id)->is_closed());
}

TEST_F(ClusterPoolFullTest, DelayedCluster) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.close_cluster(id, 0);
    EXPECT_EQ(count_closed(), 1);
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 1);
    EXPECT_EQ(ids[0], id);
    
    EXPECT_EQ(count_closed(), 0);
    EXPECT_EQ(count_delayed(), 1);
}

// ============================================================================
// 3. ПЕРЕХОДЫ МЕЖДУ СОСТОЯНИЯМИ
// ============================================================================

TEST_F(ClusterPoolFullTest, ActiveToWide) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    EXPECT_EQ(count_active(), 1);
    EXPECT_EQ(count_wide(), 0);
    
    pool.add_to_wide(id);
    
    EXPECT_EQ(count_active(), 1);
    EXPECT_EQ(count_wide(), 1);
}

TEST_F(ClusterPoolFullTest, ActiveToClosed) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    EXPECT_EQ(count_active(), 1);
    EXPECT_EQ(count_closed(), 0);
    
    pool.close_cluster(id, 0);
    
    EXPECT_EQ(count_active(), 0);
    EXPECT_EQ(count_closed(), 1);
}

TEST_F(ClusterPoolFullTest, ClosedToDelayed) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.close_cluster(id, 0);
    EXPECT_EQ(count_closed(), 1);
    EXPECT_EQ(count_delayed(), 0);
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 1);
    
    EXPECT_EQ(count_closed(), 0);
    EXPECT_EQ(count_delayed(), 1);
}

TEST_F(ClusterPoolFullTest, DelayedToRemoved) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.close_cluster(id, 0);
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(count_delayed(), 1);
    EXPECT_EQ(count_total(), 1);
    
    size_t cleaned = pool.cleanup_delayed_clusters(0, 0.0);
    EXPECT_EQ(cleaned, 1);
    EXPECT_EQ(count_delayed(), 0);
    EXPECT_EQ(count_total(), 0);
}

TEST_F(ClusterPoolFullTest, WideToProcessed) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.add_to_wide(id);
    EXPECT_TRUE(pool.has_wide_clusters());  // есть в списке
    EXPECT_EQ(count_wide(), 1);             // состояние WIDE
    
    auto ids = pool.take_wide_clusters();
    EXPECT_EQ(ids.size(), 1);
    
    // После take_wide_clusters():
    EXPECT_FALSE(pool.has_wide_clusters());  // список пуст
    EXPECT_EQ(count_wide(), 1);              // состояние осталось WIDE (история)
}

// В тесте WideIdsList:
TEST_F(ClusterPoolFullTest, WideIdsList) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.add_to_wide(id);
    }
    
    EXPECT_TRUE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 5);
    
    auto ids = pool.take_wide_clusters();
    EXPECT_EQ(ids.size(), 5);
    
    EXPECT_FALSE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 5);  // состояние осталось WIDE
}

// В тесте MultipleClustersDifferentStates:
TEST_F(ClusterPoolFullTest, MultipleClustersDifferentStates) {
    auto& pool = ClusterPool::instance();
    
    std::vector<uint64_t> ids;
    for (int i = 0; i < 6; ++i) {
        ids.push_back(create_cluster_with_point(100 + i * 10, 50 + i * 5));
    }
    
    pool.add_to_wide(ids[0]);
    pool.close_cluster(ids[1], 0);
    pool.close_cluster(ids[2], 0);
    pool.add_to_wide(ids[3]);
    
    EXPECT_TRUE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 2);
    EXPECT_EQ(count_active(), 4);
    
    auto wide = pool.take_wide_clusters();
    EXPECT_EQ(wide.size(), 2);
    
    EXPECT_FALSE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 2);  // состояние осталось WIDE
    EXPECT_EQ(count_active(), 4);
}

// В тесте TakeAllWideClusters:
TEST_F(ClusterPoolFullTest, TakeAllWideClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.add_to_wide(id);
    }
    
    EXPECT_TRUE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 5);
    
    auto ids = pool.take_wide_clusters();
    EXPECT_EQ(ids.size(), 5);
    
    EXPECT_FALSE(pool.has_wide_clusters());
    EXPECT_EQ(count_wide(), 5);
}





// ============================================================================
// 4. СПИСКИ И СЕКТОРА
// ============================================================================

TEST_F(ClusterPoolFullTest, ActiveIdsList) {
    auto& pool = ClusterPool::instance();
    
    std::vector<uint64_t> ids;
    for (int i = 0; i < 5; ++i) {
        ids.push_back(create_cluster_with_point(100 + i * 10, 50 + i * 5));
    }
    
    EXPECT_EQ(count_active(), 5);
    
    // Закрываем некоторые
    pool.close_cluster(ids[0], 0);
    pool.close_cluster(ids[2], 0);
    pool.close_cluster(ids[4], 0);
    
    EXPECT_EQ(count_active(), 2);
    EXPECT_EQ(count_closed(), 3);
}

TEST_F(ClusterPoolFullTest, ClosedBySector) {
    auto& pool = ClusterPool::instance();
    
    // Создаём кластеры в разных секторах
    for (int s = 0; s < 5; ++s) {
        uint64_t id = create_cluster_with_point(s * ClusterPool::SECTOR_SIZE + 10, 50);
        pool.close_cluster(id, s);
    }
    
    for (int s = 0; s < 5; ++s) {
        EXPECT_TRUE(pool.has_closed_clusters(s));
    }
    
    auto ids0 = pool.take_closed_clusters(0);
    EXPECT_EQ(ids0.size(), 1);
    EXPECT_FALSE(pool.has_closed_clusters(0));
}


TEST_F(ClusterPoolFullTest, DelayedIdsList) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.close_cluster(id, 0);
    }
    
    EXPECT_EQ(count_closed(), 5);
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 5);
    EXPECT_EQ(count_closed(), 0);
    EXPECT_EQ(count_delayed(), 5);
}

TEST_F(ClusterPoolFullTest, MultipleSectors) {
    auto& pool = ClusterPool::instance();
    
    for (int s = 0; s < ClusterPool::NUM_SECTORS; ++s) {
        uint64_t id = create_cluster_with_point(s * ClusterPool::SECTOR_SIZE + 10, 50);
        pool.close_cluster(id, s);
    }
    
    for (int s = 0; s < ClusterPool::NUM_SECTORS; ++s) {
        EXPECT_TRUE(pool.has_closed_clusters(s));
    }
    
    // Забираем из половины секторов
    for (int s = 0; s < ClusterPool::NUM_SECTORS / 2; ++s) {
        auto ids = pool.take_closed_clusters(s);
        EXPECT_EQ(ids.size(), 1);
        EXPECT_FALSE(pool.has_closed_clusters(s));
    }
    
    // В другой половине ещё есть
    for (int s = ClusterPool::NUM_SECTORS / 2; s < ClusterPool::NUM_SECTORS; ++s) {
        EXPECT_TRUE(pool.has_closed_clusters(s));
    }
}

TEST_F(ClusterPoolFullTest, SectorOverflow) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    
    // Сектор за пределами
    pool.close_cluster(id, 100);
    
    // Кластер должен быть закрыт, но не добавлен в сектор
    EXPECT_TRUE(pool.get_cluster(id)->is_closed());
    EXPECT_EQ(count_closed(), 0);  // Не добавлен в сектор
    EXPECT_EQ(count_active(), 0);
}


TEST_F(ClusterPoolFullTest, TakeAllClosedClusters) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.close_cluster(id, 0);
    }
    
    EXPECT_EQ(count_closed(), 5);
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 5);
    EXPECT_EQ(count_closed(), 0);
}


TEST_F(ClusterPoolFullTest, CleanupAllDelayed) {
    auto& pool = ClusterPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.close_cluster(id, 0);
    }
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 5);
    EXPECT_EQ(count_delayed(), 5);
    EXPECT_EQ(count_total(), 5);
    
    size_t cleaned = pool.cleanup_delayed_clusters(0, 0.0);
    EXPECT_EQ(cleaned, 5);
    EXPECT_EQ(count_delayed(), 0);
    EXPECT_EQ(count_total(), 0);
}

TEST_F(ClusterPoolFullTest, CleanupPartialDelayed) {
    auto& pool = ClusterPool::instance();
    
    // Создаём кластеры с разными временами задержки
    for (int i = 0; i < 5; ++i) {
        uint64_t id = create_cluster_with_point(100 + i * 10, 50 + i * 5);
        pool.close_cluster(id, 0);
    }
    
    auto ids = pool.take_closed_clusters(0);
    EXPECT_EQ(ids.size(), 5);
    EXPECT_EQ(count_delayed(), 5);
    
    // Очищаем с задержкой 1 (ничего не должно удалиться)
    size_t cleaned = pool.cleanup_delayed_clusters(1, 1.0);
    EXPECT_EQ(cleaned, 0);
    EXPECT_EQ(count_delayed(), 5);
    
    // Очищаем с задержкой 0 (всё должно удалиться)
    cleaned = pool.cleanup_delayed_clusters(1, 0.0);
    EXPECT_EQ(cleaned, 5);
    EXPECT_EQ(count_delayed(), 0);
    EXPECT_EQ(count_total(), 0);
}

// ============================================================================
// 6. ГРАНИЧНЫЕ СЛУЧАИ
// ============================================================================

TEST_F(ClusterPoolFullTest, GetNonExistentCluster) {
    auto& pool = ClusterPool::instance();
    
    Cluster* cluster = pool.get_cluster(99999);
    EXPECT_EQ(cluster, nullptr);
    
    const Cluster* const_cluster = pool.get_cluster(99999);
    EXPECT_EQ(const_cluster, nullptr);
}

TEST_F(ClusterPoolFullTest, CloseNonExistentCluster) {
    auto& pool = ClusterPool::instance();
    
    EXPECT_NO_THROW(pool.close_cluster(99999, 0));
}

TEST_F(ClusterPoolFullTest, MarkAsWideNonExistent) {
    auto& pool = ClusterPool::instance();
    
    EXPECT_NO_THROW(pool.add_to_wide(99999));
}

TEST_F(ClusterPoolFullTest, DoubleClose) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.close_cluster(id, 0);
    
    // Повторное закрытие не должно вызывать ошибку
    EXPECT_NO_THROW(pool.close_cluster(id, 0));
    EXPECT_TRUE(pool.get_cluster(id)->is_closed());
}

TEST_F(ClusterPoolFullTest, DoubleMarkAsWide) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    pool.add_to_wide(id);
    
    // Повторная пометка не должна вызывать ошибку
    EXPECT_NO_THROW(pool.add_to_wide(id));
    EXPECT_EQ(count_wide(), 1);
}

TEST_F(ClusterPoolFullTest, TakeClosedFromEmptySector) {
    auto& pool = ClusterPool::instance();
    
    auto ids = pool.take_closed_clusters(5);
    EXPECT_EQ(ids.size(), 0);
    EXPECT_FALSE(pool.has_closed_clusters(5));
}

TEST_F(ClusterPoolFullTest, CleanupEmptyDelayed) {
    auto& pool = ClusterPool::instance();
    
    size_t cleaned = pool.cleanup_delayed_clusters(0, 0.0);
    EXPECT_EQ(cleaned, 0);
}

TEST_F(ClusterPoolFullTest, Exists) {
    auto& pool = ClusterPool::instance();
    
    uint64_t id = create_cluster_with_point(100, 50);
    
    EXPECT_TRUE(pool.exists(id));
    EXPECT_FALSE(pool.exists(id + 1));
}

// ============================================================================
// 7. СТАТИСТИКА
// ============================================================================

TEST_F(ClusterPoolFullTest, StatsAfterOperations) {
    auto& pool = ClusterPool::instance();
    
    std::vector<uint64_t> ids;
    for (int i = 0; i < 10; ++i) {
        ids.push_back(create_cluster_with_point(100 + i * 10, 50 + i * 5));
    }
    
    // Разные состояния
    pool.add_to_wide(ids[0]);
    pool.add_to_wide(ids[1]);
    pool.close_cluster(ids[2], 0);
    pool.close_cluster(ids[3], 0);
    pool.close_cluster(ids[4], 0);
    
    auto stats = pool.get_stats();
    EXPECT_EQ(stats.total_clusters, 10);
    EXPECT_EQ(stats.active_clusters, 7);  // ids[5]-ids[9]
    EXPECT_EQ(stats.wide_clusters, 2);
    EXPECT_EQ(stats.closed_clusters, 3);
    EXPECT_EQ(stats.delayed_clusters, 0);
    
    // Берём закрытые
    auto closed = pool.take_closed_clusters(0);
    EXPECT_EQ(closed.size(), 3);
    
    stats = pool.get_stats();
    EXPECT_EQ(stats.closed_clusters, 0);
    EXPECT_EQ(stats.delayed_clusters, 3);
}

TEST_F(ClusterPoolFullTest, StatsEmptyPool) {
    auto& pool = ClusterPool::instance();
    
    auto stats = pool.get_stats();
    EXPECT_EQ(stats.total_clusters, 0);
    EXPECT_EQ(stats.active_clusters, 0);
    EXPECT_EQ(stats.closed_clusters, 0);
    EXPECT_EQ(stats.wide_clusters, 0);
    EXPECT_EQ(stats.delayed_clusters, 0);
    
    for (int i = 0; i < ClusterPool::NUM_SECTORS; ++i) {
        EXPECT_EQ(stats.clusters_by_sector[i], 0);
    }
}


int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
