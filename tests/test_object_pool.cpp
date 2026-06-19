// tests/test_object_pool.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/object_pool.hpp"
#include "vrl/radar/core/replies.h"
#include <thread>
#include <vector>

using namespace vrl::radar::core;
using namespace vrl::radar;

class ObjectPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        pool_ = std::make_unique<ObjectPool<int>>(10, 20);
    }
    
    std::unique_ptr<ObjectPool<int>> pool_;
};

TEST_F(ObjectPoolTest, AcquireAndRelease) {
    int* ptr = pool_->acquire();
    EXPECT_NE(ptr, nullptr);
    *ptr = 42;
    EXPECT_EQ(*ptr, 42);
    pool_->release(ptr);
}

TEST_F(ObjectPoolTest, PoolReusesObjects) {
    // Забираем объект из пула
    int* ptr1 = pool_->acquire();
    *ptr1 = 123;
    
    // Возвращаем в пул
    pool_->release(ptr1);
    
    // Забираем снова
    int* ptr2 = pool_->acquire();
    EXPECT_NE(ptr2, nullptr);
    EXPECT_EQ(*ptr2, 0);  // Сброшен в состояние по умолчанию
    
    pool_->release(ptr2);
}

TEST_F(ObjectPoolTest, PoolSize) {
    EXPECT_EQ(pool_->size(), 10);  // Начальный размер
    
    int* ptr1 = pool_->acquire();
    int* ptr2 = pool_->acquire();
    EXPECT_EQ(pool_->size(), 8);  // 10 - 2
    
    pool_->release(ptr1);
    pool_->release(ptr2);
    EXPECT_EQ(pool_->size(), 10);  // Вернулись
}

TEST_F(ObjectPoolTest, MaxSize) {
    EXPECT_EQ(pool_->max_size(), 20);
    
    std::vector<int*> ptrs;
    
    // Забираем все объекты из пула
    for (int i = 0; i < 20; ++i) {
        int* ptr = pool_->acquire();
        *ptr = i;
        ptrs.push_back(ptr);
    }
    
    EXPECT_EQ(pool_->size(), 0);
    
    // Создаем новый объект (пул пуст)
    int* ptr = pool_->acquire();
    EXPECT_NE(ptr, nullptr);
    *ptr = 999;
    pool_->release(ptr);
    
    // Возвращаем все обратно
    for (int* p : ptrs) {
        pool_->release(p);
    }
    
    // Должно быть 20 объектов в пуле (максимум)
    EXPECT_EQ(pool_->size(), 20);
}

TEST_F(ObjectPoolTest, Statistics) {
    auto stats = pool_->get_stats();
    EXPECT_EQ(stats.pool_size, 10);
    EXPECT_EQ(stats.max_size, 20);
    EXPECT_EQ(stats.total_acquired, 0);
    EXPECT_EQ(stats.total_created, 0);
    
    int* ptr = pool_->acquire();
    stats = pool_->get_stats();
    EXPECT_EQ(stats.total_acquired, 1);
    EXPECT_EQ(stats.total_created, 0);  // Объект из пула
    
    pool_->release(ptr);
    stats = pool_->get_stats();
    EXPECT_EQ(stats.total_released, 1);
}

TEST_F(ObjectPoolTest, ThreadSafety) {
    auto& pool = *pool_;
    std::vector<std::thread> threads;
    std::atomic<int> counter{0};
    const int NUM_THREADS = 10;
    const int ITERATIONS_PER_THREAD = 100;
    
    for (int i = 0; i < NUM_THREADS; ++i) {
        threads.emplace_back([&pool, &counter]() {
            for (int j = 0; j < ITERATIONS_PER_THREAD; ++j) {
                int* ptr = pool.acquire();
                if (ptr) {
                    *ptr = counter++;
                    pool.release(ptr);
                }
            }
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    auto stats = pool_.get()->get_stats();
    EXPECT_EQ(stats.pool_size, 10);  // Все объекты вернулись
    EXPECT_EQ(stats.total_acquired, NUM_THREADS * ITERATIONS_PER_THREAD);
    EXPECT_EQ(stats.total_released, NUM_THREADS * ITERATIONS_PER_THREAD);
    EXPECT_EQ(stats.total_created, 0);  // Все объекты из пула
}


TEST_F(ObjectPoolTest, PooledObjectWrapper) {
    ObjectPool<int> pool(5, 10);
    
    {
        PooledObject<int> obj(pool, pool.acquire());
        *obj = 42;
        EXPECT_EQ(*obj, 42);
    }  // Автоматически возвращается в пул
    
    EXPECT_EQ(pool.size(), 5);
}

// ============================================================================
// ТЕСТЫ ДЛЯ REPLY POOLS
// ============================================================================

class ReplyPoolsTest : public ::testing::Test {
protected:
    void SetUp() override {
        pools_ = &ReplyPools::instance();
    }
    
    ReplyPools* pools_;
};

TEST_F(ReplyPoolsTest, AcquireRBS) {
    auto rbs = pools_->acquire_rbs();
    EXPECT_NE(rbs.get(), nullptr);
    
    rbs->azimuth = 100;
    rbs->range = 50;
    rbs->code12 = 0x123;
    
    EXPECT_EQ(rbs->azimuth, 100);
    EXPECT_EQ(rbs->range, 50);
    EXPECT_EQ(rbs->code12, 0x123);
}

TEST_F(ReplyPoolsTest, AcquireUVD) {
    auto uvd = pools_->acquire_uvd();
    EXPECT_NE(uvd.get(), nullptr);
    
    uvd->azimuth = 200;
    uvd->range = 100;
    uvd->data20 = 0x12345;
    
    EXPECT_EQ(uvd->azimuth, 200);
    EXPECT_EQ(uvd->range, 100);
    EXPECT_EQ(uvd->data20, 0x12345);
}

TEST_F(ReplyPoolsTest, PoolReusesRBS) {
    // Забираем RBS из пула
    auto rbs1 = pools_->acquire_rbs();
    rbs1->azimuth = 999;
    rbs1->range = 888;
    
    // Возвращаем в пул
    rbs1.reset();
    
    // Забираем снова
    auto rbs2 = pools_->acquire_rbs();
    EXPECT_NE(rbs2.get(), nullptr);
    EXPECT_EQ(rbs2->azimuth, 0);  // Сброшен
    EXPECT_EQ(rbs2->range, 0);    // Сброшен
}

TEST_F(ReplyPoolsTest, PoolReusesUVD) {
    // Забираем UVD из пула
    auto uvd1 = pools_->acquire_uvd();
    uvd1->azimuth = 999;
    uvd1->range = 888;
    
    // Возвращаем в пул
    uvd1.reset();
    
    // Забираем снова
    auto uvd2 = pools_->acquire_uvd();
    EXPECT_NE(uvd2.get(), nullptr);
    EXPECT_EQ(uvd2->azimuth, 0);  // Сброшен
    EXPECT_EQ(uvd2->range, 0);    // Сброшен
}

TEST_F(ReplyPoolsTest, Statistics) {
    auto stats = pools_->get_stats();
    EXPECT_GT(stats.rbs_stats.pool_size, 0);
    EXPECT_GT(stats.uvd_stats.pool_size, 0);
    
    {
        auto rbs = pools_->acquire_rbs();
        auto uvd = pools_->acquire_uvd();
    }
    
    stats = pools_->get_stats();
    EXPECT_GT(stats.rbs_stats.total_acquired, 0);
    EXPECT_GT(stats.uvd_stats.total_acquired, 0);
}

TEST_F(ReplyPoolsTest, MultipleAcquisitions) {
    std::vector<decltype(pools_->acquire_rbs())> rbs_list;
    std::vector<decltype(pools_->acquire_uvd())> uvd_list;
    
    // Забираем много объектов
    for (int i = 0; i < 20; ++i) {
        rbs_list.push_back(pools_->acquire_rbs());
        uvd_list.push_back(pools_->acquire_uvd());
        
        rbs_list.back()->azimuth = i;
        uvd_list.back()->azimuth = i;
    }
    
    // Освобождаем все автоматически при уничтожении
}

TEST_F(ReplyPoolsTest, ResetMethod) {
    auto rbs = pools_->acquire_rbs();
    rbs->azimuth = 100;
    rbs->range = 50;
    rbs->code12 = 0x123;
    rbs->spi = true;
    rbs->is_valid = true;
    
    rbs->reset();
    
    EXPECT_EQ(rbs->azimuth, 0);
    EXPECT_EQ(rbs->range, 0);
    EXPECT_EQ(rbs->code12, 0);
    EXPECT_FALSE(rbs->spi);
    EXPECT_FALSE(rbs->is_valid);
    
    auto uvd = pools_->acquire_uvd();
    uvd->azimuth = 200;
    uvd->range = 100;
    uvd->data20 = 0x12345;
    uvd->is_valid = true;
    
    uvd->reset();
    
    EXPECT_EQ(uvd->azimuth, 0);
    EXPECT_EQ(uvd->range, 0);
    EXPECT_EQ(uvd->data20, 0);
    EXPECT_FALSE(uvd->is_valid);
}
