// tests/test_track_pool.cpp
#include <gtest/gtest.h>
#include "vrl/radar/v2/track_pool.hpp"

using namespace vrl::radar::v2;

class TrackPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        TrackPool::instance().init(100);
    }
    
    void TearDown() override {
        TrackPool::instance().clear();
    }
};

TEST_F(TrackPoolTest, Singleton) {
    auto& pool1 = TrackPool::instance();
    auto& pool2 = TrackPool::instance();
    EXPECT_EQ(&pool1, &pool2);
}

TEST_F(TrackPoolTest, Init) {
    auto& pool = TrackPool::instance();
    EXPECT_TRUE(pool.is_initialized());
    EXPECT_EQ(pool.size(), 100);
}

TEST_F(TrackPoolTest, CreateTrack) {
    auto& pool = TrackPool::instance();
    Track* track = pool.create_track();
    
    ASSERT_NE(track, nullptr);
    EXPECT_GT(track->id, 0);
    EXPECT_EQ(track->state, 0);  // NEW
    EXPECT_EQ(track->hit_count, 0);
    EXPECT_EQ(track->coast_count, 0);
}

TEST_F(TrackPoolTest, CreateMultipleTracks) {
    auto& pool = TrackPool::instance();
    std::vector<uint64_t> ids;
    
    for (int i = 0; i < 10; ++i) {
        Track* track = pool.create_track();
        ASSERT_NE(track, nullptr);
        ids.push_back(track->id);
    }
    
    // Проверяем уникальность ID
    std::sort(ids.begin(), ids.end());
    auto unique = std::unique(ids.begin(), ids.end());
    EXPECT_EQ(unique - ids.begin(), 10);
}

TEST_F(TrackPoolTest, GetTrack) {
    auto& pool = TrackPool::instance();
    Track* created = pool.create_track();
    uint64_t id = created->id;
    
    Track* retrieved = pool.get_track(id);
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->id, id);
}

TEST_F(TrackPoolTest, GetTrackInvalidId) {
    auto& pool = TrackPool::instance();
    Track* track = pool.get_track(999);
    EXPECT_EQ(track, nullptr);
}

TEST_F(TrackPoolTest, GetTrackZeroId) {
    auto& pool = TrackPool::instance();
    Track* track = pool.get_track(0);
    EXPECT_EQ(track, nullptr);
}

TEST_F(TrackPoolTest, TrackReset) {
    auto& pool = TrackPool::instance();
    Track* track = pool.create_track();
    
    // Изменяем состояние
    track->state = 1;
    track->hit_count = 5;
    track->coast_count = 2;
    track->x = 100.0;
    track->y = 200.0;
    
    // Создаем новый трек (должен перезаписать слот)
    Track* new_track = pool.create_track();
    
    // Проверяем, что новый трек сброшен
    EXPECT_EQ(new_track->state, 0);
    EXPECT_EQ(new_track->hit_count, 0);
    EXPECT_EQ(new_track->coast_count, 0);
    EXPECT_EQ(new_track->x, 0.0);
    EXPECT_EQ(new_track->y, 0.0);
}

TEST_F(TrackPoolTest, GetAllTracks) {
    auto& pool = TrackPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        pool.create_track();
    }
    
    auto tracks = pool.get_all_tracks();
    EXPECT_EQ(tracks.size(), 100);  // Все слоты
}

TEST_F(TrackPoolTest, Clear) {
    auto& pool = TrackPool::instance();
    
    for (int i = 0; i < 5; ++i) {
        pool.create_track();
    }
    
    pool.clear();
    
    auto tracks = pool.get_all_tracks();
    for (auto* track : tracks) {
        EXPECT_EQ(track->state, 0);
        EXPECT_EQ(track->hit_count, 0);
        EXPECT_EQ(track->coast_count, 0);
        EXPECT_EQ(track->x, 0.0);
        EXPECT_EQ(track->y, 0.0);
    }
}

TEST_F(TrackPoolTest, CircularBuffer) {
    auto& pool = TrackPool::instance();
    const size_t MAX = 100;
    
    // Создаем больше треков, чем размер пула
    for (size_t i = 0; i < MAX * 1.5; ++i) {
        Track* track = pool.create_track();
        track->x = static_cast<double>(i);
    }
    
    // Проверяем, что циклический буфер работает
    auto tracks = pool.get_all_tracks();
    bool found_old_value = false;
    for (auto* track : tracks) {
        if (track->x < MAX * 0.5 && track->x > 0) {
            found_old_value = true;
            break;
        }
    }
    // Старые значения должны быть перезаписаны
    EXPECT_FALSE(found_old_value);
}
