// tests/test_grid_index.cpp
#include <gtest/gtest.h>
#include "vrl/radar/v2/grid_index.hpp"
#include "vrl/radar/v2/grid_config.hpp"

using namespace vrl::radar::v2;

class GridIndexTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.cell_size_km = 5.0;
        config_.max_range_km = 400.0;
        config_.rings_near = 1;
        config_.rings_far = 2;
        config_.far_threshold_km = 150.0;
        grid_index_ = std::make_unique<GridIndex>(config_);
    }

    GridConfig config_;
    std::unique_ptr<GridIndex> grid_index_;
};

TEST_F(GridIndexTest, AddTrack) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    EXPECT_TRUE(grid_index_->has_track(1));
    
    auto tracks = grid_index_->get_all_tracks();
    ASSERT_EQ(tracks.size(), 1);
    EXPECT_EQ(tracks[0], 1);
}

TEST_F(GridIndexTest, AddMultipleTracks) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->add_track(2, 20000.0, 20000.0);
    grid_index_->add_track(3, 30000.0, 30000.0);
    
    auto tracks = grid_index_->get_all_tracks();
    ASSERT_EQ(tracks.size(), 3);
}

TEST_F(GridIndexTest, UpdateTrack) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->update_track(1, 20000.0, 20000.0);
    
    EXPECT_TRUE(grid_index_->has_track(1));
    
    auto nearby = grid_index_->get_nearby_tracks(20000.0, 20000.0);
    ASSERT_FALSE(nearby.empty());
    EXPECT_EQ(nearby[0], 1);
}

TEST_F(GridIndexTest, UpdateTrackSameCell) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->update_track(1, 10001.0, 10000.0);
    
    EXPECT_TRUE(grid_index_->has_track(1));
}

TEST_F(GridIndexTest, RemoveTrack) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->remove_track(1);
    
    EXPECT_FALSE(grid_index_->has_track(1));
    auto tracks = grid_index_->get_all_tracks();
    EXPECT_TRUE(tracks.empty());
}

TEST_F(GridIndexTest, RemoveNonExistentTrack) {
    EXPECT_NO_THROW(grid_index_->remove_track(999));
}

TEST_F(GridIndexTest, GetNearbyTracks) {
    grid_index_->add_track(1, 0.0, 0.0);
    grid_index_->add_track(2, 10000.0, 10000.0);
    grid_index_->add_track(3, 100000.0, 100000.0);
    
    auto nearby = grid_index_->get_nearby_tracks(0.0, 0.0);
    EXPECT_FALSE(nearby.empty());
}

TEST_F(GridIndexTest, GetNearbyTracksWithRings) {
    grid_index_->add_track(1, 0.0, 0.0);
    grid_index_->add_track(2, 30000.0, 0.0);
    grid_index_->add_track(3, 50000.0, 0.0);
    
    auto nearby = grid_index_->get_nearby_tracks(0.0, 0.0, 2);
    EXPECT_FALSE(nearby.empty());
}

TEST_F(GridIndexTest, GetNearbyTracksOutOfRange) {
    grid_index_->add_track(1, 500000.0, 500000.0);
    
    auto tracks = grid_index_->get_all_tracks();
    EXPECT_TRUE(tracks.empty());
}

TEST_F(GridIndexTest, GetStats) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->add_track(2, 20000.0, 20000.0);
    grid_index_->add_track(3, 30000.0, 30000.0);
    
    auto stats = grid_index_->get_stats();
    EXPECT_EQ(stats.total_tracks, 3);
    EXPECT_GT(stats.non_empty_cells, 0);
}

TEST_F(GridIndexTest, Clear) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->add_track(2, 20000.0, 20000.0);
    
    grid_index_->clear();
    
    auto tracks = grid_index_->get_all_tracks();
    EXPECT_TRUE(tracks.empty());
    auto stats = grid_index_->get_stats();
    EXPECT_EQ(stats.total_tracks, 0);
    EXPECT_EQ(stats.non_empty_cells, 0);
}

TEST_F(GridIndexTest, DuplicateTrack) {
    grid_index_->add_track(1, 10000.0, 10000.0);
    grid_index_->add_track(1, 10000.0, 10000.0);
    
    auto tracks = grid_index_->get_all_tracks();
    ASSERT_EQ(tracks.size(), 1);
    EXPECT_EQ(tracks[0], 1);
}

TEST_F(GridIndexTest, GetCellKey) {
    auto key = grid_index_->get_cell_key(10000.0, 10000.0);
    EXPECT_NE(key.x, 0);
    EXPECT_NE(key.y, 0);
    
    auto key1 = grid_index_->get_cell_key(10000.0, 10000.0);
    auto key2 = grid_index_->get_cell_key(10001.0, 10000.0);
    EXPECT_EQ(key1.x, key2.x);
    EXPECT_EQ(key1.y, key2.y);
    
    auto key3 = grid_index_->get_cell_key(50000.0, 50000.0);
    EXPECT_NE(key1.x, key3.x);
    EXPECT_NE(key1.y, key3.y);
}

// ===== ИСПРАВЛЕННЫЙ ТЕСТ =====
TEST_F(GridIndexTest, GetRingsForRange) {
    // Близкая дальность → rings_near
    int rings_near = grid_index_->get_rings_for_range(50000.0);
    EXPECT_EQ(rings_near, config_.rings_near);
    
    // Дальняя дальность → rings_far
    int rings_far = grid_index_->get_rings_for_range(200000.0);
    EXPECT_EQ(rings_far, config_.rings_far);
    
    // На границе → rings_near (<= far_threshold)
    int rings_boundary = grid_index_->get_rings_for_range(150000.0);
    EXPECT_EQ(rings_boundary, config_.rings_near);
    
    // Чуть дальше границы → rings_far
    int rings_far2 = grid_index_->get_rings_for_range(150001.0);
    EXPECT_EQ(rings_far2, config_.rings_far);
}

// ===== ИСПРАВЛЕННЫЙ ТЕСТ =====
TEST_F(GridIndexTest, IsInRange) {
    auto key_near = grid_index_->get_cell_key(100000.0, 0.0);
    EXPECT_TRUE(grid_index_->is_in_range(key_near));
    
    auto key_far = grid_index_->get_cell_key(500000.0, 0.0);
    EXPECT_FALSE(grid_index_->is_in_range(key_far));
    
    // Используем значение, при котором центр ячейки будет на границе
    // cell_size_m_ = 5000 м (5 км)
    // Чтобы центр ячейки был на 400000 м, нужно x = 400000 - 2500 = 397500
    auto key_boundary = grid_index_->get_cell_key(397500.0, 0.0);
    EXPECT_TRUE(grid_index_->is_in_range(key_boundary));
}

TEST_F(GridIndexTest, GetNeighborCells) {
    auto center = grid_index_->get_cell_key(0.0, 0.0);
    
    auto neighbors_1 = grid_index_->get_neighbor_cells(center, 1);
    EXPECT_EQ(neighbors_1.size(), 9);
    
    auto neighbors_2 = grid_index_->get_neighbor_cells(center, 2);
    EXPECT_EQ(neighbors_2.size(), 25);
}
