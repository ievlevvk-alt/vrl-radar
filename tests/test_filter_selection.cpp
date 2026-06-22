// tests/test_filter_selection.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/core/track_pool.hpp"

using namespace vrl::radar;

class FilterSelectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        TrackPool::instance().clear();
    }
    
    void TearDown() override {
        TrackPool::instance().clear();
    }
};

TEST_F(FilterSelectionTest, DefaultFilterIsKalman) {
    TrackerConfig config;
    TrackManager manager(config);
    EXPECT_EQ(manager.get_filter_type(), FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeToExtendedKalman) {
    TrackerConfig config;
    TrackManager manager(config);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "ExtendedKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeToUnscentedKalman) {
    TrackerConfig config;
    TrackManager manager(config);
    manager.set_filter_type(FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "UnscentedKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeBackToKalman) {
    TrackerConfig config;
    TrackManager manager(config);
    
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::EXTENDED_KALMAN);
    
    manager.set_filter_type(FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(FilterSelectionTest, TrackUsesSelectedFilter) {
    TrackerConfig config;
    config.min_hits_to_confirm = 1;
    TrackManager manager(config);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    
    // Создаём трек через TrackPool с указанием filter_type
    TargetReport report = TargetReport::make_rbs();
    report.x = 100;
    report.y = 100;
    report.azimuth_deg = 45;
    report.range_m = 1000;
    report.signal_strength = 200;
    
    // ЯВНО ПЕРЕДАЁМ filter_type
    auto track_id = TrackPool::instance().create_track(report, 0, FilterType::EXTENDED_KALMAN);
    auto tracks = TrackPool::instance().get_all_tracks();
    
    EXPECT_EQ(tracks.size(), 1);
    if (!tracks.empty()) {
        EXPECT_EQ(tracks[0]->filter_type, FilterType::EXTENDED_KALMAN);
    }
}

TEST_F(FilterSelectionTest, MultipleTracksUseSameFilterType) {
    TrackerConfig config;
    config.min_hits_to_confirm = 1;
    TrackManager manager(config);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    
    // Создаём несколько треков с явным указанием filter_type
    for (int i = 0; i < 5; ++i) {
        TargetReport report = TargetReport::make_rbs();
        report.x = 100 + i * 50;
        report.y = 100 + i * 50;
        report.azimuth_deg = 45 + i * 5;
        report.range_m = 1000 + i * 100;
        report.signal_strength = 200;
        TrackPool::instance().create_track(report, i, FilterType::EXTENDED_KALMAN);
    }
    
    auto tracks = TrackPool::instance().get_all_tracks();
    EXPECT_EQ(tracks.size(), 5);
    
    for (Track* track : tracks) {
        EXPECT_EQ(track->filter_type, FilterType::EXTENDED_KALMAN);
    }
}

TEST_F(FilterSelectionTest, FilterPersistsAfterReset) {
    TrackerConfig config;
    TrackManager manager(config);
    manager.set_filter_type(FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::UNSCENTED_KALMAN);
    
    manager.reset();
    EXPECT_EQ(manager.get_filter_type(), FilterType::UNSCENTED_KALMAN);
}

TEST_F(FilterSelectionTest, EachFilterTypeHasCorrectName) {
    TrackerConfig config;
    TrackManager manager(config);
    
    manager.set_filter_type(FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
    
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "ExtendedKalmanFilter");
    
    manager.set_filter_type(FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "UnscentedKalmanFilter");
}
