// tests/test_filter_selection.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/processing/extended_kalman_filter.h"
#include "vrl/radar/processing/unscented_kalman_filter.h"
#include <memory>

using namespace vrl::radar;

class FilterSelectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_.min_hits_to_confirm = 1;
        config_.max_coast_count = 100;  // Увеличиваем, чтобы треки не удалялись
        config_.max_gate_distance = 300.0;
        config_.max_gate_azimuth = 30.0;
        config_.process_noise = 0.5;
        config_.measurement_noise = 0.1;
    }
    
    TrackerConfig config_;
};

TEST_F(FilterSelectionTest, DefaultFilterIsKalman) {
    TrackManager manager(config_);
    EXPECT_EQ(manager.get_filter_type(), FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeToExtendedKalman) {
    TrackManager manager(config_);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "ExtendedKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeToUnscentedKalman) {
    TrackManager manager(config_);
    manager.set_filter_type(FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::UNSCENTED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "UnscentedKalmanFilter");
}

TEST_F(FilterSelectionTest, SetFilterTypeBackToKalman) {
    TrackManager manager(config_);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "ExtendedKalmanFilter");
    
    manager.set_filter_type(FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_type(), FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(FilterSelectionTest, TrackUsesSelectedFilter) {
    TrackManager manager(config_);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    
    TargetReport report = TargetReport::make_rbs();
    report.x = 100.0;
    report.y = 100.0;
    report.azimuth_deg = 45.0;
    report.range_m = 141.4;
    report.rbs.mode3a_code = 1234;
    
    for (int i = 0; i < 2; ++i) {
        manager.process_targets({report}, i * 10);
    }
    
    auto tracks = manager.get_active_tracks();
    EXPECT_EQ(tracks.size(), 1);
    EXPECT_EQ(tracks[0].filter_type, FilterType::EXTENDED_KALMAN);
}

TEST_F(FilterSelectionTest, CustomFilterOverridesDefault) {
    auto custom_filter = std::make_unique<ExtendedKalmanFilter>(0.5, 0.1);
    TrackManager manager(config_, std::move(custom_filter));
    
    EXPECT_EQ(manager.get_filter_name(), "ExtendedKalmanFilter");
    
    manager.set_filter_type(FilterType::KALMAN);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(FilterSelectionTest, MultipleTracksUseSameFilterType) {
    TrackManager manager(config_);
    manager.set_filter_type(FilterType::EXTENDED_KALMAN);
    
    // Создаем 3 разные цели с сильно разными координатами
    // и разными кодами, чтобы они не объединялись
    
    // Цель 1 - близко
    TargetReport r1 = TargetReport::make_rbs();
    r1.x = 100.0;
    r1.y = 100.0;
    r1.azimuth_deg = 45.0;
    r1.range_m = 141.4;
    r1.rbs.mode3a_code = 1234;
    
    // Цель 2 - далеко по X
    TargetReport r2 = TargetReport::make_rbs();
    r2.x = 500.0;
    r2.y = 100.0;
    r2.azimuth_deg = 78.7;
    r2.range_m = 509.9;
    r2.rbs.mode3a_code = 5678;
    
    // Цель 3 - далеко по Y
    TargetReport r3 = TargetReport::make_rbs();
    r3.x = 100.0;
    r3.y = 500.0;
    r3.azimuth_deg = 11.3;
    r3.range_m = 509.9;
    r3.rbs.mode3a_code = 9012;
    
    // Обрабатываем каждую цель отдельно с обновлениями
    for (int i = 0; i < 3; ++i) {
        TargetReport report;
        if (i == 0) report = r1;
        else if (i == 1) report = r2;
        else report = r3;
        
        // 3 обновления для каждой цели
        for (int j = 0; j < 3; ++j) {
            manager.process_targets({report}, i * 10 + j);
        }
    }
    
    auto tracks = manager.get_active_tracks();
    
    // Проверяем, что у нас 3 трека
    EXPECT_EQ(tracks.size(), 3);
    
    // Проверяем, что все треки используют правильный фильтр
    for (const auto& track : tracks) {
        EXPECT_EQ(track.filter_type, FilterType::EXTENDED_KALMAN);
    }
}
