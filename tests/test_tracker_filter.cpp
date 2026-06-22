// tests/test_tracker_filter.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/core/track_pool.hpp"

using namespace vrl::radar;

// Тестовый фильтр
class TestFilter : public ITrackerFilter {
public:
    void init(double x, double y, uint32_t revolution) override {
        x_ = x; 
        y_ = y; 
        initialized_ = true;
        last_revolution_ = revolution;
    }
    
    void predict(uint32_t delta_revolutions) override {
        if (!initialized_) return;
        // Простая модель: движение с постоянной скоростью
        x_ += vx_ * delta_revolutions;
        y_ += vy_ * delta_revolutions;
    }
    
    void update(double x, double y, uint32_t revolution) override {
        if (!initialized_) {
            init(x, y, revolution);
            return;
        }
        
        uint32_t delta_rev = revolution - last_revolution_;
        if (delta_rev > 0) {
            predict(delta_rev);
        }
        
        // Простое обновление
        double alpha = 0.5;
        x_ = x_ + alpha * (x - x_);
        y_ = y_ + alpha * (y - y_);
        
        if (delta_rev > 0) {
            vx_ = (x_ - (x_ - vx_ * delta_rev)) / delta_rev;
            vy_ = (y_ - (y_ - vy_ * delta_rev)) / delta_rev;
        }
        
        last_revolution_ = revolution;
    }
    
    double get_x() const override { return x_; }
    double get_y() const override { return y_; }
    double get_vx() const override { return vx_; }
    double get_vy() const override { return vy_; }
    double get_speed() const override { return std::sqrt(vx_*vx_ + vy_*vy_); }
    double get_course() const override { return std::atan2(vx_, vy_) * 180.0 / M_PI; }
    
    std::pair<double, double> predict_position(uint32_t delta) const override {
        return {x_ + vx_ * delta, y_ + vy_ * delta};
    }
    
    bool is_initialized() const override { return initialized_; }
    void reset() override { 
        initialized_ = false; 
        x_ = 0; y_ = 0; vx_ = 0; vy_ = 0;
    }
    
    std::unique_ptr<ITrackerFilter> clone() const override {
        auto clone = std::make_unique<TestFilter>();
        if (initialized_) {
            clone->init(x_, y_, last_revolution_);
            clone->vx_ = vx_;
            clone->vy_ = vy_;
        }
        return clone;
    }
    
    std::string get_name() const override { return "TestFilter"; }
    
private:
    double x_{0.0}, y_{0.0};
    double vx_{0.0}, vy_{0.0};
    uint32_t last_revolution_{0};
    bool initialized_{false};
};

class TrackerFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        TrackPool::instance().clear();
    }
    
    void TearDown() override {
        TrackPool::instance().clear();
    }
};

TEST_F(TrackerFilterTest, DefaultKalmanFilter) {
    TrackerConfig config;
    config.min_hits_to_confirm = 3;
    TrackManager manager(config);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
}

TEST_F(TrackerFilterTest, CustomFilter) {
    TrackerConfig config;
    auto filter = std::make_unique<TestFilter>();
    TrackManager manager(config, std::move(filter));
    EXPECT_EQ(manager.get_filter_name(), "TestFilter");
}

TEST_F(TrackerFilterTest, ChangeFilter) {
    TrackerConfig config;
    TrackManager manager(config);
    EXPECT_EQ(manager.get_filter_name(), "RevolutionKalmanFilter");
    
    auto filter = std::make_unique<TestFilter>();
    manager.set_filter(std::move(filter));
    EXPECT_EQ(manager.get_filter_name(), "TestFilter");
}

TEST_F(TrackerFilterTest, FilterCloning) {
    TrackerConfig config;
    config.min_hits_to_confirm = 1;
    TrackManager manager(config);
    
    // Создаём трек через TrackPool
    TargetReport report = TargetReport::make_rbs();
    report.x = 100;
    report.y = 100;
    report.azimuth_deg = 45;
    report.range_m = 1000;
    report.signal_strength = 200;
    
    auto track_id = TrackPool::instance().create_track(report, 0);
    auto tracks = TrackPool::instance().get_all_tracks();
    
    // Проверяем, что трек создан
    EXPECT_EQ(tracks.size(), 1);
}

TEST_F(TrackerFilterTest, FilterPredictAndUpdate) {
    auto filter = std::make_unique<TestFilter>();
    filter->init(100, 100, 0);
    
    EXPECT_TRUE(filter->is_initialized());
    EXPECT_EQ(filter->get_x(), 100);
    EXPECT_EQ(filter->get_y(), 100);
    
    // Предсказание на 2 оборота
    auto [x, y] = filter->predict_position(2);
    EXPECT_EQ(x, 100);  // Скорость 0, позиция не меняется
    EXPECT_EQ(y, 100);
    
    // Обновление с новой позицией
    filter->update(110, 110, 2);
    EXPECT_NEAR(filter->get_x(), 105, 0.1);
    EXPECT_NEAR(filter->get_y(), 105, 0.1);
}

TEST_F(TrackerFilterTest, FilterReset) {
    auto filter = std::make_unique<TestFilter>();
    filter->init(100, 100, 0);
    EXPECT_TRUE(filter->is_initialized());
    
    filter->reset();
    EXPECT_FALSE(filter->is_initialized());
    EXPECT_EQ(filter->get_x(), 0);
    EXPECT_EQ(filter->get_y(), 0);
}
