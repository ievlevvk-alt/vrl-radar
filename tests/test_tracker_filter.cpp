// tests/test_tracker_filter.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/tracker.h"
#include "vrl/radar/processing/kalman_filter.h"
#include <memory>

using namespace vrl::radar;

// Простой тестовый фильтр для демонстрации
class TestFilter : public ITrackerFilter {
public:
    void init(double x, double y, uint32_t rev) override {
        x_ = x; y_ = y; rev_ = rev; initialized_ = true;
    }
    void predict(uint32_t delta) override {
        if (!initialized_) return;
        x_ += vx_ * delta;
        y_ += vy_ * delta;
    }
    void update(double x, double y, uint32_t rev) override {
        if (!initialized_) { init(x, y, rev); return; }
        uint32_t delta = rev - rev_;
        if (delta > 0) predict(delta);
        double alpha = 0.5;
        x_ = x_ + alpha * (x - x_);
        y_ = y_ + alpha * (y - y_);
        if (delta > 0) {
            vx_ = (x_ - (x_ - vx_ * delta)) / delta;
            vy_ = (y_ - (y_ - vy_ * delta)) / delta;
        }
        rev_ = rev;
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
    void reset() override { initialized_ = false; x_ = y_ = vx_ = vy_ = 0; }
    std::unique_ptr<ITrackerFilter> clone() const override {
        auto f = std::make_unique<TestFilter>();
        if (initialized_) {
            f->init(x_, y_, rev_);
            f->vx_ = vx_;
            f->vy_ = vy_;
        }
        return f;
    }
    std::string get_name() const override { return "TestFilter"; }
    
private:
    double x_{0}, y_{0}, vx_{0}, vy_{0};
    uint32_t rev_{0};
    bool initialized_{false};
};

TEST(TrackerFilterTest, DefaultKalmanFilter) {
    TrackerConfig config;
    TrackManager manager(config);
    
    EXPECT_NE(manager.get_filter(), nullptr);
    EXPECT_EQ(manager.get_filter()->get_name(), "RevolutionKalmanFilter");
}

TEST(TrackerFilterTest, CustomFilter) {
    TrackerConfig config;
    auto filter = std::make_unique<TestFilter>();
    TrackManager manager(config, std::move(filter));
    
    EXPECT_NE(manager.get_filter(), nullptr);
    EXPECT_EQ(manager.get_filter()->get_name(), "TestFilter");
}

TEST(TrackerFilterTest, ChangeFilter) {
    TrackerConfig config;
    TrackManager manager(config);
    
    EXPECT_EQ(manager.get_filter()->get_name(), "RevolutionKalmanFilter");
    
    manager.set_filter(std::make_unique<TestFilter>());
    EXPECT_EQ(manager.get_filter()->get_name(), "TestFilter");
}

TEST(TrackerFilterTest, FilterCloning) {
    TrackerConfig config;
    auto filter = std::make_unique<TestFilter>();
    TrackManager manager(config, std::move(filter));
    
    // Создаем трек через менеджер
    TargetReport report = TargetReport::make_rbs();
    report.x = 100.0;
    report.y = 100.0;
    report.azimuth_deg = 45.0;
    report.range_m = 141.4;
    
    manager.process_targets({report}, 0);
    auto tracks = manager.get_active_tracks();
    
    EXPECT_EQ(tracks.size(), 1);
    EXPECT_EQ(tracks[0].x, 100.0);
    EXPECT_EQ(tracks[0].y, 100.0);
}
