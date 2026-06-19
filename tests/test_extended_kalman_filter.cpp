// tests/test_extended_kalman_filter.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/extended_kalman_filter.h"
#include <cmath>

using namespace vrl::radar;

class ExtendedKalmanFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = std::make_unique<ExtendedKalmanFilter>(0.1, 1.0);
    }
    
    std::unique_ptr<ExtendedKalmanFilter> filter_;
};

TEST_F(ExtendedKalmanFilterTest, InitialState) {
    EXPECT_FALSE(filter_->is_initialized());
    EXPECT_DOUBLE_EQ(filter_->get_x(), 0.0);
    EXPECT_DOUBLE_EQ(filter_->get_y(), 0.0);
    EXPECT_DOUBLE_EQ(filter_->get_vx(), 0.0);
    EXPECT_DOUBLE_EQ(filter_->get_vy(), 0.0);
}

TEST_F(ExtendedKalmanFilterTest, InitSetsState) {
    filter_->init(100.0, 200.0, 0);
    
    EXPECT_TRUE(filter_->is_initialized());
    EXPECT_DOUBLE_EQ(filter_->get_x(), 100.0);
    EXPECT_DOUBLE_EQ(filter_->get_y(), 200.0);
}

TEST_F(ExtendedKalmanFilterTest, UpdateWithoutInit) {
    filter_->update(100.0, 200.0, 0);
    EXPECT_TRUE(filter_->is_initialized());
    EXPECT_DOUBLE_EQ(filter_->get_x(), 100.0);
    EXPECT_DOUBLE_EQ(filter_->get_y(), 200.0);
}

TEST_F(ExtendedKalmanFilterTest, PredictPosition) {
    filter_->init(0.0, 0.0, 0);
    filter_->update(100.0, 0.0, 10);
    filter_->update(200.0, 0.0, 20);
    
    auto [x, y] = filter_->predict_position(5);
    EXPECT_GT(x, filter_->get_x());
    EXPECT_DOUBLE_EQ(y, filter_->get_y());
}

TEST_F(ExtendedKalmanFilterTest, SpeedAndCourse) {
    filter_->init(0.0, 0.0, 0);
    
    for (int i = 1; i <= 10; ++i) {
        filter_->update(10.0 * i, 10.0 * i, i * 10);
    }
    
    double speed = filter_->get_speed();
    double course = filter_->get_course();
    
    EXPECT_GT(speed, 0.0);
    EXPECT_GE(course, 0.0);
    EXPECT_LE(course, 360.0);
}

TEST_F(ExtendedKalmanFilterTest, AccelerationModel) {
    ExtendedKalmanFilter accel_filter(0.1, 1.0, true);
    accel_filter.init(0.0, 0.0, 0);
    
    for (int i = 1; i <= 5; ++i) {
        accel_filter.update(100.0 * i * i, 0.0, i * 10);
    }
    
    EXPECT_TRUE(accel_filter.is_initialized());
    EXPECT_GT(accel_filter.get_ax(), 0.0);
}

TEST_F(ExtendedKalmanFilterTest, Clone) {
    filter_->init(100.0, 200.0, 0);
    filter_->update(150.0, 250.0, 10);
    
    auto clone = filter_->clone();
    EXPECT_TRUE(clone->is_initialized());
    EXPECT_DOUBLE_EQ(clone->get_x(), filter_->get_x());
    EXPECT_DOUBLE_EQ(clone->get_y(), filter_->get_y());
    EXPECT_DOUBLE_EQ(clone->get_vx(), filter_->get_vx());
    EXPECT_DOUBLE_EQ(clone->get_vy(), filter_->get_vy());
}

TEST_F(ExtendedKalmanFilterTest, Reset) {
    filter_->init(100.0, 200.0, 0);
    EXPECT_TRUE(filter_->is_initialized());
    
    filter_->reset();
    EXPECT_FALSE(filter_->is_initialized());
    EXPECT_DOUBLE_EQ(filter_->get_x(), 0.0);
    EXPECT_DOUBLE_EQ(filter_->get_y(), 0.0);
}
