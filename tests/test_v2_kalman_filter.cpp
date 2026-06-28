// tests/test_v2_kalman_filter.cpp
#include <gtest/gtest.h>
#include "vrl/radar/v2/kalman_filter.hpp"

using namespace vrl::radar::v2;

class V2KalmanFilterTest : public ::testing::Test {
protected:
    void SetUp() override {
        filter_ = std::make_unique<KalmanFilter>(0.1, 1.0);
    }
    
    std::unique_ptr<KalmanFilter> filter_;
};

TEST_F(V2KalmanFilterTest, Init) {
    filter_->init(100.0, 200.0, 0);
    EXPECT_TRUE(filter_->is_initialized());
    EXPECT_DOUBLE_EQ(filter_->get_x(), 100.0);
    EXPECT_DOUBLE_EQ(filter_->get_y(), 200.0);
}

TEST_F(V2KalmanFilterTest, Predict) {
    filter_->init(0.0, 0.0, 0);
    filter_->update(10.0, 20.0, 4096);
    
    auto [pred_x, pred_y] = filter_->predict_position(4096);
    EXPECT_GT(pred_x, 0.0);
    EXPECT_GT(pred_y, 0.0);
}

TEST_F(V2KalmanFilterTest, Clone) {
    filter_->init(100.0, 200.0, 0);
    auto clone = filter_->clone();
    
    EXPECT_TRUE(clone->is_initialized());
    EXPECT_DOUBLE_EQ(clone->get_x(), 100.0);
    EXPECT_DOUBLE_EQ(clone->get_y(), 200.0);
}
