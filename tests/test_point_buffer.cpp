// tests/test_point_buffer.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/point_buffer.hpp"

using namespace vrl::radar;

class PointBufferTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(10);
    }
    
    StoredPoint create_point(uint16_t azimuth, uint16_t range, bool is_rbs = true) {
        StoredPoint p;
        p.azimuth = azimuth;
        p.range = range;
        p.is_rbs = is_rbs;
        p.amplitude = 100;
        return p;
    }
};

TEST_F(PointBufferTest, Init) {
    EXPECT_TRUE(PointBuffer::instance().is_initialized());
    EXPECT_EQ(PointBuffer::instance().size(), 10);
}

TEST_F(PointBufferTest, AddAndGetPoint) {
    auto p1 = create_point(100, 50);
    size_t idx1 = PointBuffer::instance().add_point(p1);
    EXPECT_EQ(idx1, 0);
    
    const auto& retrieved = PointBuffer::instance().get_point(idx1);
    EXPECT_EQ(retrieved.azimuth, 100);
    EXPECT_EQ(retrieved.range, 50);
}

TEST_F(PointBufferTest, CircularOverwrite) {
    // Добавляем 15 точек в буфер размера 10
    for (int i = 0; i < 15; ++i) {
        auto p = create_point(i, i);
        PointBuffer::instance().add_point(p);
    }
    
    // После 15 добавлений:
    // head = 15 % 10 = 5
    // Индексы 0-4 перезаписаны точками 10-14
    // Индексы 5-9 содержат точки 5-9 (еще не перезаписаны)
    
    // Индекс 0 → точка 10
    const auto& p0 = PointBuffer::instance().get_point(0);
    EXPECT_EQ(p0.azimuth, 10);
    
    // Индекс 4 → точка 14
    const auto& p4 = PointBuffer::instance().get_point(4);
    EXPECT_EQ(p4.azimuth, 14);
    
    // Индекс 5 → точка 5 (еще не перезаписана)
    const auto& p5 = PointBuffer::instance().get_point(5);
    EXPECT_EQ(p5.azimuth, 5);
    
    // Индекс 9 → точка 9 (еще не перезаписана)
    const auto& p9 = PointBuffer::instance().get_point(9);
    EXPECT_EQ(p9.azimuth, 9);
}

TEST_F(PointBufferTest, RBSAndUVD) {
    auto rbs = create_point(100, 50, true);
    auto uvd = create_point(102, 52, false);
    
    size_t idx_rbs = PointBuffer::instance().add_point(rbs);
    size_t idx_uvd = PointBuffer::instance().add_point(uvd);
    
    EXPECT_TRUE(PointBuffer::instance().get_point(idx_rbs).is_rbs);
    EXPECT_FALSE(PointBuffer::instance().get_point(idx_uvd).is_rbs);
}
