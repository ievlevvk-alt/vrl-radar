// tests/test_plot_pool.cpp
#include <gtest/gtest.h>
#include "vrl/radar/v2/plot_pool.hpp"

using namespace vrl::radar::v2;

class PlotPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        PlotPool::instance().init(100);
    }
    
    void TearDown() override {
        PlotPool::instance().clear();
    }
    
    Plot create_test_plot(uint64_t id = 1) {
        Plot plot;
        plot.x = 100.0;
        plot.y = 200.0;
        plot.azimuth_maia = 512;
        plot.range_bins = 100;
        plot.mode3a_code = 1234;
        plot.confidence = 0.9;
        plot.source_type = Plot::SourceType::RBS;
        return plot;
    }
};

TEST_F(PlotPoolTest, Singleton) {
    auto& pool1 = PlotPool::instance();
    auto& pool2 = PlotPool::instance();
    EXPECT_EQ(&pool1, &pool2);
}

TEST_F(PlotPoolTest, Init) {
    auto& pool = PlotPool::instance();
    EXPECT_TRUE(pool.is_initialized());
    EXPECT_EQ(pool.size(), 100);
}

TEST_F(PlotPoolTest, AddPlot) {
    auto& pool = PlotPool::instance();
    Plot plot = create_test_plot();
    
    uint64_t index = pool.add_plot(plot);
    EXPECT_GT(index, 0);
    
    const Plot* retrieved = pool.get_plot(index);
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->x, plot.x);
    EXPECT_EQ(retrieved->y, plot.y);
    EXPECT_EQ(retrieved->azimuth_maia, plot.azimuth_maia);
}

TEST_F(PlotPoolTest, AddMultiplePlots) {
    auto& pool = PlotPool::instance();
    std::vector<uint64_t> indices;
    
    for (int i = 0; i < 10; ++i) {
        Plot plot = create_test_plot(i + 1);
        uint64_t index = pool.add_plot(plot);
        indices.push_back(index);
    }
    
    // Проверяем уникальность индексов
    std::sort(indices.begin(), indices.end());
    auto unique = std::unique(indices.begin(), indices.end());
    EXPECT_EQ(unique - indices.begin(), 10);
}

TEST_F(PlotPoolTest, GetPlot) {
    auto& pool = PlotPool::instance();
    Plot plot = create_test_plot();
    uint64_t index = pool.add_plot(plot);
    
    const Plot* retrieved = pool.get_plot(index);
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->source_type, Plot::SourceType::RBS);
}

TEST_F(PlotPoolTest, GetPlotInvalidIndex) {
    auto& pool = PlotPool::instance();
    const Plot* plot = pool.get_plot(999);
    EXPECT_EQ(plot, nullptr);
}

TEST_F(PlotPoolTest, GetPlotZeroIndex) {
    auto& pool = PlotPool::instance();
    const Plot* plot = pool.get_plot(0);
    EXPECT_EQ(plot, nullptr);
}

TEST_F(PlotPoolTest, CircularBuffer) {
    auto& pool = PlotPool::instance();
    const size_t MAX = 100;
    
    // Добавляем больше плотов, чем размер пула
    for (size_t i = 0; i < MAX * 1.5; ++i) {
        Plot plot = create_test_plot(i);
        plot.x = static_cast<double>(i);
        pool.add_plot(plot);
    }
    
    // Проверяем, что старые данные перезаписаны
    const Plot* plot = pool.get_plot(1);
    if (plot) {
        // Если индекс 1 существует, его значение должно быть от MAX * 0.5
        EXPECT_GE(plot->x, MAX * 0.5);
    }
}

TEST_F(PlotPoolTest, Clear) {
    auto& pool = PlotPool::instance();
    
    for (int i = 0; i < 10; ++i) {
        Plot plot = create_test_plot(i);
        pool.add_plot(plot);
    }
    
    pool.clear();
    
    // После очистки все плоты должны быть сброшены
    for (int i = 1; i <= 10; ++i) {
        const Plot* plot = pool.get_plot(i);
        if (plot) {
            EXPECT_EQ(plot->x, 0.0);
            EXPECT_EQ(plot->y, 0.0);
        }
    }
}

TEST_F(PlotPoolTest, PlotValidation) {
    Plot plot;
    plot.x = 0.0;
    plot.y = 0.0;
    
    EXPECT_FALSE(plot.is_valid());
    
    plot.x = 100.0;
    EXPECT_TRUE(plot.is_valid());
}
