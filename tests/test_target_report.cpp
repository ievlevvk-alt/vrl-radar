// tests/test_target_report.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/replies.h"

using namespace vrl::radar;

class TargetReportTest : public ::testing::Test {
protected:
    RBSReply create_rbs() {
        RBSReply reply;
        reply.code12 = 0x123;
        reply.azimuth = 100;
        reply.range = 50;
        reply.spi = true;
        reply.is_valid = true;
        return reply;
    }
    
    UVDReply create_uvd() {
        UVDReply reply;
        reply.data20 = 0x12345;
        reply.azimuth = 200;
        reply.range = 100;
        reply.is_valid = true;
        return reply;
    }
};

TEST_F(TargetReportTest, CreateRBSReport) {
    auto report = TargetReport::make_rbs();
    EXPECT_EQ(report.type, TargetReport::SourceType::RBS);
    EXPECT_TRUE(report.sources.empty());
    EXPECT_EQ(report.signal_strength, 0);
}

TEST_F(TargetReportTest, CreateUVDReport) {
    auto report = TargetReport::make_uvd();
    EXPECT_EQ(report.type, TargetReport::SourceType::UVD);
    EXPECT_TRUE(report.sources.empty());
    EXPECT_EQ(report.signal_strength, 0);
}

TEST_F(TargetReportTest, AddRBSSource) {
    auto report = TargetReport::make_rbs();
    auto rbs = create_rbs();
    
    report.add_source(&rbs);
    
    EXPECT_EQ(report.sources.size(), 1);
    EXPECT_TRUE(report.has_sources());
    
    auto rbs_sources = report.get_rbs_sources();
    EXPECT_EQ(rbs_sources.size(), 1);
    EXPECT_EQ(rbs_sources[0]->code12, 0x123);
    EXPECT_EQ(rbs_sources[0]->azimuth, 100);
    
    auto uvd_sources = report.get_uvd_sources();
    EXPECT_TRUE(uvd_sources.empty());
}

TEST_F(TargetReportTest, AddUVDSource) {
    auto report = TargetReport::make_uvd();
    auto uvd = create_uvd();
    
    report.add_source(&uvd);
    
    EXPECT_EQ(report.sources.size(), 1);
    EXPECT_TRUE(report.has_sources());
    
    auto uvd_sources = report.get_uvd_sources();
    EXPECT_EQ(uvd_sources.size(), 1);
    EXPECT_EQ(uvd_sources[0]->data20, 0x12345);
    EXPECT_EQ(uvd_sources[0]->azimuth, 200);
    
    auto rbs_sources = report.get_rbs_sources();
    EXPECT_TRUE(rbs_sources.empty());
}

TEST_F(TargetReportTest, AddMultipleSources) {
    auto report = TargetReport::make_rbs();
    auto rbs1 = create_rbs();
    auto rbs2 = create_rbs();
    auto uvd = create_uvd();
    
    report.add_source(&rbs1);
    report.add_source(&rbs2);
    report.add_source(&uvd);
    
    EXPECT_EQ(report.sources.size(), 3);
    EXPECT_TRUE(report.has_sources());
    
    auto rbs_sources = report.get_rbs_sources();
    EXPECT_EQ(rbs_sources.size(), 2);
    
    auto uvd_sources = report.get_uvd_sources();
    EXPECT_EQ(uvd_sources.size(), 1);
}

TEST_F(TargetReportTest, ClearSources) {
    auto report = TargetReport::make_rbs();
    auto rbs = create_rbs();
    
    report.add_source(&rbs);
    EXPECT_TRUE(report.has_sources());
    
    report.clear_sources();
    EXPECT_TRUE(report.sources.empty());
    EXPECT_FALSE(report.has_sources());
}

TEST_F(TargetReportTest, SourceTypeSafety) {
    auto report = TargetReport::make_rbs();
    auto uvd = create_uvd();
    
    report.add_source(&uvd);
    
    // Проверяем, что get_rbs_sources не путает типы
    auto rbs_sources = report.get_rbs_sources();
    EXPECT_TRUE(rbs_sources.empty());
    
    auto uvd_sources = report.get_uvd_sources();
    EXPECT_EQ(uvd_sources.size(), 1);
}

// ИСПРАВЛЕННЫЙ ТЕСТ: используем visit_sources из replies.h
TEST_F(TargetReportTest, SourceVisitor) {
    auto report = TargetReport::make_rbs();
    auto rbs = create_rbs();
    auto uvd = create_uvd();
    
    report.add_source(&rbs);
    report.add_source(&uvd);
    
    int rbs_count = 0;
    int uvd_count = 0;
    
    // visit_sources теперь доступна из replies.h
    visit_sources(report.sources, [&](const auto* source) {
        if constexpr (std::is_same_v<decltype(source), const RBSReply*>) {
            rbs_count++;
        } else if constexpr (std::is_same_v<decltype(source), const UVDReply*>) {
            uvd_count++;
        }
    });
    
    EXPECT_EQ(rbs_count, 1);
    EXPECT_EQ(uvd_count, 1);
}

TEST_F(TargetReportTest, CopyAndMove) {
    auto report1 = TargetReport::make_rbs();
    auto rbs = create_rbs();
    report1.add_source(&rbs);
    
    // Копирование
    auto report2 = report1;
    EXPECT_EQ(report2.sources.size(), 1);
    EXPECT_EQ(report2.get_rbs_sources().size(), 1);
    
    // Перемещение
    auto report3 = std::move(report1);
    EXPECT_EQ(report3.sources.size(), 1);
    EXPECT_EQ(report3.get_rbs_sources().size(), 1);
    // report1 теперь в неопределенном состоянии, но валидном
    EXPECT_TRUE(report1.sources.empty());
}

// Дополнительный тест: проверка корректности типа через std::get_if
TEST_F(TargetReportTest, DirectVariantAccess) {
    auto report = TargetReport::make_rbs();
    auto rbs = create_rbs();
    auto uvd = create_uvd();
    
    report.add_source(&rbs);
    report.add_source(&uvd);
    
    // Проверяем, что можно получить указатели напрямую через std::get_if
    const auto* first_rbs = std::get_if<const RBSReply*>(&report.sources[0]);
    ASSERT_NE(first_rbs, nullptr);
    EXPECT_EQ((*first_rbs)->code12, 0x123);
    
    const auto* first_uvd = std::get_if<const UVDReply*>(&report.sources[1]);
    ASSERT_NE(first_uvd, nullptr);
    EXPECT_EQ((*first_uvd)->data20, 0x12345);
    
    // Проверяем, что неправильный тип возвращает nullptr
    const auto* wrong = std::get_if<const UVDReply*>(&report.sources[0]);
    EXPECT_EQ(wrong, nullptr);
}

// Тест: проверка, что источники остаются валидными после копирования отчета
TEST_F(TargetReportTest, SourcesRemainValidAfterCopy) {
    auto rbs = create_rbs();
    auto uvd = create_uvd();
    
    auto report1 = TargetReport::make_rbs();
    report1.add_source(&rbs);
    report1.add_source(&uvd);
    
    auto report2 = report1;  // Копирование
    
    auto rbs_sources1 = report1.get_rbs_sources();
    auto rbs_sources2 = report2.get_rbs_sources();
    
    ASSERT_EQ(rbs_sources1.size(), 1);
    ASSERT_EQ(rbs_sources2.size(), 1);
    
    // Указатели должны быть одинаковыми (ссылаются на одни и те же объекты)
    EXPECT_EQ(rbs_sources1[0], rbs_sources2[0]);
    
    auto uvd_sources1 = report1.get_uvd_sources();
    auto uvd_sources2 = report2.get_uvd_sources();
    
    ASSERT_EQ(uvd_sources1.size(), 1);
    ASSERT_EQ(uvd_sources2.size(), 1);
    EXPECT_EQ(uvd_sources1[0], uvd_sources2[0]);
}
