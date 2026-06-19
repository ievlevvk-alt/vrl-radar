// tests/test_circular_history.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/tracker.h"

using namespace vrl::radar;

// Определяем типы для тестов
using IntHistory = CircularHistory<int, 5>;
using StringHistory = CircularHistory<std::string, 5>;

class CircularHistoryTest : public ::testing::Test {
protected:
    IntHistory int_hist;
    StringHistory string_hist;
};

TEST_F(CircularHistoryTest, EmptyHistory) {
    EXPECT_TRUE(int_hist.empty());
    EXPECT_EQ(int_hist.size(), 0);
    EXPECT_EQ(int_hist.max_size(), 5);
    EXPECT_FALSE(int_hist.is_full());
    EXPECT_EQ(int_hist.back(), nullptr);
    EXPECT_EQ(int_hist.front(), nullptr);
}

TEST_F(CircularHistoryTest, PushElements) {
    int_hist.push(10);
    EXPECT_FALSE(int_hist.empty());
    EXPECT_EQ(int_hist.size(), 1);
    EXPECT_EQ(*int_hist.back(), 10);
    EXPECT_EQ(*int_hist.front(), 10);
    
    int_hist.push(20);
    EXPECT_EQ(int_hist.size(), 2);
    EXPECT_EQ(*int_hist.back(), 20);
    EXPECT_EQ(*int_hist.front(), 10);
}

TEST_F(CircularHistoryTest, GetElements) {
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    EXPECT_EQ(int_hist.size(), 5);
    EXPECT_TRUE(int_hist.is_full());
    
    auto all = int_hist.get_all();
    EXPECT_EQ(all.size(), 5);
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(all[i], i + 1);
    }
}

TEST_F(CircularHistoryTest, OverwriteOldest) {
    // Заполняем буфер
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    // Добавляем еще один - перезаписываем самый старый
    int_hist.push(6);
    
    EXPECT_EQ(int_hist.size(), 5);
    auto all = int_hist.get_all();
    EXPECT_EQ(all[0], 2);  // 1 был перезаписан
    EXPECT_EQ(all[1], 3);
    EXPECT_EQ(all[2], 4);
    EXPECT_EQ(all[3], 5);
    EXPECT_EQ(all[4], 6);
}

TEST_F(CircularHistoryTest, BackAndFrontAfterOverwrite) {
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    EXPECT_EQ(*int_hist.front(), 1);
    EXPECT_EQ(*int_hist.back(), 5);
    
    int_hist.push(6);
    
    EXPECT_EQ(*int_hist.front(), 2);
    EXPECT_EQ(*int_hist.back(), 6);
}

TEST_F(CircularHistoryTest, GetByIndex) {
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    for (int i = 0; i < 5; ++i) {
        const int* val = int_hist.get(i);
        ASSERT_NE(val, nullptr);
        EXPECT_EQ(*val, i + 1);
    }
    
    // Индекс за пределами
    EXPECT_EQ(int_hist.get(5), nullptr);
}

TEST_F(CircularHistoryTest, RangeBasedFor) {
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    int expected = 1;
    for (const int& val : int_hist) {
        EXPECT_EQ(val, expected++);
    }
}

TEST_F(CircularHistoryTest, Clear) {
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    EXPECT_EQ(int_hist.size(), 5);
    int_hist.clear();
    EXPECT_TRUE(int_hist.empty());
    EXPECT_EQ(int_hist.size(), 0);
    EXPECT_EQ(int_hist.back(), nullptr);
    EXPECT_EQ(int_hist.front(), nullptr);
    
    // После очистки можно добавлять снова
    int_hist.push(42);
    EXPECT_EQ(int_hist.size(), 1);
    EXPECT_EQ(*int_hist.back(), 42);
}

TEST_F(CircularHistoryTest, MultipleOverwrites) {
    // Заполняем
    for (int i = 1; i <= 5; ++i) {
        int_hist.push(i);
    }
    
    // Перезаписываем несколько раз
    for (int i = 6; i <= 20; ++i) {
        int_hist.push(i);
    }
    
    EXPECT_EQ(int_hist.size(), 5);
    auto all = int_hist.get_all();
    
    // Последние 5 элементов: 16, 17, 18, 19, 20
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(all[i], 16 + i);
    }
}

// Тест для строк (move-семантика)
TEST_F(CircularHistoryTest, StringPush) {
    std::string s1 = "hello";
    std::string s2 = "world";
    
    string_hist.push(s1);  // Копирование
    string_hist.push(std::move(s2));  // Перемещение
    
    auto all = string_hist.get_all();
    EXPECT_EQ(all.size(), 2);
    EXPECT_EQ(all[0], "hello");
    EXPECT_EQ(all[1], "world");
    
    // s2 должен быть в перемещенном состоянии
    EXPECT_TRUE(s2.empty());
}

// Тест для строк с перезаписью
TEST_F(CircularHistoryTest, StringOverwrite) {
    for (int i = 0; i < 5; ++i) {
        string_hist.push("Item " + std::to_string(i));
    }
    
    EXPECT_EQ(string_hist.size(), 5);
    EXPECT_TRUE(string_hist.is_full());
    
    string_hist.push("Item 5");  // Перезаписывает самый старый
    
    auto all = string_hist.get_all();
    EXPECT_EQ(all.size(), 5);
    EXPECT_EQ(all[0], "Item 1");  // "Item 0" был перезаписан
    EXPECT_EQ(all[4], "Item 5");
}

// Тест с TargetReport (реальный тип)
TEST_F(CircularHistoryTest, TargetReportHistory) {
    CircularHistory<TargetReport, 10> report_hist;
    
    EXPECT_TRUE(report_hist.empty());
    EXPECT_EQ(report_hist.max_size(), 10);
    
    // Создаем несколько отчетов
    for (int i = 0; i < 8; ++i) {
        auto report = TargetReport::make_rbs();
        report.x = static_cast<double>(i * 10);
        report.y = static_cast<double>(i * 5);
        report.azimuth_deg = static_cast<double>(i * 10);
        report.range_m = static_cast<double>(i * 100);
        report_hist.push(std::move(report));
    }
    
    EXPECT_EQ(report_hist.size(), 8);
    EXPECT_FALSE(report_hist.is_full());
    
    // Добавляем еще 3 отчета (переполнение на 1)
    for (int i = 8; i < 11; ++i) {
        auto report = TargetReport::make_rbs();
        report.x = static_cast<double>(i * 10);
        report.y = static_cast<double>(i * 5);
        report_hist.push(std::move(report));
    }
    
    EXPECT_EQ(report_hist.size(), 10);
    EXPECT_TRUE(report_hist.is_full());
    
    auto all = report_hist.get_all();
    EXPECT_EQ(all.size(), 10);
    
    // Проверяем, что самый старый был перезаписан (i=0 должен отсутствовать)
    // Первый элемент должен быть i=1
    EXPECT_EQ(all[0].x, 10.0);
    EXPECT_EQ(all[0].y, 5.0);
    
    // Последний элемент должен быть i=10
    EXPECT_EQ(all[9].x, 100.0);
    EXPECT_EQ(all[9].y, 50.0);
}

// Тест: const методы
TEST_F(CircularHistoryTest, ConstMethods) {
    for (int i = 1; i <= 3; ++i) {
        int_hist.push(i);
    }
    
    const IntHistory& const_hist = int_hist;
    
    EXPECT_EQ(const_hist.size(), 3);
    EXPECT_FALSE(const_hist.empty());
    EXPECT_EQ(const_hist.max_size(), 5);
    EXPECT_FALSE(const_hist.is_full());
    
    const int* back = const_hist.back();
    ASSERT_NE(back, nullptr);
    EXPECT_EQ(*back, 3);
    
    const int* front = const_hist.front();
    ASSERT_NE(front, nullptr);
    EXPECT_EQ(*front, 1);
    
    const int* second = const_hist.get(1);
    ASSERT_NE(second, nullptr);
    EXPECT_EQ(*second, 2);
    
    auto all = const_hist.get_all();
    EXPECT_EQ(all.size(), 3);
    EXPECT_EQ(all[0], 1);
    EXPECT_EQ(all[1], 2);
    EXPECT_EQ(all[2], 3);
}

// Тест: итераторы для пустой истории
TEST_F(CircularHistoryTest, EmptyRangeBasedFor) {
    // Итерация по пустой истории не должна вызывать проблем
    int count = 0;
    for (const int& val : int_hist) {
        (void)val;
        count++;
    }
    EXPECT_EQ(count, 0);
}

// Тест: большое количество операций
TEST_F(CircularHistoryTest, StressTest) {
    const int NUM_OPERATIONS = 1000;
    
    for (int i = 0; i < NUM_OPERATIONS; ++i) {
        int_hist.push(i);
    }
    
    EXPECT_EQ(int_hist.size(), 5);  // Размер буфера
    EXPECT_TRUE(int_hist.is_full());
    
    auto all = int_hist.get_all();
    EXPECT_EQ(all.size(), 5);
    
    // Последние 5 элементов: 995, 996, 997, 998, 999
    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(all[i], 995 + i);
    }
}
