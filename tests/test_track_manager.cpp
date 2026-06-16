// file: tests/test_track_manager.cpp
#include "radar/track_manager.h"
#include "radar/simulator.h"
#include <iostream>
#include <cassert>
#include <chrono>

using namespace radar;

void test_single_track() {
    std::cout << "\n=== Test Single Track ===\n";
    
    TrackerConfig config;
    config.min_hits_to_confirm = 3;
    config.max_coast_count = 5;
    config.max_gate_distance = 50.0;
    
    TrackManager manager(config);
    
    std::vector<TargetReport> targets;
    uint32_t current_time = 1000;
    
    // Создаем цель, движущуюся по прямой
    for (int i = 0; i < 10; ++i) {
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = 1000 + i * 10;
        report.y = 2000 + i * 5;
        report.rbs.mode3a_code = 0x123;
        
        targets.push_back(report);
        manager.process_targets({report}, current_time);
        current_time += 1000;
    }
    
    auto tracks = manager.get_confirmed_tracks();
    assert(tracks.size() >= 1);
    assert(tracks[0].ground_speed > 0);
    assert(tracks[0].hit_count >= 3);
    
    std::cout << "Track ID: " << tracks[0].id << "\n";
    std::cout << "Position: (" << tracks[0].x << ", " << tracks[0].y << ")\n";
    std::cout << "Speed: " << tracks[0].ground_speed << " m/s\n";
    std::cout << "Course: " << tracks[0].course_deg << " deg\n";
    std::cout << "Hits: " << tracks[0].hit_count << "\n";
    std::cout << "Confidence: " << tracks[0].confidence << "\n";
    
    std::cout << "Single track test passed!\n";
}

void test_track_coasting() {
    std::cout << "\n=== Test Track Coasting ===\n";
    
    TrackerConfig config;
    config.min_hits_to_confirm = 2;
    config.max_coast_count = 2;
    
    TrackManager manager(config);
    
    uint32_t current_time = 1000;
    
    // Создаем трек с 3 попаданиями
    for (int i = 0; i < 3; ++i) {
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = 1000;
        report.y = 2000;
        report.rbs.mode3a_code = 0x123;
        manager.process_targets({report}, current_time);
        current_time += 1000;
    }
    
    // Затем пропускаем несколько оборотов
    for (int i = 0; i < 5; ++i) {
        manager.process_targets({}, current_time);
        current_time += 1000;
    }
    
    auto tracks = manager.get_active_tracks();
    // Трек должен быть сброшен после coast_count = 2
    assert(tracks.size() == 0);
    
    std::cout << "Track coasting test passed!\n";
}

void test_multiple_tracks() {
    std::cout << "\n=== Test Multiple Tracks ===\n";
    
    TrackerConfig config;
    config.min_hits_to_confirm = 2;
    config.max_coast_count = 5;
    config.max_gate_distance = 100.0;
    
    TrackManager manager(config);
    
    uint32_t current_time = 1000;
    
    // Цель 1: движется вправо
    for (int i = 0; i < 5; ++i) {
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = 1000 + i * 20;
        report.y = 2000;
        report.rbs.mode3a_code = 0x111;
        manager.process_targets({report}, current_time);
        current_time += 1000;
    }
    
    // Цель 2: движется вверх
    for (int i = 0; i < 5; ++i) {
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = 3000;
        report.y = 1000 + i * 15;
        report.rbs.mode3a_code = 0x222;
        manager.process_targets({report}, current_time);
        current_time += 1000;
    }
    
    auto tracks = manager.get_confirmed_tracks();
    assert(tracks.size() == 2);
    
    // Проверяем, что треки имеют разные коды
    bool has_code_111 = false;
    bool has_code_222 = false;
    
    for (const auto& track : tracks) {
        if (track.mode3a_code == 0x111) has_code_111 = true;
        if (track.mode3a_code == 0x222) has_code_222 = true;
        std::cout << "Track " << track.id << ": code=0x" << std::hex << track.mode3a_code 
                  << std::dec << ", speed=" << track.ground_speed << " m/s\n";
    }
    
    assert(has_code_111 && has_code_222);
    
    std::cout << "Multiple tracks test passed!\n";
}

void test_track_prediction() {
    std::cout << "\n=== Test Track Prediction ===\n";
    
    TrackerConfig config;
    config.min_hits_to_confirm = 2;
    config.max_coast_count = 10;
    
    TrackManager manager(config);
    
    uint32_t current_time = 1000;
    std::vector<std::pair<double, double>> positions;
    
    // Создаем трек с равномерным движением
    for (int i = 0; i < 3; ++i) {
        TargetReport report;
        report.type = TargetReport::SourceType::RBS;
        report.x = 1000 + i * 10;
        report.y = 2000 + i * 10;
        report.rbs.mode3a_code = 0x333;
        positions.emplace_back(report.x, report.y);
        manager.process_targets({report}, current_time);
        current_time += 1000;
    }
    
    // Пропускаем один оборот и получаем предсказание
    current_time += 5000;
    manager.process_targets({}, current_time);
    
    auto tracks = manager.get_active_tracks();
    assert(tracks.size() == 1);
    
    // Предсказанная позиция должна быть дальше
    double dx = tracks[0].x - positions.back().first;
    double dy = tracks[0].y - positions.back().second;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // За 5 секунд при скорости ~14 м/с должно пройти ~70 метров
    assert(distance > 40 && distance < 100);
    
    std::cout << "Track " << tracks[0].id << " predicted position: ("
              << tracks[0].x << ", " << tracks[0].y << ")\n";
    std::cout << "Distance from last known: " << distance << " m\n";
    std::cout << "Prediction test passed!\n";
}

int main() {
    std::cout << "Running Track Manager tests...\n";
    
    test_single_track();
    test_track_coasting();
    test_multiple_tracks();
    test_track_prediction();
    
    std::cout << "\nAll track manager tests passed!\n";
    return 0;
}
