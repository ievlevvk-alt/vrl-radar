// file: tests/test_garbling_solver.cpp
#include "radar/garbling_solver.h"
#include "radar/simulator.h"
#include <iostream>
#include <cassert>
#include <iomanip>

using namespace radar;

void test_threshold_solver() {
    std::cout << "\n=== Testing Threshold Garbling Solver ===\n";
    
    RadarConfig cfg;
    cfg.min_amplitude = 10;
    
    ThresholdGarblingSolver solver(cfg, 50);
    
    // Создаем два перекрывающихся ответа
    SimulatorConfig sim_cfg;
    sim_cfg.radar = cfg;
    ReplySimulator sim(sim_cfg);
    
    auto reply1 = sim.generate_rbs(1000, 500, 0x123, false);
    auto reply2 = sim.generate_rbs(1000, 503, 0x456, false);
    
    std::vector<RBSReply> mixture = {reply1, reply2};
    
    auto result = solver.separate_rbs(mixture);
    
    std::cout << "Separated " << result.separated_replies.size() 
              << " replies (confidence=" << result.confidence << ")\n";
    
    for (const auto& r : result.separated_replies) {
        std::cout << "  Code: 0x" << std::hex << r.code12 << std::dec << "\n";
    }
    
    assert(!result.separated_replies.empty());
    std::cout << "Threshold solver test passed!\n";
}

void test_iterative_solver() {
    std::cout << "\n=== Testing Iterative Subtraction Solver ===\n";
    
    RadarConfig cfg;
    cfg.min_amplitude = 10;
    
    IterativeSubtractionSolver solver(cfg, 0.3, 5);
    
    SimulatorConfig sim_cfg;
    sim_cfg.radar = cfg;
    ReplySimulator sim(sim_cfg);
    
    // Создаем три перекрывающихся ответа с разными амплитудами
    auto strong = sim.generate_rbs(1000, 500, 0xAAA, false);
    auto medium = sim.generate_rbs(1000, 502, 0xBBB, false);
    auto weak = sim.generate_rbs(1000, 505, 0xCCC, false);
    
    // Усиливаем амплитуды для strong
    for (auto& amp : strong.ether_amplitudes) {
        if (amp > 0) amp = std::min(255, amp * 2);
    }
    
    std::vector<RBSReply> mixture = {strong, medium, weak};
    
    auto result = solver.separate_rbs(mixture);
    
    std::cout << "Iterations: separated " << result.separated_replies.size() 
              << " replies\n";
    
    for (const auto& r : result.separated_replies) {
        std::cout << "  Code: 0x" << std::hex << r.code12 << std::dec << "\n";
    }
    
    // Должны выделить хотя бы сильный ответ
    assert(result.separated_replies.size() >= 1);
    std::cout << "Iterative solver test passed!\n";
}

void test_composite_solver() {
    std::cout << "\n=== Testing Composite Solver ===\n";
    
    RadarConfig cfg;
    cfg.min_amplitude = 10;
    
    CompositeGarblingSolver solver(cfg);
    
    // Добавляем методы с разными приоритетами
    solver.add_method(
        std::make_unique<ThresholdGarblingSolver>(cfg, 50),
        1  // низкий приоритет
    );
    
    solver.add_method(
        std::make_unique<IterativeSubtractionSolver>(cfg, 0.3, 5),
        2  // средний приоритет
    );
    
    SimulatorConfig sim_cfg;
    sim_cfg.radar = cfg;
    ReplySimulator sim(sim_cfg);
    
    auto reply1 = sim.generate_rbs(1000, 500, 0x123, false);
    auto reply2 = sim.generate_rbs(1000, 503, 0x456, false);
    
    std::vector<RBSReply> mixture = {reply1, reply2};
    
    auto result = solver.separate_rbs(mixture);
    
    std::cout << "Composite solver: " << result.separated_replies.size() 
              << " replies, method=" << result.method_used << "\n";
    
    assert(!result.separated_replies.empty());
    std::cout << "Composite solver test passed!\n";
}

int main() {
    std::cout << "Running Garbling Solver tests...\n";
    
    test_threshold_solver();
    test_iterative_solver();
    test_composite_solver();
    
    std::cout << "\nAll garbling solver tests passed!\n";
    return 0;
}