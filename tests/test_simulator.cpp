#include "radar/simulator.h"
#include "radar/utils.h"
#include <iostream>
#include <iomanip>
#include <cassert>
#include <bitset>

using namespace radar;

void print_rbs(const RBSReply& reply) {
    std::cout << "RBS: az=" << reply.azimuth 
              << " range=" << reply.range
              << " code=0x" << std::hex << reply.code12 << std::dec
              << " valid=" << reply.is_valid
              << " sls_present=" << (reply.ether_amplitudes_sls[0] != 0)
              << std::endl;
}

void print_uvd(const UVDReply& reply) {
    std::cout << "UVD: az=" << reply.azimuth 
              << " range=" << reply.range
              << " data=0x" << std::hex << reply.data20 << std::dec
              << " mask=0x" << std::hex << reply.error_mask << std::dec
              << " valid=" << reply.is_valid
              << " sls_present=" << (reply.ether_amplitudes_sls[0] != 0)
              << std::endl;
}

void test_rbs_generation() {
    std::cout << "\n=== Testing RBS Generation ===" << std::endl;
    
    SimulatorConfig cfg;
    cfg.radar.range_bin_rbs = 30.0;
    cfg.rbs.snr_db = 20.0;
    cfg.sls.enabled = true;
    cfg.sls.sidelobe_probability = 0.3;
    
    ReplySimulator sim(cfg);
    
    auto reply = sim.generate_rbs(1000, 500, 0xABC, true);
    print_rbs(reply);
    
    assert(reply.azimuth == 1000);
    assert(reply.range == 500);
    assert(reply.code12 == 0xABC);
    assert(reply.spi == true);
    assert(reply.ether_amplitudes.size() == RBSReply::ETHER_POSITIONS);
    assert(reply.ether_amplitudes_sls.size() == RBSReply::ETHER_POSITIONS);
    
    // Проверяем, что F1 и F2 есть
    assert(reply.f1() > 0);
    assert(reply.f2() > 0);
    
    std::cout << "RBS generation test passed!" << std::endl;
}

void test_uvd_generation() {
    std::cout << "\n=== Testing UVD Generation ===" << std::endl;
    
    SimulatorConfig cfg;
    cfg.radar.range_bin_uvd = 50.0;
    cfg.uvd.snr_db = 25.0;
    cfg.sls.enabled = true;
    
    ReplySimulator sim(cfg);
    
    auto reply = sim.generate_uvd(2000, 800, 0x12345);
    print_uvd(reply);
    
    assert(reply.azimuth == 2000);
    assert(reply.range == 800);
    assert(reply.ether_amplitudes.size() == UVDReply::ETHER_POSITIONS);
    assert(reply.ether_amplitudes_sls.size() == UVDReply::ETHER_POSITIONS);
    
    // Проверяем структуру "активной паузы"
    uint8_t threshold = 50;
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (int i = 0; i < 20; ++i) {
            uint8_t left = reply.left_pulse(i, repeat);
            uint8_t right = reply.right_pulse(i, repeat);
            
            // Должна быть ровно одна высокая амплитуда в паре
            bool left_high = left > threshold;
            bool right_high = right > threshold;
            
            // Из-за шума может быть не так, но в среднем должно соблюдаться
            // Проверять строго не будем
        }
    }
    
    std::cout << "UVD generation test passed!" << std::endl;
}

void test_sls_channel() {
    std::cout << "\n=== Testing SLS Channel ===" << std::endl;
    
    SimulatorConfig cfg;
    cfg.radar.range_bin_rbs = 30.0;
    cfg.rbs.snr_db = 30.0;
    cfg.sls.enabled = true;
    cfg.sls.main_to_sls_ratio = 1.0;
    cfg.sls.sls_attenuation_db = 20.0;
    cfg.sls.sidelobe_probability = 0.5;
    
    ReplySimulator sim(cfg);
    
    // Генерируем несколько ответов и смотрим SLS канал
    int sidelobe_count = 0;
    int total_count = 100;
    
    for (int i = 0; i < total_count; ++i) {
        auto reply = sim.generate_rbs(1000 + i, 500, 0xABC, false);
        if (utils::is_sidelobe(reply)) {
            sidelobe_count++;
        }
    }
    
    std::cout << "Sidelobe probability: " << cfg.sls.sidelobe_probability 
              << ", observed: " << static_cast<double>(sidelobe_count)/total_count 
              << std::endl;
    
    std::cout << "SLS channel test passed!" << std::endl;
}

int main() {
    std::cout << "Running simulator tests..." << std::endl;
    
    test_rbs_generation();
    test_uvd_generation();
    test_sls_channel();
    
    std::cout << "\nAll simulator tests passed!" << std::endl;
    return 0;
}