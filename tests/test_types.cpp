#include "radar/replies.h"
#include "radar/utils.h"
#include <iostream>
#include <cassert>

using namespace radar;

void test_rbs_reply() {
    std::cout << "Testing RBSReply..." << std::endl;
    
    RBSReply reply;
    reply.azimuth = 1024;
    reply.range = 500;
    reply.code12 = 0x123;
    
    // Заполняем эфирные амплитуды
    reply.ether_amplitudes[0] = 200;  // F1
    reply.ether_amplitudes[1] = 190;  // F2
    reply.ether_amplitudes[2] = 180;  // бит 0
    
    assert(reply.f1() == 200);
    assert(reply.f2() == 190);
    assert(reply.bit(0) == 180);
    
    // Проверяем SLS канал
    reply.ether_amplitudes_sls[0] = 50;
    assert(reply.f1_sls() == 50);
    
    std::cout << "RBSReply test passed!" << std::endl;
}

void test_uvd_reply() {
    std::cout << "Testing UVDReply..." << std::endl;
    
    UVDReply reply;
    reply.azimuth = 2048;
    reply.range = 1000;
    reply.data20 = 0x12345;
    
    // Заполняем эфирные амплитуды
    for (int i = 0; i < 80; ++i) {
        reply.ether_amplitudes[i] = i;
    }
    
    assert(reply.left_pulse(0, 0) == 0);      // i=0, repeat=0: позиция 0
    assert(reply.right_pulse(0, 0) == 1);     // позиция 1
    assert(reply.left_pulse(0, 1) == 40);     // repeat=1: позиция 40
    assert(reply.right_pulse(0, 1) == 41);    // позиция 41
    
    // SLS канал
    for (int i = 0; i < 80; ++i) {
        reply.ether_amplitudes_sls[i] = i + 100;
    }
    
    assert(reply.left_pulse_sls(0, 0) == 100);
    
    std::cout << "UVDReply test passed!" << std::endl;
}

void test_utils() {
    std::cout << "Testing utils..." << std::endl;
    
    RadarConfig cfg;
    cfg.max_azimuth_diff_for_overlap = 2;
    cfg.max_range_diff_for_overlap = 10;
    
    assert(utils::is_potential_overlap(100, 500, 101, 505, cfg) == true);
    assert(utils::is_potential_overlap(100, 500, 103, 500, cfg) == false);
    assert(utils::is_potential_overlap(4095, 500, 0, 500, cfg) == true);
    
    // Тест SLS detection
    RBSReply rbs;
    rbs.ether_amplitudes[0] = 200;
    rbs.ether_amplitudes_sls[0] = 200;  // равные амплитуды
    assert(utils::is_sidelobe(rbs, 3.0) == false);  // 0 дБ < 3 дБ
    
    // Используем значение, которое не вызывает переполнения
    rbs.ether_amplitudes_sls[0] = 200;  // оставляем 200
    // Но для проверки нам нужно отношение > 3 дБ
    // В функции is_sidelobe сравнивается отношение SLS/MAIN
    // Поэтому можно оставить MAIN меньше, а не SLS больше
    rbs.ether_amplitudes[0] = 100;      // уменьшаем MAIN
    rbs.ether_amplitudes_sls[0] = 200;  // SLS в 2 раза сильнее (6 дБ)
    assert(utils::is_sidelobe(rbs, 3.0) == true);   // 6 дБ > 3 дБ
    
    std::cout << "Utils test passed!" << std::endl;
}

int main() {
    std::cout << "Running type tests..." << std::endl;
    
    test_rbs_reply();
    test_uvd_reply();
    test_utils();
    
    std::cout << "All type tests passed!" << std::endl;
    return 0;
}