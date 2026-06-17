// tools/test_config_parser.cpp
#include "radar/config_parser.h"
#include "radar/radar_system.h"  // Добавляем этот include
#include <iostream>

int main(int argc, char* argv[]) {
    std::string filename = "radar.conf";
    if (argc > 1) filename = argv[1];
    
    radar::ConfigParser parser;
    if (!parser.load(filename)) {
        std::cerr << "Failed to load config: " << filename << "\n";
        return 1;
    }
    
    std::cout << "=== Config loaded successfully ===\n";
    std::cout << "Sections:\n";
    for (const auto& section : parser.get_sections()) {
        std::cout << "  [" << section << "]\n";
        for (const auto& key : parser.get_keys(section)) {
            auto val = parser.get_string(key, section);
            std::cout << "    " << key << " = " << val.value_or("") << "\n";
        }
    }
    
    // Строим конфигурацию
    auto config = radar::ConfigBuilder::build(parser);
    
    std::cout << "\n=== Built SystemConfig ===\n";
    std::cout << "Range bin RBS: " << config.radar.range_bin_rbs << "\n";
    std::cout << "Range bin UVD: " << config.radar.range_bin_uvd << "\n";
    std::cout << "Beamwidth: " << config.beamwidth_deg << "°\n";
    std::cout << "RBS targets: " << config.rbs_targets.size() << "\n";
    for (const auto& t : config.rbs_targets) {
        std::cout << "  " << t.name << " at " << t.azimuth_deg << "°, "
                  << t.range_km << " km";
        if (t.use_linear_motion) {
            std::cout << ", moving at " << t.speed_m_per_s << " m/s";
        }
        std::cout << "\n";
    }
    std::cout << "UVD targets: " << config.uvd_targets.size() << "\n";
    
    return 0;
}
