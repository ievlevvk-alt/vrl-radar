// tools/migrate_config.cpp
#include "vrl/radar/core/config_loader.hpp"
#include "vrl/radar/core/config.h"
#include "vrl/radar/utils/logger.h"
#include <iostream>

using namespace vrl::radar;
using namespace vrl::radar::utils;

int main(int argc, char* argv[]) {
    auto& logger = Logger::instance();
    logger.set_level(LogLevel::INFO);
    logger.set_console_output(true);
    
    VRL_LOG_INFO(modules::MAIN, "=== Config Migration Tool ===");
    
    std::string old_config = "../config/radar.conf";
    std::string new_config = "../config/radar.json";
    
    if (argc > 1) old_config = argv[1];
    if (argc > 2) new_config = argv[2];
    
    VRL_LOG_INFO(modules::MAIN, "Reading old config: " + old_config);
    
    // Используем старый парсер
    ConfigParser parser;
    if (!parser.load(old_config)) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to load old config");
        return 1;
    }
    
    SystemConfig config = ConfigBuilder::build(parser);
    
    VRL_LOG_INFO(modules::MAIN, "Saving new config: " + new_config);
    
    // Используем новый загрузчик для сохранения
    ConfigLoader loader;
    if (!loader.save(config, new_config)) {
        VRL_LOG_ERROR(modules::MAIN, "Failed to save new config");
        return 1;
    }
    
    VRL_LOG_INFO(modules::MAIN, "Migration completed successfully!");
    VRL_LOG_INFO(modules::MAIN, "New config saved to: " + new_config);
    
    return 0;
}
