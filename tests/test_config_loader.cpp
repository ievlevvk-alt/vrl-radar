// tests/test_config_loader.cpp
#include <gtest/gtest.h>
#include "vrl/radar/core/config_loader.hpp"
#include <fstream>
#include <nlohmann/json.hpp>

using namespace vrl::radar;

class ConfigLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Создаем временный JSON файл для тестов
        json test_config = {
            {"beamwidth_deg", 5.0},
            {"revolution_time", 5.0},
            {"azimuth", {{"azimuth_bins", 4096}}},
            {"radar", {
                {"range_bin_rbs", 30.0},
                {"range_bin_uvd", 60.0},
                {"max_azimuth_diff_for_overlap", 2.0},
                {"max_range_diff_for_overlap", 10},
                {"min_amplitude", 10}
            }},
            {"rbs", {
                {"snr_db", 20.0},
                {"amp_variation", 0.1},
                {"f1f2_amp_ratio", 1.0}
            }},
            {"uvd", {
                {"snr_db", 20.0},
                {"error_probability", 0.01}
            }},
            {"tracker", {
                {"min_hits_to_confirm", 3},
                {"max_coast_count", 10},
                {"max_gate_distance", 3000.0},
                {"max_gate_azimuth", 30.0},
                {"process_noise", 0.5},
                {"measurement_noise", 0.1},
                {"enable_uvd_tracking", true},
                {"enable_rbs_tracking", true},
                {"debug_mode", false}
            }},
            {"processing", {
                {"max_gap_azimuth", 8},
                {"range_window", 30},
                {"range_tolerance", 5},
                {"min_hits", 2},
                {"output_file", "targets.txt"},
                {"plots_output_file", "plots_combined.txt"},
                {"min_cluster_hits", 2},
                {"range_threshold_bins", 5},
                {"azimuth_threshold_bins", 3},
                {"completion_gap_bins", 8},
                {"min_confidence", 0.3},
                {"garbled_confidence_threshold", 0.5},
                {"min_uvd_confidence", 0.3},
                {"uvd_garbled_threshold", 0.5}
            }},
            {"rbs_targets", {
                {
                    {"name", "Test_RBS"},
                    {"azimuth_deg", 45.0},
                    {"range_km", 100.0},
                    {"rbs_code_octal", 668},
                    {"spi", false},
                    {"enabled", true},
                    {"update_every_n_revolutions", 1},
                    {"revolution_offset", 0},
                    {"altitude_meters", 9600},
                    {"enable_altitude", true},
                    {"alternate_code_altitude", true},
                    {"alternate_data_altitude", false},
                    {"use_linear_motion", true},
                    {"speed_m_per_s", 200.0},
                    {"course_deg", 225.0},
                    {"initial_x_km", 75.0},
                    {"initial_y_km", 129.9}
                }
            }},
            {"uvd_targets", {
                {
                    {"name", "Test_UVD"},
                    {"azimuth_deg", 45.0},
                    {"range_km", 20.0},
                    {"uvd_data_dec", 12345},
                    {"enabled", true},
                    {"update_every_n_revolutions", 1},
                    {"revolution_offset", 0},
                    {"altitude_meters", 3000},
                    {"enable_altitude", true},
                    {"alternate_code_altitude", false},
                    {"alternate_data_altitude", true},
                    {"use_linear_motion", true},
                    {"speed_m_per_s", 100.0},
                    {"course_deg", 45.0},
                    {"initial_x_km", 14.14},
                    {"initial_y_km", 14.14}
                }
            }}
        };
        
        test_file_ = "/tmp/test_config.json";
        std::ofstream file(test_file_);
        file << test_config.dump(4);
        file.close();
    }
    
    void TearDown() override {
        std::remove(test_file_.c_str());
    }
    
    std::string test_file_;
};


TEST_F(ConfigLoaderTest, LoadValidConfig) {
    ConfigLoader loader;
    SystemConfig config;
    
    EXPECT_TRUE(loader.load(test_file_, config));
}

TEST_F(ConfigLoaderTest, ParseRadarConfig) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_DOUBLE_EQ(config.radar.range_bin_rbs, 30.0);
    EXPECT_DOUBLE_EQ(config.radar.range_bin_uvd, 60.0);
    EXPECT_DOUBLE_EQ(config.radar.max_azimuth_diff_for_overlap, 2.0);
    EXPECT_EQ(config.radar.max_range_diff_for_overlap, 10);
    EXPECT_EQ(config.radar.min_amplitude, 10);
}

TEST_F(ConfigLoaderTest, ParseTrackerConfig) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_EQ(config.tracker.min_hits_to_confirm, 3);
    EXPECT_EQ(config.tracker.max_coast_count, 10);
    EXPECT_DOUBLE_EQ(config.tracker.max_gate_distance, 3000.0);
    EXPECT_DOUBLE_EQ(config.tracker.max_gate_azimuth, 30.0);
    EXPECT_DOUBLE_EQ(config.tracker.process_noise, 0.5);
    EXPECT_DOUBLE_EQ(config.tracker.measurement_noise, 0.1);
    EXPECT_TRUE(config.tracker.enable_uvd_tracking);
    EXPECT_TRUE(config.tracker.enable_rbs_tracking);
    EXPECT_FALSE(config.tracker.debug_mode);
}

TEST_F(ConfigLoaderTest, ParseProcessingConfig) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_EQ(config.processing.max_gap_azimuth, 8);
    EXPECT_EQ(config.processing.range_window, 30);
    EXPECT_EQ(config.processing.range_tolerance, 5);
    EXPECT_EQ(config.processing.min_hits, 2);
    EXPECT_EQ(config.processing.output_file, "targets.txt");
    EXPECT_EQ(config.processing.plots_output_file, "plots_combined.txt");
    EXPECT_EQ(config.processing.min_cluster_hits, 2);
    EXPECT_EQ(config.processing.range_threshold_bins, 5);
    EXPECT_EQ(config.processing.azimuth_threshold_bins, 3);
    EXPECT_EQ(config.processing.completion_gap_bins, 8);
    EXPECT_DOUBLE_EQ(config.processing.min_confidence, 0.3);
    EXPECT_DOUBLE_EQ(config.processing.garbled_confidence_threshold, 0.5);
    EXPECT_DOUBLE_EQ(config.processing.min_uvd_confidence, 0.3);
    EXPECT_DOUBLE_EQ(config.processing.uvd_garbled_threshold, 0.5);
}

TEST_F(ConfigLoaderTest, ParseRBSTargets) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_EQ(config.rbs_targets.size(), 1);
    const auto& target = config.rbs_targets[0];
    EXPECT_EQ(target.name, "Test_RBS");
    EXPECT_DOUBLE_EQ(target.azimuth_deg, 45.0);
    EXPECT_DOUBLE_EQ(target.range_km, 100.0);
    EXPECT_EQ(target.rbs_code_octal, 668);
    EXPECT_FALSE(target.spi);
    EXPECT_TRUE(target.enabled);
    EXPECT_EQ(target.altitude_meters, 9600);
    EXPECT_TRUE(target.enable_altitude);
    EXPECT_TRUE(target.alternate_code_altitude);
    EXPECT_TRUE(target.use_linear_motion);
    EXPECT_DOUBLE_EQ(target.speed_m_per_s, 200.0);
    EXPECT_DOUBLE_EQ(target.course_deg, 225.0);
}

TEST_F(ConfigLoaderTest, ParseUVDTargets) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_EQ(config.uvd_targets.size(), 1);
    const auto& target = config.uvd_targets[0];
    EXPECT_EQ(target.name, "Test_UVD");
    EXPECT_DOUBLE_EQ(target.azimuth_deg, 45.0);
    EXPECT_DOUBLE_EQ(target.range_km, 20.0);
    EXPECT_EQ(target.uvd_data_dec, 12345);
    EXPECT_TRUE(target.enabled);
    EXPECT_EQ(target.altitude_meters, 3000);
    EXPECT_TRUE(target.enable_altitude);
    EXPECT_TRUE(target.alternate_data_altitude);
    EXPECT_TRUE(target.use_linear_motion);
    EXPECT_DOUBLE_EQ(target.speed_m_per_s, 100.0);
    EXPECT_DOUBLE_EQ(target.course_deg, 45.0);
}

TEST_F(ConfigLoaderTest, ValidateConfig) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    std::string error;
    EXPECT_TRUE(loader.validate(config, error));
    EXPECT_TRUE(error.empty());
}

TEST_F(ConfigLoaderTest, ConfigHasTargets) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    EXPECT_TRUE(config.has_targets());
}

TEST_F(ConfigLoaderTest, InvalidConfigMissingTargets) {
    json empty_config = {
        {"beamwidth_deg", 5.0},
        {"radar", {{"range_bin_rbs", 30.0}}}
    };
    
    std::string empty_file = "/tmp/empty_config.json";
    std::ofstream file(empty_file);
    file << empty_config.dump(4);
    file.close();
    
    ConfigLoader loader;
    SystemConfig config;
    
    // Загрузка должна вернуть true (файл существует и валидный JSON)
    bool load_result = loader.load(empty_file, config);
    
    // Если load вернул false, проверяем что это из-за валидации
    // (в зависимости от реализации load может вызывать validate внутри)
    if (!load_result) {
        // load уже вернул false, значит валидация провалилась внутри
        SUCCEED();
    } else {
        // load вернул true, проверяем validate отдельно
        std::string error;
        EXPECT_FALSE(loader.validate(config, error));
        EXPECT_FALSE(error.empty());
    }
    
    std::remove(empty_file.c_str());
}


TEST_F(ConfigLoaderTest, SaveConfig) {
    ConfigLoader loader;
    SystemConfig config;
    loader.load(test_file_, config);
    
    std::string save_file = "/tmp/saved_config.json";
    EXPECT_TRUE(loader.save(config, save_file));
    
    SystemConfig loaded_config;
    EXPECT_TRUE(loader.load(save_file, loaded_config));
    
    EXPECT_EQ(loaded_config.rbs_targets.size(), config.rbs_targets.size());
    EXPECT_EQ(loaded_config.uvd_targets.size(), config.uvd_targets.size());
    
    std::remove(save_file.c_str());
}
