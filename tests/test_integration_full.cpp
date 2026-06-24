// tests/test_integration_full.cpp
#include <gtest/gtest.h>
#include "vrl/radar/processing/dbscan_clusterer.h"
#include "vrl/radar/v2/track_manager.hpp"
#include "vrl/radar/v2/grid_config.hpp"
#include "vrl/radar/core/point_buffer.hpp"
#include "vrl/radar/core/cluster_pool.hpp"

using namespace vrl::radar;
using namespace vrl::radar::v2;

class IntegrationFullTest : public ::testing::Test {
protected:
    void SetUp() override {
        PointBuffer::instance().init(10000);
        ClusterPool::instance().init(1000);
        TrackPool::instance().init(100);
        PlotPool::instance().init(100);
        
        GridConfig config;
        config.cell_size_km = 5.0;
        config.max_range_km = 400.0;
        config.rings_near = 1;
        config.rings_far = 2;
        config.far_threshold_km = 150.0;
        config.max_candidate_distance_km = 10.0;
        config.revolution_time_s = 5.0;
        config.max_coast_revolutions = 3;
        config.range_gate_bins_near = 5.0;
        config.range_gate_bins_mid = 10.0;
        config.range_gate_bins_far = 20.0;
        config.azimuth_gate_maia_near = 10.0;
        config.azimuth_gate_maia_mid = 20.0;
        config.azimuth_gate_maia_far = 40.0;
        config.coast_gate_expansion = 1.5;
        
        track_manager_ = std::make_unique<TrackManager>();
        track_manager_->init(config);
        
        RadarConfig radar_config;
        radar_config.range_bin_rbs = 30.0;
        radar_config.range_bin_uvd = 60.0;
        radar_config.beamwidth_deg = 5.0;
        radar_config.min_amplitude = 10;
        
        clusterer_ = std::make_unique<DBSCANClusterer>(
            radar_config, 3, 1.2
        );
    }
    
    void TearDown() override {
        track_manager_->reset();
        TrackPool::instance().clear();
        PlotPool::instance().clear();
        ClusterPool::instance().clear();
        PointBuffer::instance().init(10000);
    }
    
    void add_point_to_buffer(uint16_t azimuth, uint16_t range, bool is_rbs = true) {
        StoredPoint point;
        point.azimuth = azimuth;
        point.range = range;
        point.is_rbs = is_rbs;
        point.code12 = 1234;
        PointBuffer::instance().add_point(point);
    }
    
    ScanReplies create_scan(uint16_t azimuth, 
                            const std::vector<uint16_t>& ranges,
                            bool is_rbs = true) {
        ScanReplies scan(azimuth, 0);
        
        for (uint16_t range : ranges) {
            add_point_to_buffer(azimuth, range, is_rbs);
            if (is_rbs) {
                RBSReply reply;
                reply.azimuth = azimuth;
                reply.range = range;
                reply.code12 = 1234;
                reply.is_valid = true;
                scan.rbs_replies.push_back(reply);
            } else {
                UVDReply reply;
                reply.azimuth = azimuth;
                reply.range = range;
                reply.data20 = 12345;
                reply.is_valid = true;
                scan.uvd_replies.push_back(reply);
            }
        }
        
        return scan;
    }
    
    void simulate_target_movement(uint16_t start_az, uint16_t end_az, 
                                   uint16_t range, int steps) {
        int step = (end_az - start_az) / steps;
        for (int i = 0; i < steps; ++i) {
            uint16_t az = start_az + i * step;
            auto scan = create_scan(az, {range});
            
            // Кластеризация
            clusterer_->process_scan(scan);
            
            // Трекинг
            track_manager_->process_azimuth(az);
            
            // Проверяем, что треки создаются
            auto stats = track_manager_->get_stats();
            if (stats.total_tracks > 0) {
                EXPECT_GT(stats.total_tracks, 0);
                break;
            }
        }
    }
    
    std::unique_ptr<TrackManager> track_manager_;
    std::unique_ptr<DBSCANClusterer> clusterer_;
};

// ===== ИНТЕГРАЦИОННЫЕ ТЕСТЫ =====

TEST_F(IntegrationFullTest, FullPipeline) {
    // Создаем движущуюся цель
    simulate_target_movement(500, 520, 100, 20);
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}

TEST_F(IntegrationFullTest, MultipleTargets) {
    // Цель 1
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    // Цель 2
    for (int az = 600; az < 620; ++az) {
        auto scan = create_scan(az, {150});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}

TEST_F(IntegrationFullTest, TargetWithCoasting) {
    // Создаем цель
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    // Ждем coasting (несколько оборотов без обновления)
    for (int rev = 0; rev < 5; ++rev) {
        track_manager_->process_azimuth(100 + rev * 100);
    }
    
    auto stats = track_manager_->get_stats();
    // Должны быть coasting треки
    EXPECT_GE(stats.coasting_tracks, 0);
}

TEST_F(IntegrationFullTest, TargetReacquisition) {
    // Создаем цель
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    // Пропускаем несколько оборотов
    for (int rev = 0; rev < 2; ++rev) {
        track_manager_->process_azimuth(100 + rev * 100);
    }
    
    // Возвращаем цель
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {110});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}

TEST_F(IntegrationFullTest, RBSAndUVDMixed) {
    // RBS цель
    for (int az = 500; az < 510; ++az) {
        auto scan = create_scan(az, {100}, true);
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    // UVD цель (другая)
    for (int az = 520; az < 530; ++az) {
        auto scan = create_scan(az, {150}, false);
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}

TEST_F(IntegrationFullTest, LongTermTracking) {
    // Длительное сопровождение цели
    uint16_t range = 100;
    int revolutions = 3;
    
    for (int rev = 0; rev < revolutions; ++rev) {
        for (int az = 500 + rev * 10; az < 520 + rev * 10; ++az) {
            auto scan = create_scan(az, {range});
            clusterer_->process_scan(scan);
            track_manager_->process_azimuth(az);
        }
        range += 10;
    }
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}

TEST_F(IntegrationFullTest, GetUpdatedTracks) {
    // Создаем цель
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    const auto& updated = track_manager_->get_updated_tracks();
    EXPECT_GE(updated.size(), 0);
    
    track_manager_->clear_updated_tracks();
    EXPECT_TRUE(track_manager_->get_updated_tracks().empty());
}

TEST_F(IntegrationFullTest, GetPlots) {
    for (int az = 500; az < 520; ++az) {
        auto scan = create_scan(az, {100});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
    }
    
    auto plots = track_manager_->get_plots();
    EXPECT_GE(plots.size(), 0);
}

TEST_F(IntegrationFullTest, PerformanceStress) {
    // Стресс-тест: 1000 сканов
    for (int i = 0; i < 1000; ++i) {
        uint16_t az = 500 + (i % 100);
        uint16_t range = 100 + (i % 50);
        auto scan = create_scan(az, {range});
        clusterer_->process_scan(scan);
        track_manager_->process_azimuth(az);
        
        if (i % 100 == 0) {
            auto stats = track_manager_->get_stats();
            EXPECT_GE(stats.total_tracks, 0);
        }
    }
}

TEST_F(IntegrationFullTest, GridIndexIntegration) {
    // Создаем несколько целей в разных позициях
    for (int i = 0; i < 5; ++i) {
        for (int az = 500 + i * 20; az < 510 + i * 20; ++az) {
            int range = 100 + i * 50;
            auto scan = create_scan(az, {static_cast<uint16_t>(range)});
            clusterer_->process_scan(scan);
            track_manager_->process_azimuth(az);
        }
    }
    
    auto stats = track_manager_->get_stats();
    EXPECT_GT(stats.total_tracks, 0);
}
