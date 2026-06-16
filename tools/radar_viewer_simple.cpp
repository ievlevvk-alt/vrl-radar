// file: tools/radar_viewer_simple.cpp
#include "radar/radar_system.h"
#include <SDL2/SDL.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>
#include <iomanip>

using namespace radar;

class SimpleRadarViewer {
public:
    SimpleRadarViewer(int width = 1024, int height = 768) 
        : width_(width), height_(height), running_(true) {
        
        center_x_ = width / 2;
        center_y_ = height / 2;
        max_range_m_ = 3000.0;
        scale_ = static_cast<double>(std::min(width, height) - 100) / (max_range_m_ * 2);
        
        if (SDL_Init(SDL_INIT_VIDEO) < 0) return;
        
        window_ = SDL_CreateWindow("Radar Display", 
                                   SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                   width_, height_, SDL_WINDOW_SHOWN);
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    }
    
    ~SimpleRadarViewer() {
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_) SDL_DestroyWindow(window_);
        SDL_Quit();
    }
    
    void update(const std::vector<TargetReport>& targets, 
                const std::vector<Track>& tracks,
                const ScanReplies& scan, 
                int revolution) {
        targets_ = targets;
        tracks_ = tracks;
        current_scan_ = scan;
        current_revolution_ = revolution;
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        SDL_RenderClear(renderer_);
        
        draw_grid();
        draw_raw_replies();
        draw_scan_line();
        draw_targets();
        draw_tracks();
        
        SDL_RenderPresent(renderer_);
    }
    
    bool handle_events() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) return false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) return false;
            if (event.type == SDL_MOUSEMOTION) {
                // Выводим координаты мыши в консоль
                int dx = event.motion.x - center_x_;
                int dy = center_y_ - event.motion.y;
                double range = std::sqrt(dx*dx + dy*dy) / scale_;
                double az = std::atan2(dx, dy) * 180.0 / M_PI;
                if (az < 0) az += 360.0;
                
                std::cout << "\rMouse: range=" << std::fixed << std::setprecision(0) << range 
                          << "m, az=" << std::setprecision(1) << az << "°      " << std::flush;
            }
        }
        return true;
    }
    
    bool is_running() const { return running_; }
    
private:
    void draw_grid() {
        SDL_SetRenderDrawColor(renderer_, 40, 40, 40, 255);
        for (int r = 500; r <= 3000; r += 500) {
            int radius = static_cast<int>(r * scale_);
            draw_circle(center_x_, center_y_, radius);
        }
        
        for (int az = 0; az < 360; az += 30) {
            double rad = az * M_PI / 180.0;
            int x2 = center_x_ + static_cast<int>(max_range_m_ * scale_ * sin(rad));
            int y2 = center_y_ - static_cast<int>(max_range_m_ * scale_ * cos(rad));
            SDL_RenderDrawLine(renderer_, center_x_, center_y_, x2, y2);
        }
    }
    
    void draw_raw_replies() {
        for (const auto& reply : current_scan_.rbs_replies) {
            double rad = reply.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
            double range_m = reply.range * 30.0;
            int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
            int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
            
            SDL_SetRenderDrawColor(renderer_, 0, 100, 255, 255);
            SDL_Rect rect = {x-2, y-2, 4, 4};
            SDL_RenderFillRect(renderer_, &rect);
        }
        
        for (const auto& reply : current_scan_.uvd_replies) {
            double rad = reply.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
            double range_m = reply.range * 50.0;
            int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
            int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
            
            SDL_SetRenderDrawColor(renderer_, 255, 0, 0, 255);
            SDL_Rect rect = {x-2, y-2, 4, 4};
            SDL_RenderFillRect(renderer_, &rect);
        }
    }
    
    void draw_scan_line() {
        double rad = current_scan_.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
        int x2 = center_x_ + static_cast<int>(max_range_m_ * scale_ * sin(rad));
        int y2 = center_y_ - static_cast<int>(max_range_m_ * scale_ * cos(rad));
        
        SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 200);
        SDL_RenderDrawLine(renderer_, center_x_, center_y_, x2, y2);
        draw_circle(x2, y2, 5);
    }
    
    void draw_targets() {
        for (const auto& target : targets_) {
            int x = center_x_ + static_cast<int>(target.x * scale_);
            int y = center_y_ - static_cast<int>(target.y * scale_);
            
            SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 255);
            draw_circle(x, y, 8);
            SDL_SetRenderDrawColor(renderer_, 0, 200, 200, 255);
            draw_filled_circle(x, y, 4);
        }
    }
    
    void draw_tracks() {
        for (const auto& track : tracks_) {
            int x = center_x_ + static_cast<int>(track.x * scale_);
            int y = center_y_ - static_cast<int>(track.y * scale_);
            
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
            draw_circle(x, y, 12);
            draw_filled_circle(x, y, 5);
        }
    }
    
    void draw_circle(int cx, int cy, int r) {
        for (int y = -r; y <= r; y++)
            for (int x = -r; x <= r; x++)
                if (x*x + y*y <= r*r)
                    SDL_RenderDrawPoint(renderer_, cx + x, cy + y);
    }
    
    void draw_filled_circle(int cx, int cy, int r) {
        for (int y = -r; y <= r; y++)
            for (int x = -r; x <= r; x++)
                if (x*x + y*y <= r*r)
                    SDL_RenderDrawPoint(renderer_, cx + x, cy + y);
    }
    
    int width_, height_;
    int center_x_, center_y_;
    double scale_;
    double max_range_m_;
    bool running_;
    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    
    std::vector<TargetReport> targets_;
    std::vector<Track> tracks_;
    ScanReplies current_scan_;
    int current_revolution_{0};
};

int main() {
    std::cout << "=== Radar Viewer ===\n";
    std::cout << "Move mouse - coordinates in console\n";
    std::cout << "ESC - exit\n\n";
    
    auto config = SystemConfig::load_from_file("radar.conf");
    RadarSystem system(config);
    
    if (!system.initialize()) return 1;
    
    SimpleRadarViewer viewer(1024, 768);
    
    std::vector<TargetReport> all_targets;
    std::vector<Track> all_tracks;
    ScanReplies last_scan;
    int last_revolution = 0;
    
    system.set_target_callback([&](const TargetReport& target) {
        all_targets.push_back(target);
        if (all_targets.size() > 50) all_targets.erase(all_targets.begin());
    });
    
    int scan_num = 0;
    auto last_time = std::chrono::steady_clock::now();
    
    while (viewer.handle_events()) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count() >= 10) {
            uint16_t azimuth = (scan_num * 4) % 4096;
            auto scan = system.generate_test_scan(azimuth, 0);
            system.process_scan(scan);
            
            last_scan = scan;
            last_revolution = scan_num / 1024;
            scan_num++;
            last_time = now;
            
            all_tracks = system.get_tracks();
        }
        
        viewer.update(all_targets, all_tracks, last_scan, last_revolution);
        viewer.render();
        SDL_Delay(10);
    }
    
    system.shutdown();
    std::cout << "\n=== Viewer Stopped ===\n";
    return 0;
}
