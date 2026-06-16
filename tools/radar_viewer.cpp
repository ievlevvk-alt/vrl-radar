// file: tools/radar_viewer.cpp
#include "radar/radar_system.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace radar;

class RadarViewer {
public:
    RadarViewer(int width = 1280, int height = 960) 
        : width_(width), height_(height), running_(true),
          max_range_km_(200.0), min_range_km_(10.0) {
        
        center_x_ = width / 2;
        center_y_ = height / 2;
        current_range_km_ = max_range_km_;
        max_range_m_ = max_range_km_ * 1000.0;
        scale_ = static_cast<double>(std::min(width, height) - 100) / (max_range_m_ * 2);
        
        init_sdl();
    }
    
    ~RadarViewer() {
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_) SDL_DestroyWindow(window_);
        if (font_) TTF_CloseFont(font_);
        if (ttf_initialized_) TTF_Quit();
        SDL_Quit();
    }
    
    void update_targets(const std::vector<TargetReport>& targets) {
        targets_ = targets;
    }
    
    void update_tracks(const std::vector<Track>& tracks) {
        tracks_ = tracks;
    }
    
    void update_scan(const ScanReplies& scan, int revolution) {
        current_scan_ = scan;
        current_revolution_ = revolution;
        
        // НАКОПЛЕНИЕ ЗА ОБОРОТ
        if (revolution != last_revolution_) {
            accumulated_rbs_.clear();
            accumulated_uvd_.clear();
            last_revolution_ = revolution;
        }
        for (const auto& reply : scan.rbs_replies) {
            accumulated_rbs_.push_back(reply);
        }
        for (const auto& reply : scan.uvd_replies) {
            accumulated_uvd_.push_back(reply);
        }
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        SDL_RenderClear(renderer_);
        
        draw_grid();
        draw_accumulated_replies();  // НАКОПЛЕННЫЕ ОТВЕТЫ (ВМЕСТО draw_raw_replies)
        draw_scan_line();
        draw_targets();
        draw_tracks();
        draw_info();
        draw_mouse_coords();
        
        SDL_RenderPresent(renderer_);
    }
    
    bool is_running() const { return running_; }
    
    void handle_events() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running_ = false;
            } else if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) {
                    running_ = false;
                } else if (event.key.keysym.sym == SDLK_r) {
                    current_range_km_ = max_range_km_;
                    update_scale();
                } else if (event.key.keysym.sym == SDLK_c) {
                    accumulated_rbs_.clear();
                    accumulated_uvd_.clear();
                }
            } else if (event.type == SDL_MOUSEWHEEL) {
                if (event.wheel.y > 0) {
                    zoom_in();
                } else if (event.wheel.y < 0) {
                    zoom_out();
                }
            } else if (event.type == SDL_MOUSEMOTION) {
                int mouse_x = event.motion.x;
                int mouse_y = event.motion.y;
                
                double dx = mouse_x - center_x_;
                double dy = center_y_ - mouse_y;
                
                mouse_world_range_m_ = std::sqrt(dx*dx + dy*dy) / scale_;
                mouse_world_range_km_ = mouse_world_range_m_ / 1000.0;
                mouse_world_azimuth_ = std::atan2(dx, dy) * 180.0 / M_PI;
                if (mouse_world_azimuth_ < 0) mouse_world_azimuth_ += 360.0;
                
                mouse_world_x_km_ = mouse_world_range_km_ * std::sin(mouse_world_azimuth_ * M_PI / 180.0);
                mouse_world_y_km_ = mouse_world_range_km_ * std::cos(mouse_world_azimuth_ * M_PI / 180.0);
                has_mouse_pos_ = true;
            }
        }
    }
    
private:
    void init_sdl() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            return;
        }
        
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
        
        window_ = SDL_CreateWindow("Radar Display - Range: 200 km", 
                                   SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                   width_, height_, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        if (!window_) {
            std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
            return;
        }
        
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
        if (!renderer_) {
            std::cerr << "Renderer creation failed: " << SDL_GetError() << std::endl;
            return;
        }
        
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
        
        if (TTF_Init() == 0) {
            ttf_initialized_ = true;
            font_ = TTF_OpenFont("/usr/share/fonts/truetype/ubuntu/Ubuntu-M.ttf", 14);
            if (!font_) {
                font_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14);
            }
            if (!font_) {
                font_ = TTF_OpenFont("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 14);
            }
            if (font_) {
                TTF_SetFontStyle(font_, TTF_STYLE_NORMAL);
            }
        }
    }
    
    void update_scale() {
        int window_w, window_h;
        SDL_GetWindowSize(window_, &window_w, &window_h);
        width_ = window_w;
        height_ = window_h;
        center_x_ = width_ / 2;
        center_y_ = height_ / 2;
        
        if (current_range_km_ < min_range_km_) current_range_km_ = min_range_km_;
        if (current_range_km_ > max_range_km_) current_range_km_ = max_range_km_;
        
        double current_range_m = current_range_km_ * 1000.0;
        scale_ = static_cast<double>(std::min(width_, height_) - 100) / (current_range_m * 2);
    }
    
    void zoom_in() {
        double new_range = current_range_km_ * 0.8;
        if (new_range >= min_range_km_) {
            current_range_km_ = new_range;
            update_scale();
        }
    }
    
    void zoom_out() {
        double new_range = current_range_km_ * 1.25;
        if (new_range <= max_range_km_) {
            current_range_km_ = new_range;
            update_scale();
        }
    }
    
    void draw_grid() {
        SDL_SetRenderDrawColor(renderer_, 40, 40, 40, 255);
        
        double current_range_m = current_range_km_ * 1000.0;
        
        for (int range_km = 50; range_km <= 200; range_km += 50) {
            double range_m = range_km * 1000.0;
            if (range_m > current_range_m + 0.1) break;
            
            int radius = static_cast<int>(range_m * scale_);
            draw_circle(center_x_, center_y_, radius, false);
            
            if (font_) {
                char label[16];
                snprintf(label, sizeof(label), "%d km", range_km);
                SDL_Surface* surface = TTF_RenderText_Solid(font_, label, {80, 80, 80, 255});
                if (surface) {
                    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
                    SDL_Rect rect = {center_x_ + radius - 30, center_y_ - 10, surface->w, surface->h};
                    SDL_RenderCopy(renderer_, texture, nullptr, &rect);
                    SDL_FreeSurface(surface);
                    SDL_DestroyTexture(texture);
                }
            }
        }
        
        SDL_SetRenderDrawColor(renderer_, 50, 50, 50, 255);
        double start_range_m = 10000.0;
        int start_radius = static_cast<int>(start_range_m * scale_);
        
        if (start_radius < std::min(width_, height_) / 2) {
            for (int az = 0; az < 360; az += 30) {
                double rad = az * M_PI / 180.0;
                int x_start = center_x_ + static_cast<int>(start_range_m * scale_ * sin(rad));
                int y_start = center_y_ - static_cast<int>(start_range_m * scale_ * cos(rad));
                int x_end = center_x_ + static_cast<int>(current_range_m * scale_ * sin(rad));
                int y_end = center_y_ - static_cast<int>(current_range_m * scale_ * cos(rad));
                SDL_RenderDrawLine(renderer_, x_start, y_start, x_end, y_end);
                
                if (font_ && current_range_km_ >= 190) {
                    char label[8];
                    if (az == 0) snprintf(label, sizeof(label), "N");
                    else if (az == 90) snprintf(label, sizeof(label), "E");
                    else if (az == 180) snprintf(label, sizeof(label), "S");
                    else if (az == 270) snprintf(label, sizeof(label), "W");
                    else snprintf(label, sizeof(label), "%d°", az);
                    
                    SDL_Surface* surface = TTF_RenderText_Solid(font_, label, {80, 80, 80, 255});
                    if (surface) {
                        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
                        int x_label = x_end - 15;
                        int y_label = y_end - 10;
                        if (az == 0) { x_label = x_end - 8; y_label = y_end - 25; }
                        else if (az == 90) { x_label = x_end + 5; y_label = y_end - 8; }
                        else if (az == 180) { x_label = x_end - 8; y_label = y_end + 10; }
                        else if (az == 270) { x_label = x_end - 25; y_label = y_end - 8; }
                        
                        SDL_Rect rect = {x_label, y_label, surface->w, surface->h};
                        SDL_RenderCopy(renderer_, texture, nullptr, &rect);
                        SDL_FreeSurface(surface);
                        SDL_DestroyTexture(texture);
                    }
                }
            }
        }
        
        SDL_SetRenderDrawColor(renderer_, 200, 200, 200, 255);
        int cross_size = 12;
        SDL_RenderDrawLine(renderer_, center_x_ - cross_size, center_y_, 
                           center_x_ + cross_size, center_y_);
        SDL_RenderDrawLine(renderer_, center_x_, center_y_ - cross_size,
                           center_x_, center_y_ + cross_size);
        SDL_RenderDrawLine(renderer_, center_x_ - cross_size/2, center_y_ - cross_size/2,
                           center_x_ + cross_size/2, center_y_ + cross_size/2);
        SDL_RenderDrawLine(renderer_, center_x_ - cross_size/2, center_y_ + cross_size/2,
                           center_x_ + cross_size/2, center_y_ - cross_size/2);
    }
    
    // НОВАЯ ФУНКЦИЯ ДЛЯ НАКОПЛЕННЫХ ОТВЕТОВ
    void draw_accumulated_replies() {
        double current_range_m = current_range_km_ * 1000.0;
        
        // RBS: 30 метров на дискрет
        SDL_SetRenderDrawColor(renderer_, 100, 100, 255, 180);
        for (const auto& reply : accumulated_rbs_) {
            double range_m = reply.range * 30.0;
            if (range_m > current_range_m) continue;
            
            double rad = reply.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
            int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
            int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                SDL_Rect rect = {x - 2, y - 2, 4, 4};
                SDL_RenderFillRect(renderer_, &rect);
            }
        }
        
        // UVD: 60 метров на дискрет
        SDL_SetRenderDrawColor(renderer_, 255, 100, 100, 180);
        for (const auto& reply : accumulated_uvd_) {
            double range_m = reply.range * 60.0;
            if (range_m > current_range_m) continue;
            
            double rad = reply.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
            int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
            int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                SDL_Rect rect = {x - 2, y - 2, 4, 4};
                SDL_RenderFillRect(renderer_, &rect);
            }
        }
    }
    
    void draw_scan_line() {
        double current_range_m = current_range_km_ * 1000.0;
        double rad = current_scan_.azimuth * RadarConfig::azimuth_per_bin * M_PI / 180.0;
        int x2 = center_x_ + static_cast<int>(current_range_m * scale_ * sin(rad));
        int y2 = center_y_ - static_cast<int>(current_range_m * scale_ * cos(rad));
        
        for (int i = 0; i <= 10; ++i) {
            double factor = i / 10.0;
            int alpha = static_cast<int>(150 * (1 - factor * 0.7));
            SDL_SetRenderDrawColor(renderer_, 0, 255, 0, alpha);
            int x_step = center_x_ + static_cast<int>(x2 - center_x_) * factor;
            int y_step = center_y_ + static_cast<int>(y2 - center_y_) * factor;
            SDL_RenderDrawLine(renderer_, center_x_, center_y_, x_step, y_step);
        }
        
        SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
        SDL_RenderDrawLine(renderer_, center_x_, center_y_, x2, y2);
        
        draw_circle(x2, y2, 6, false);
        draw_circle(x2, y2, 3, true);
    }
    
    void draw_targets() {
        double current_range_m = current_range_km_ * 1000.0;
        
        for (const auto& target : targets_) {
            double range_m = std::sqrt(target.x * target.x + target.y * target.y);
            if (range_m > current_range_m) continue;
            
            int x = center_x_ + static_cast<int>(target.x * scale_);
            int y = center_y_ - static_cast<int>(target.y * scale_);
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                if (target.type == TargetReport::SourceType::RBS) {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 255);
                    draw_circle(x, y, 8, false);
                    SDL_SetRenderDrawColor(renderer_, 0, 200, 200, 200);
                    draw_circle(x, y, 4, true);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);
                    draw_circle(x, y, 8, false);
                    SDL_SetRenderDrawColor(renderer_, 200, 200, 0, 200);
                    draw_circle(x, y, 4, true);
                }
            }
        }
    }
    
    void draw_tracks() {
        double current_range_m = current_range_km_ * 1000.0;
        
        for (const auto& track : tracks_) {
            double range_m = std::sqrt(track.x * track.x + track.y * track.y);
            if (range_m > current_range_m) continue;
            
            int x = center_x_ + static_cast<int>(track.x * scale_);
            int y = center_y_ - static_cast<int>(track.y * scale_);
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                if (track.state == TrackState::ACTIVE) {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
                } else if (track.state == TrackState::COASTING) {
                    SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 128, 128, 128, 255);
                }
                
                draw_circle(x, y, 12, false);
                draw_circle(x, y, 5, true);
                
                if (track.ground_speed > 1) {
                    double speed_scale = std::min(30.0, track.ground_speed / 5.0);
                    double course_rad = track.course_deg * M_PI / 180.0;
                    int x2 = x + static_cast<int>(speed_scale * scale_ * sin(course_rad));
                    int y2 = y - static_cast<int>(speed_scale * scale_ * cos(course_rad));
                    SDL_RenderDrawLine(renderer_, x, y, x2, y2);
                    draw_arrow(x, y, x2, y2);
                }
            }
        }
    }
    
    void draw_arrow(int x1, int y1, int x2, int y2) {
        double angle = std::atan2(y2 - y1, x2 - x1);
        double arrow_angle = 0.5;
        int arrow_len = 10;
        
        int x3 = x2 - static_cast<int>(arrow_len * cos(angle - arrow_angle));
        int y3 = y2 - static_cast<int>(arrow_len * sin(angle - arrow_angle));
        int x4 = x2 - static_cast<int>(arrow_len * cos(angle + arrow_angle));
        int y4 = y2 - static_cast<int>(arrow_len * sin(angle + arrow_angle));
        
        SDL_RenderDrawLine(renderer_, x2, y2, x3, y3);
        SDL_RenderDrawLine(renderer_, x2, y2, x4, y4);
    }
    
    void draw_info() {
        if (!font_) return;
        
        char info[512];
        snprintf(info, sizeof(info), 
                 "Rev: %d | Az: %d | RBS: %zu | UVD: %zu | Targets: %zu | Tracks: %zu | Range: %.1f km",
                 current_revolution_, current_scan_.azimuth,
                 accumulated_rbs_.size(), accumulated_uvd_.size(),
                 targets_.size(), tracks_.size(), current_range_km_);
        
        SDL_Surface* surface = TTF_RenderText_Solid(font_, info, {200, 200, 200, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect rect = {10, 10, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, nullptr, &rect);
            SDL_FreeSurface(surface);
            SDL_DestroyTexture(texture);
        }
        
        const char* help = "Mouse wheel: Zoom | R: Reset (200 km) | C: Clear hist | ESC: Exit";
        surface = TTF_RenderText_Solid(font_, help, {150, 150, 150, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect rect = {10, height_ - 30, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, nullptr, &rect);
            SDL_FreeSurface(surface);
            SDL_DestroyTexture(texture);
        }
    }
    
    void draw_mouse_coords() {
        if (!has_mouse_pos_ || !font_) return;
        
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "Cursor: R=" << mouse_world_range_km_ << " km";
        ss << " | Az=" << std::setprecision(1) << mouse_world_azimuth_ << "°";
        ss << " | X=" << std::setprecision(1) << mouse_world_x_km_ << " km";
        ss << ", Y=" << std::setprecision(1) << mouse_world_y_km_ << " km";
        
        SDL_Surface* surface = TTF_RenderText_Solid(font_, ss.str().c_str(), 
                                                     {255, 255, 200, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            
            int x = 10;
            int y = height_ - 55;
            
            SDL_Rect rect = {x, y, surface->w, surface->h};
            
            SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 180);
            SDL_Rect bg_rect = {x - 2, y - 2, surface->w + 4, surface->h + 4};
            SDL_RenderFillRect(renderer_, &bg_rect);
            
            SDL_RenderCopy(renderer_, texture, nullptr, &rect);
            SDL_FreeSurface(surface);
            SDL_DestroyTexture(texture);
        }
    }
    
    void draw_circle(int cx, int cy, int radius, bool fill) {
        if (fill) {
            for (int y = -radius; y <= radius; ++y) {
                for (int x = -radius; x <= radius; ++x) {
                    if (x*x + y*y <= radius*radius) {
                        SDL_RenderDrawPoint(renderer_, cx + x, cy + y);
                    }
                }
            }
        } else {
            int x = radius;
            int y = 0;
            int err = 0;
            
            while (x >= y) {
                SDL_RenderDrawPoint(renderer_, cx + x, cy + y);
                SDL_RenderDrawPoint(renderer_, cx + y, cy + x);
                SDL_RenderDrawPoint(renderer_, cx - y, cy + x);
                SDL_RenderDrawPoint(renderer_, cx - x, cy + y);
                SDL_RenderDrawPoint(renderer_, cx - x, cy - y);
                SDL_RenderDrawPoint(renderer_, cx - y, cy - x);
                SDL_RenderDrawPoint(renderer_, cx + y, cy - x);
                SDL_RenderDrawPoint(renderer_, cx + x, cy - y);
                
                y++;
                err += 1 + 2*y;
                if (2*(err - x) + 1 > 0) {
                    x--;
                    err += 1 - 2*x;
                }
            }
        }
    }
    
    int width_, height_;
    int center_x_, center_y_;
    double scale_;
    const double max_range_km_;
    const double min_range_km_;
    double current_range_km_;
    double max_range_m_;
    bool running_;
    
    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    TTF_Font* font_{nullptr};
    bool ttf_initialized_{false};
    
    std::vector<TargetReport> targets_;
    std::vector<Track> tracks_;
    ScanReplies current_scan_;
    int current_revolution_{0};
    int last_revolution_{-1};
    
    // НАКОПЛЕННЫЕ ОТВЕТЫ
    std::vector<RBSReply> accumulated_rbs_;
    std::vector<UVDReply> accumulated_uvd_;
    
    bool has_mouse_pos_{false};
    double mouse_world_range_m_{0.0};
    double mouse_world_range_km_{0.0};
    double mouse_world_azimuth_{0.0};
    double mouse_world_x_km_{0.0}, mouse_world_y_km_{0.0};
};

int main(int argc, char* argv[]) {
    std::cout << "=== Radar Viewer - Range 200 km ===\n";
    std::cout << "Controls: Mouse wheel - Zoom, R - Reset, C - Clear history, ESC - Exit\n\n";
    
    auto config = SystemConfig::load_from_file("radar.conf");
    config.tracker.debug_mode = false;
    
    config.radar.range_bin_rbs = 30.0;
    config.radar.range_bin_uvd = 60.0;
    
    RadarSystem system(config);
    
    if (!system.initialize()) {
        std::cerr << "Failed to initialize radar system\n";
        return 1;
    }
    
    RadarViewer viewer(1280, 960);
    
    std::vector<TargetReport> all_targets;
    all_targets.reserve(200);
    
    system.set_target_callback([&](const TargetReport& target) {
        all_targets.push_back(target);
        if (all_targets.size() > 200) {
            all_targets.erase(all_targets.begin());
        }
        viewer.update_targets(all_targets);
    });
    
    system.enable_raw_reply_logging(true, "raw_replies.txt");


    int scan_num = 0;
    auto start_time = std::chrono::steady_clock::now();
    const int SCAN_INTERVAL_MS = 2;
    
    std::cout << "Starting simulation...\n";
    
    while (viewer.is_running()) {
        viewer.handle_events();
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        
        if (elapsed >= SCAN_INTERVAL_MS) {
            uint16_t azimuth = (scan_num * 2) % 4096;
            auto scan = system.generate_test_scan(azimuth, 0);
            system.process_scan(scan);
            
            viewer.update_scan(scan, scan_num / 2048);
            scan_num++;
            start_time = now;
        }
        
        auto tracks = system.get_tracks();
        viewer.update_tracks(tracks);
        viewer.render();
        
        SDL_Delay(1);
    }
    
    system.shutdown();
    std::cout << "\n=== Radar Viewer Stopped ===\n";
    
    return 0;
}
