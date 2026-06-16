// tools/4_radar_player.cpp
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>

struct ReplyData {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool spi;
};

struct PlotData {
    double time_sec;
    double azimuth_deg;
    double range_km;
    double x_km;
    double y_km;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool spi;
    int reply_count;
};

struct TrackData {
    double time_sec;
    int track_id;
    double x_km;
    double y_km;
    double speed_km_s;
    double course_deg;
    uint32_t code_data;
    int altitude;
    double confidence;
    int hit_count;
};

class RadarPlayer {
public:
    RadarPlayer(int width = 1280, int height = 960) 
        : width_(width), height_(height), running_(true), current_time_(0.0),
          max_range_km_(200.0), current_range_km_(200.0), play_speed_(1.0) {
        
        center_x_ = width / 2;
        center_y_ = height / 2;
        scale_ = static_cast<double>(std::min(width, height) - 100) / (max_range_km_ * 1000.0 * 2);
        
        init_sdl();
    }
    
    ~RadarPlayer() {
        if (renderer_) SDL_DestroyRenderer(renderer_);
        if (window_) SDL_DestroyWindow(window_);
        if (font_) TTF_CloseFont(font_);
        if (ttf_initialized_) TTF_Quit();
        SDL_Quit();
    }
    
    void load_data(const std::string& replies_file, 
                   const std::string& plots_file, 
                   const std::string& tracks_file) {
        load_replies(replies_file);
        load_plots(plots_file);
        load_tracks(tracks_file);
        
        max_time_ = 0.0;
        for (const auto& r : replies_) max_time_ = std::max(max_time_, r.time_sec);
        for (const auto& p : plots_) max_time_ = std::max(max_time_, p.time_sec);
        for (const auto& t : tracks_) max_time_ = std::max(max_time_, t.time_sec);
        
        std::cout << "Loaded: " << replies_.size() << " replies, "
                  << plots_.size() << " plots, "
                  << tracks_.size() << " tracks\n";
        std::cout << "Total time: " << max_time_ << " seconds\n";
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
        SDL_RenderClear(renderer_);
        
        draw_grid();
        draw_replies_at_time(current_time_);
        draw_plots_at_time(current_time_);
        draw_tracks_at_time(current_time_);
        draw_scan_line(current_time_);
        draw_info();
        
        SDL_RenderPresent(renderer_);
    }
    
    void update(double delta_sec) {
        current_time_ += delta_sec * play_speed_;
        if (current_time_ > max_time_) {
            current_time_ = 0.0;  // Loop
        }
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
                } else if (event.key.keysym.sym == SDLK_SPACE) {
                    if (play_speed_ > 0.0) {
                        play_speed_ = 0.0;
                    } else {
                        play_speed_ = 1.0;
                    }
                } else if (event.key.keysym.sym == SDLK_r) {
                    current_time_ = 0.0;
                } else if (event.key.keysym.sym == SDLK_UP) {
                    play_speed_ *= 2.0;
                } else if (event.key.keysym.sym == SDLK_DOWN) {
                    play_speed_ /= 2.0;
                }
            } else if (event.type == SDL_MOUSEWHEEL) {
                if (event.wheel.y > 0) {
                    zoom_in();
                } else if (event.wheel.y < 0) {
                    zoom_out();
                }
            }
        }
    }
    
private:
    void load_replies(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            ReplyData reply;
            std::string spi_str;
            int sls;
            
            ss >> reply.time_sec;
            ss.ignore(1, ',');
            ss >> reply.azimuth;
            ss.ignore(1, ',');
            ss >> reply.range;
            ss.ignore(1, ',');
            ss >> reply.type;
            ss.ignore(1, ',');
            ss >> reply.code_data;
            ss.ignore(1, ',');
            ss >> reply.altitude;
            ss.ignore(1, ',');
            ss >> spi_str;
            reply.spi = (spi_str == "1");
            ss.ignore(1, ',');
            ss >> sls;
            
            replies_.push_back(reply);
        }
    }
    
    void load_plots(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            PlotData plot;
            std::string spi_str;
            
            ss >> plot.time_sec;
            ss.ignore(1, ',');
            ss >> plot.azimuth_deg;
            ss.ignore(1, ',');
            ss >> plot.range_km;
            ss.ignore(1, ',');
            ss >> plot.x_km;
            ss.ignore(1, ',');
            ss >> plot.y_km;
            ss.ignore(1, ',');
            ss >> plot.type;
            ss.ignore(1, ',');
            ss >> plot.code_data;
            ss.ignore(1, ',');
            ss >> plot.altitude;
            ss.ignore(1, ',');
            ss >> spi_str;
            plot.spi = (spi_str == "1");
            ss.ignore(1, ',');
            ss >> plot.reply_count;
            
            plots_.push_back(plot);
        }
    }
    
    void load_tracks(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::stringstream ss(line);
            TrackData track;
            
            ss >> track.time_sec;
            ss.ignore(1, ',');
            ss >> track.track_id;
            ss.ignore(1, ',');
            ss >> track.x_km;
            ss.ignore(1, ',');
            ss >> track.y_km;
            ss.ignore(1, ',');
            ss >> track.speed_km_s;
            ss.ignore(1, ',');
            ss >> track.course_deg;
            ss.ignore(1, ',');
            ss >> track.code_data;
            ss.ignore(1, ',');
            ss >> track.altitude;
            ss.ignore(1, ',');
            ss >> track.confidence;
            ss.ignore(1, ',');
            ss >> track.hit_count;
            
            tracks_.push_back(track);
        }
    }
    
    void draw_replies_at_time(double time) {
        double time_window = 0.5;  // Показываем ответы за последние 0.5 секунды
        double current_range_m = current_range_km_ * 1000.0;
        
        for (const auto& reply : replies_) {
            if (std::abs(reply.time_sec - time) > time_window) continue;
            
            double range_m = reply.range * 30.0;  // Предполагаем RBS дискрет 30м
            if (range_m > current_range_m) continue;
            
            double rad = reply.azimuth * (360.0 / 4096.0) * M_PI / 180.0;
            int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
            int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                if (reply.type == "RBS") {
                    SDL_SetRenderDrawColor(renderer_, 100, 100, 255, 150);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 255, 100, 100, 150);
                }
                SDL_Rect rect = {x - 2, y - 2, 4, 4};
                SDL_RenderFillRect(renderer_, &rect);
            }
        }
    }
    
    void draw_plots_at_time(double time) {
        double time_window = 0.5;
        double current_range_m = current_range_km_ * 1000.0;
        
        for (const auto& plot : plots_) {
            if (std::abs(plot.time_sec - time) > time_window) continue;
            
            double range_m = plot.range_km * 1000.0;
            if (range_m > current_range_m) continue;
            
            int x = center_x_ + static_cast<int>(plot.x_km * 1000.0 * scale_);
            int y = center_y_ - static_cast<int>(plot.y_km * 1000.0 * scale_);
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                if (plot.type == "RBS") {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 255);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);
                }
                draw_circle(x, y, 6, false);
            }
        }
    }
    
    void draw_tracks_at_time(double time) {
        double time_window = 1.0;
        double current_range_m = current_range_km_ * 1000.0;
        
        std::map<int, TrackData> latest_tracks;
        for (const auto& track : tracks_) {
            if (std::abs(track.time_sec - time) <= time_window) {
                if (latest_tracks.find(track.track_id) == latest_tracks.end() ||
                    track.time_sec > latest_tracks[track.track_id].time_sec) {
                    latest_tracks[track.track_id] = track;
                }
            }
        }
        
        for (const auto& [id, track] : latest_tracks) {
            double range_m = std::sqrt(track.x_km*track.x_km + track.y_km*track.y_km) * 1000.0;
            if (range_m > current_range_m) continue;
            
            int x = center_x_ + static_cast<int>(track.x_km * 1000.0 * scale_);
            int y = center_y_ - static_cast<int>(track.y_km * 1000.0 * scale_);
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                // Цвет трека в зависимости от уверенности
                int green = static_cast<int>(track.confidence * 255);
                SDL_SetRenderDrawColor(renderer_, 0, green, 0, 255);
                draw_circle(x, y, 10, false);
                draw_circle(x, y, 4, true);
                
                // Вектор скорости
                if (track.speed_km_s > 0.01) {
                    double speed_scale = std::min(50.0, track.speed_km_s * 10.0);
                    double course_rad = track.course_deg * M_PI / 180.0;
                    int x2 = x + static_cast<int>(speed_scale * scale_ * 1000.0 * sin(course_rad));
                    int y2 = y - static_cast<int>(speed_scale * scale_ * 1000.0 * cos(course_rad));
                    SDL_RenderDrawLine(renderer_, x, y, x2, y2);
                }
            }
        }
    }
    
    void draw_scan_line(double time) {
        double revolution_time = 5.0;  // 5 секунд на оборот
        double progress = fmod(time / revolution_time, 1.0);
        int azimuth = static_cast<int>(progress * 4096) % 4096;
        
        double current_range_m = current_range_km_ * 1000.0;
        double rad = azimuth * (360.0 / 4096.0) * M_PI / 180.0;
        int x2 = center_x_ + static_cast<int>(current_range_m * scale_ * sin(rad));
        int y2 = center_y_ - static_cast<int>(current_range_m * scale_ * cos(rad));
        
        SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 200);
        SDL_RenderDrawLine(renderer_, center_x_, center_y_, x2, y2);
        
        draw_circle(x2, y2, 6, false);
    }
    
    void draw_grid() {
        // ... (как в radar_viewer.cpp)
    }
    
    void draw_info() {
        if (!font_) return;
        
        char info[256];
        snprintf(info, sizeof(info), 
                 "Time: %.1fs / %.1fs | Speed: %.1fx | Replies: %zu | Plots: %zu | Tracks: %zu",
                 current_time_, max_time_, play_speed_,
                 replies_.size(), plots_.size(), tracks_.size());
        
        SDL_Surface* surface = TTF_RenderText_Solid(font_, info, {200, 200, 200, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect rect = {10, 10, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, nullptr, &rect);
            SDL_FreeSurface(surface);
            SDL_DestroyTexture(texture);
        }
        
        const char* help = "SPACE: Pause | UP/DOWN: Speed | R: Reset | Wheel: Zoom | ESC: Exit";
        surface = TTF_RenderText_Solid(font_, help, {150, 150, 150, 255});
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect rect = {10, height_ - 30, surface->w, surface->h};
            SDL_RenderCopy(renderer_, texture, nullptr, &rect);
            SDL_FreeSurface(surface);
            SDL_DestroyTexture(texture);
        }
    }
    
    void init_sdl() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
            return;
        }
        
        window_ = SDL_CreateWindow("Radar Player - All Stages", 
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
        }
    }
    
    void draw_circle(int cx, int cy, int radius, bool fill) {
        // ... (как в radar_viewer.cpp)
    }
    
    void zoom_in() {
        current_range_km_ *= 0.8;
        if (current_range_km_ < 10.0) current_range_km_ = 10.0;
        update_scale();
    }
    
    void zoom_out() {
        current_range_km_ *= 1.25;
        if (current_range_km_ > max_range_km_) current_range_km_ = max_range_km_;
        update_scale();
    }
    
    void update_scale() {
        int w, h;
        SDL_GetWindowSize(window_, &w, &h);
        width_ = w;
        height_ = h;
        center_x_ = width_ / 2;
        center_y_ = height_ / 2;
        scale_ = static_cast<double>(std::min(width_, height_) - 100) / (current_range_km_ * 1000.0 * 2);
    }
    
    int width_, height_;
    int center_x_, center_y_;
    double scale_;
    double max_range_km_;
    double current_range_km_;
    double play_speed_;
    double current_time_;
    double max_time_;
    bool running_;
    
    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    TTF_Font* font_{nullptr};
    bool ttf_initialized_{false};
    
    std::vector<ReplyData> replies_;
    std::vector<PlotData> plots_;
    std::vector<TrackData> tracks_;
};

int main(int argc, char* argv[]) {
    std::string replies_file = "replies.txt";
    std::string plots_file = "plots.txt";
    std::string tracks_file = "tracks.txt";
    
    if (argc > 1) replies_file = argv[1];
    if (argc > 2) plots_file = argv[2];
    if (argc > 3) tracks_file = argv[3];
    
    std::cout << "=== Step 4: Radar Player ===\n";
    std::cout << "Replies: " << replies_file << "\n";
    std::cout << "Plots: " << plots_file << "\n";
    std::cout << "Tracks: " << tracks_file << "\n\n";
    
    RadarPlayer player(1280, 960);
    player.load_data(replies_file, plots_file, tracks_file);
    
    auto last_time = std::chrono::steady_clock::now();
    
    while (player.is_running()) {
        player.handle_events();
        
        auto now = std::chrono::steady_clock::now();
        double delta_sec = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        
        player.update(delta_sec);
        player.render();
        
        SDL_Delay(1);
    }
    
    return 0;
}
