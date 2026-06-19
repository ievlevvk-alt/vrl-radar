// tools/4_radar_player.cpp
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <algorithm>
#include <memory>

// ============================================================================
// RAII-ОБЕРТКИ ДЛЯ SDL РЕСУРСОВ
// ============================================================================

struct SDLDeleter {
    void operator()(SDL_Window* w) const { if (w) SDL_DestroyWindow(w); }
    void operator()(SDL_Renderer* r) const { if (r) SDL_DestroyRenderer(r); }
    void operator()(SDL_Texture* t) const { if (t) SDL_DestroyTexture(t); }
    void operator()(SDL_Surface* s) const { if (s) SDL_FreeSurface(s); }
    void operator()(TTF_Font* f) const { if (f) TTF_CloseFont(f); }
};

using UniqueWindow = std::unique_ptr<SDL_Window, SDLDeleter>;
using UniqueRenderer = std::unique_ptr<SDL_Renderer, SDLDeleter>;
using UniqueTexture = std::unique_ptr<SDL_Texture, SDLDeleter>;
using UniqueSurface = std::unique_ptr<SDL_Surface, SDLDeleter>;
using UniqueFont = std::unique_ptr<TTF_Font, SDLDeleter>;

// Вспомогательная функция для создания текстуры из текста
class TextureCache {
public:
    struct CachedTexture {
        UniqueTexture texture;
        int width{0};
        int height{0};
    };
    
    CachedTexture* get_or_create(SDL_Renderer* renderer, TTF_Font* font, 
                                  const std::string& text, const SDL_Color& color) {
        std::string key = text + "|" + std::to_string(color.r) + "," + 
                          std::to_string(color.g) + "," + std::to_string(color.b);
        
        auto it = cache_.find(key);
        if (it != cache_.end()) {
            return &it->second;
        }
        
        UniqueSurface surface(TTF_RenderText_Blended(font, text.c_str(), color));
        if (!surface) {
            return nullptr;
        }
        
        CachedTexture cached;
        cached.width = surface->w;
        cached.height = surface->h;
        cached.texture = UniqueTexture(
            SDL_CreateTextureFromSurface(renderer, surface.get())
        );
        
        if (!cached.texture) {
            return nullptr;
        }
        
        auto result = cache_.emplace(key, std::move(cached));
        return &result.first->second;
    }
    
    void clear() {
        cache_.clear();
    }
    
private:
    std::map<std::string, CachedTexture> cache_;
};

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ СТРУКТУРЫ
// ============================================================================

struct ReplyData {
    double time_sec;
    uint16_t azimuth;
    uint16_t range;
    std::string type;
    uint32_t code_data;
    int altitude;
    bool spi;
    bool is_valid;
    bool is_garble;
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
    bool altitude_valid;
    int altitude_attempts;
    bool spi;
    int reply_count;
    int garble_count;
    double azimuth_span_deg;
    double range_span_km;
    double first_reply_time;
    double last_reply_time;
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
    bool altitude_valid;
    double confidence;
    int hit_count;
    int state;
    int garble_count;
    int reply_count;
    std::string type;
    bool code_reliable;
    bool altitude_reliable;
};

struct TrackLabel {
    int track_id;
    std::string code;
    int altitude;
    double confidence;
    int hit_count;
    double speed;
    double course;
    std::string type;
    bool code_reliable;
    bool altitude_reliable;
};

// ============================================================================
// ПАРСЕРЫ
// ============================================================================

static bool parse_reply_line(const std::string& line, ReplyData& reply) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() < 10) return false;
    
    try {
        reply.time_sec = std::stod(parts[0]);
        reply.azimuth = static_cast<uint16_t>(std::stoi(parts[1]));
        reply.range = static_cast<uint16_t>(std::stoi(parts[2]));
        reply.type = parts[3];
        
        if (!parts[4].empty() && parts[4][0] == '0') {
            reply.code_data = static_cast<uint32_t>(std::stoi(parts[4], nullptr, 8));
        } else {
            reply.code_data = static_cast<uint32_t>(std::stoi(parts[4]));
        }
        
        reply.altitude = std::stoi(parts[5]);
        reply.spi = (parts[6] == "1");
        reply.is_valid = (parts[8] == "1");
        reply.is_garble = (parts[9] == "1");
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

static bool parse_plot_line(const std::string& line, PlotData& plot) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() < 17) return false;
    
    try {
        plot.time_sec = std::stod(parts[0]);
        plot.azimuth_deg = std::stod(parts[1]);
        plot.range_km = std::stod(parts[2]);
        plot.x_km = std::stod(parts[3]);
        plot.y_km = std::stod(parts[4]);
        plot.type = parts[5];
        
        std::string code_str = parts[6];
        if (plot.type == "RBS" || plot.type == "RBS_A" || plot.type == "RBS_C") {
            plot.code_data = static_cast<uint32_t>(std::stoi(code_str, nullptr, 8));
        } else {
            plot.code_data = static_cast<uint32_t>(std::stoul(code_str));
        }
        
        plot.altitude = std::stoi(parts[7]);
        plot.altitude_valid = (parts[8] == "1");
        plot.altitude_attempts = std::stoi(parts[9]);
        plot.spi = (parts[10] == "1");
        plot.reply_count = std::stoi(parts[11]);
        plot.garble_count = (parts.size() > 12) ? std::stoi(parts[12]) : 0;
        plot.azimuth_span_deg = std::stod(parts[13]);
        plot.range_span_km = std::stod(parts[14]);
        plot.first_reply_time = std::stod(parts[15]);
        plot.last_reply_time = std::stod(parts[16]);
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

static bool parse_track_line(const std::string& line, TrackData& track) {
    std::vector<std::string> parts;
    std::stringstream ss_line(line);
    std::string part;
    while (std::getline(ss_line, part, ',')) {
        parts.push_back(part);
    }
    if (parts.size() < 13) return false;
    
    try {
        track.time_sec = std::stod(parts[0]);
        track.track_id = std::stoi(parts[1]);
        track.x_km = std::stod(parts[2]);
        track.y_km = std::stod(parts[3]);
        track.speed_km_s = std::stod(parts[4]);
        track.course_deg = std::stod(parts[5]);
        
        std::string code_str = parts[6];
        track.type = parts[12];
        
        if (track.type == "RBS") {
            track.code_data = static_cast<uint32_t>(std::stoi(code_str, nullptr, 8));
        } else {
            track.code_data = static_cast<uint32_t>(std::stoul(code_str));
        }
        
        track.altitude = std::stoi(parts[7]);
        track.altitude_valid = (parts[8] == "1");
        track.confidence = std::stod(parts[9]);
        track.hit_count = std::stoi(parts[10]);
        track.state = std::stoi(parts[11]);
        
        track.code_reliable = true;
        track.altitude_reliable = true;
        if (parts.size() >= 15) {
            track.code_reliable = (parts[13] == "1");
            track.altitude_reliable = (parts[14] == "1");
        }
        
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ ОТРИСОВКИ
// ============================================================================

static void draw_circle(SDL_Renderer* renderer, int cx, int cy, int radius, bool fill) {
    if (fill) {
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                if (x*x + y*y <= radius*radius) {
                    SDL_RenderDrawPoint(renderer, cx + x, cy + y);
                }
            }
        }
    } else {
        int x = radius;
        int y = 0;
        int err = 0;
        while (x >= y) {
            SDL_RenderDrawPoint(renderer, cx + x, cy + y);
            SDL_RenderDrawPoint(renderer, cx + y, cy + x);
            SDL_RenderDrawPoint(renderer, cx - y, cy + x);
            SDL_RenderDrawPoint(renderer, cx - x, cy + y);
            SDL_RenderDrawPoint(renderer, cx - x, cy - y);
            SDL_RenderDrawPoint(renderer, cx - y, cy - x);
            SDL_RenderDrawPoint(renderer, cx + y, cy - x);
            SDL_RenderDrawPoint(renderer, cx + x, cy - y);
            y++;
            err += 1 + 2*y;
            if (2*(err - x) + 1 > 0) {
                x--;
                err += 1 - 2*x;
            }
        }
    }
}

static void draw_arrow(SDL_Renderer* renderer, int x1, int y1, int x2, int y2) {
    double angle = std::atan2(y2 - y1, x2 - x1);
    double arrow_angle = 0.5;
    int arrow_len = 10;
    
    int x3 = x2 - static_cast<int>(arrow_len * cos(angle - arrow_angle));
    int y3 = y2 - static_cast<int>(arrow_len * sin(angle - arrow_angle));
    int x4 = x2 - static_cast<int>(arrow_len * cos(angle + arrow_angle));
    int y4 = y2 - static_cast<int>(arrow_len * sin(angle + arrow_angle));
    
    SDL_RenderDrawLine(renderer, x2, y2, x3, y3);
    SDL_RenderDrawLine(renderer, x2, y2, x4, y4);
}

// ============================================================================
// ОСНОВНОЙ КЛАСС RADAR PLAYER
// ============================================================================

class RadarPlayer {
public:
    RadarPlayer(int width = 1280, int height = 960);
    ~RadarPlayer();
    
    void load_data(const std::string& replies_file, 
                   const std::string& plots_file, 
                   const std::string& tracks_file);
    
    void render();
    void update(double delta_sec);
    bool is_running() const { return running_; }
    void handle_events();
    
private:
    void load_replies(const std::string& filename);
    void load_plots(const std::string& filename);
    void load_tracks(const std::string& filename);
    
    void draw_grid();
    void draw_replies_at_time(double time);
    void draw_plots_at_time(double time);
    void draw_track_history();
    void draw_current_tracks_at_time(double time);
    void draw_scan_line(double time);
    void draw_info();
    void draw_track_label(int x, int y, const TrackLabel& label);
    
    double get_current_azimuth(double time);
    bool is_visible_by_beam(double object_time, double object_azimuth_deg);
    bool init_sdl();
    void zoom_in();
    void zoom_out();
    void update_scale();
    
    int width_, height_;
    int center_x_, center_y_;
    double scale_;
    double max_range_km_;
    double current_range_km_;
    double play_speed_;
    double current_time_;
    double max_time_;
    double revolution_time_;
    double lookahead_time_;
    double display_beamwidth_deg_;
    bool running_;
    bool show_replies_;
    bool show_plots_;
    bool show_tracks_;
    
    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    TTF_Font* font_{nullptr};
    bool ttf_initialized_{false};
    
    // Кеш текстур для устранения утечек
    TextureCache texture_cache_;
    
    std::vector<ReplyData> replies_;
    std::vector<PlotData> plots_;
    std::vector<TrackData> tracks_;
    std::map<int, std::vector<TrackData>> track_history_;
};

RadarPlayer::RadarPlayer(int width, int height) 
    : width_(width), height_(height), running_(true),
      max_range_km_(200.0), current_range_km_(200.0),
      play_speed_(1.0), current_time_(0.0), max_time_(0.0),
      revolution_time_(5.0), display_beamwidth_deg_(3.0),
      show_replies_(true), show_plots_(true), show_tracks_(true),
      lookahead_time_(0.3) {
    
    center_x_ = width / 2;
    center_y_ = height / 2;
    scale_ = static_cast<double>(std::min(width, height) - 100) / (max_range_km_ * 1000.0 * 2);
    
    if (!init_sdl()) {
        running_ = false;
    }
}

RadarPlayer::~RadarPlayer() {
    // Очищаем кеш текстур
    texture_cache_.clear();
    
    // Закрываем шрифт
    if (font_) {
        TTF_CloseFont(font_);
        font_ = nullptr;
    }
    
    // Уничтожаем рендерер и окно
    if (renderer_) {
        SDL_DestroyRenderer(renderer_);
        renderer_ = nullptr;
    }
    if (window_) {
        SDL_DestroyWindow(window_);
        window_ = nullptr;
    }
    
    // Выходим из SDL
    if (ttf_initialized_) {
        TTF_Quit();
        ttf_initialized_ = false;
    }
    SDL_Quit();
}

void RadarPlayer::load_data(const std::string& replies_file, 
                            const std::string& plots_file, 
                            const std::string& tracks_file) {
    load_replies(replies_file);
    load_plots(plots_file);
    load_tracks(tracks_file);
    
    for (const auto& r : replies_) max_time_ = std::max(max_time_, r.time_sec);
    for (const auto& p : plots_) max_time_ = std::max(max_time_, p.time_sec);
    for (const auto& t : tracks_) max_time_ = std::max(max_time_, t.time_sec);
    
    for (const auto& t : tracks_) {
        track_history_[t.track_id].push_back(t);
    }
    
    for (auto& [id, history] : track_history_) {
        std::sort(history.begin(), history.end(),
            [](const TrackData& a, const TrackData& b) {
                return a.time_sec < b.time_sec;
            });
    }
    
    std::cout << "Loaded: " << replies_.size() << " replies, "
              << plots_.size() << " plots, "
              << tracks_.size() << " track points\n";
    std::cout << "Track IDs: " << track_history_.size() << "\n";
    std::cout << "Total time: " << max_time_ << " seconds\n";
}

void RadarPlayer::load_replies(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        ReplyData reply;
        if (parse_reply_line(line, reply)) {
            replies_.push_back(reply);
        }
    }
}

void RadarPlayer::load_plots(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        PlotData plot;
        if (parse_plot_line(line, plot)) {
            plots_.push_back(plot);
        }
    }
}

void RadarPlayer::load_tracks(const std::string& filename) {
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;
        TrackData track;
        if (parse_track_line(line, track)) {
            tracks_.push_back(track);
        }
    }
}

bool RadarPlayer::init_sdl() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL initialization failed: " << SDL_GetError() << std::endl;
        return false;
    }
    
    window_ = SDL_CreateWindow("Radar Player - VRL Radar", 
                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                               width_, height_, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        std::cerr << "Window creation failed: " << SDL_GetError() << std::endl;
        return false;
    }
    
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        std::cerr << "Renderer creation failed: " << SDL_GetError() << std::endl;
        return false;
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
    
    return true;
}

double RadarPlayer::get_current_azimuth(double time) {
    double progress = fmod(time / revolution_time_, 1.0);
    return progress * 360.0;
}

bool RadarPlayer::is_visible_by_beam(double object_time, double object_azimuth_deg) {
    double time_offset = object_time - current_time_;
    if (time_offset > lookahead_time_) return false;
    if (time_offset < -revolution_time_) return false;
    
    double beam_azimuth = get_current_azimuth(object_time);
    double az_diff = std::abs(object_azimuth_deg - beam_azimuth);
    az_diff = std::min(az_diff, 360.0 - az_diff);
    
    return az_diff < display_beamwidth_deg_;
}

void RadarPlayer::render() {
    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);
    
    draw_grid();
    draw_track_history();
    draw_replies_at_time(current_time_);
    draw_plots_at_time(current_time_);
    draw_current_tracks_at_time(current_time_);
    draw_scan_line(current_time_);
    draw_info();
    
    SDL_RenderPresent(renderer_);
}

void RadarPlayer::update(double delta_sec) {
    current_time_ += delta_sec * play_speed_;
    if (current_time_ > max_time_) {
        current_time_ = 0.0;
    }
}

void RadarPlayer::handle_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            running_ = false;
        } else if (event.type == SDL_KEYDOWN) {
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE: running_ = false; break;
                case SDLK_SPACE: play_speed_ = (play_speed_ > 0.0) ? 0.0 : 1.0; break;
                case SDLK_r: current_time_ = 0.0; break;
                case SDLK_UP: play_speed_ = std::min(play_speed_ * 2.0, 10.0); break;
                case SDLK_DOWN: play_speed_ = std::max(play_speed_ / 2.0, 0.1); break;
                case SDLK_1: show_replies_ = !show_replies_; break;
                case SDLK_2: show_plots_ = !show_plots_; break;
                case SDLK_3: show_tracks_ = !show_tracks_; break;
                default: break;
            }
        } else if (event.type == SDL_MOUSEWHEEL) {
            if (event.wheel.y > 0) zoom_in();
            else if (event.wheel.y < 0) zoom_out();
        }
    }
}

void RadarPlayer::zoom_in() {
    double new_range = current_range_km_ * 0.8;
    if (new_range >= 10.0) {
        current_range_km_ = new_range;
        update_scale();
    }
}

void RadarPlayer::zoom_out() {
    double new_range = current_range_km_ * 1.25;
    if (new_range <= max_range_km_) {
        current_range_km_ = new_range;
        update_scale();
    }
}

void RadarPlayer::update_scale() {
    int w, h;
    SDL_GetWindowSize(window_, &w, &h);
    width_ = w;
    height_ = h;
    center_x_ = width_ / 2;
    center_y_ = height_ / 2;
    scale_ = static_cast<double>(std::min(width_, height_) - 100) / (current_range_km_ * 1000.0 * 2);
}

void RadarPlayer::draw_grid() {
    SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255);
    double current_range_m = current_range_km_ * 1000.0;
    
    SDL_Color gray_color = {80, 80, 80, 255};
    
    // Рисуем круги дальности
    for (int range_km = 50; range_km <= 200; range_km += 50) {
        double range_m = range_km * 1000.0;
        if (range_m > current_range_m + 0.1) break;
        int radius = static_cast<int>(range_m * scale_);
        SDL_SetRenderDrawColor(renderer_, 50, 50, 50, 255);
        draw_circle(renderer_, center_x_, center_y_, radius, false);
        
        if (font_) {
            char label[16];
            snprintf(label, sizeof(label), "%d km", range_km);
            
            auto* cached = texture_cache_.get_or_create(renderer_, font_, label, gray_color);
            if (cached && cached->texture) {
                SDL_Rect rect = {center_x_ + radius - 30, center_y_ - 10, 
                                 cached->width, cached->height};
                SDL_RenderCopy(renderer_, cached->texture.get(), nullptr, &rect);
            }
        }
    }
    
    // Рисуем азимутальные линии
    SDL_SetRenderDrawColor(renderer_, 40, 40, 40, 255);
    
    double start_range_m = std::min(10000.0, current_range_m * 0.05);
    int start_radius = static_cast<int>(start_range_m * scale_);
    
    for (int az = 0; az < 360; az += 30) {
        double rad = az * M_PI / 180.0;
        
        int x_start = center_x_ + static_cast<int>(start_radius * sin(rad));
        int y_start = center_y_ - static_cast<int>(start_radius * cos(rad));
        int x_end = center_x_ + static_cast<int>(current_range_m * scale_ * sin(rad));
        int y_end = center_y_ - static_cast<int>(current_range_m * scale_ * cos(rad));
        
        SDL_RenderDrawLine(renderer_, x_start, y_start, x_end, y_end);
        
        if (font_) {
            char label[8];
            if (az == 0) snprintf(label, sizeof(label), "N");
            else if (az == 90) snprintf(label, sizeof(label), "E");
            else if (az == 180) snprintf(label, sizeof(label), "S");
            else if (az == 270) snprintf(label, sizeof(label), "W");
            else snprintf(label, sizeof(label), "%d°", az);
            
            auto* cached = texture_cache_.get_or_create(renderer_, font_, label, gray_color);
            if (cached && cached->texture) {
                int x_label = x_end - 15;
                int y_label = y_end - 10;
                
                if (az == 0) { 
                    x_label = x_end - 8; 
                    y_label = y_end - 25; 
                } else if (az == 90) { 
                    x_label = x_end + 5; 
                    y_label = y_end - 8; 
                } else if (az == 180) { 
                    x_label = x_end - 8; 
                    y_label = y_end + 10; 
                } else if (az == 270) { 
                    x_label = x_end - 25; 
                    y_label = y_end - 8; 
                }
                
                if (x_label < 0) x_label = 0;
                if (x_label + cached->width > width_) x_label = width_ - cached->width;
                if (y_label < 0) y_label = 0;
                if (y_label + cached->height > height_) y_label = height_ - cached->height;
                
                SDL_Rect rect = {x_label, y_label, cached->width, cached->height};
                SDL_RenderCopy(renderer_, cached->texture.get(), nullptr, &rect);
            }
        }
    }
    
    // Рисуем центр
    SDL_SetRenderDrawColor(renderer_, 200, 200, 200, 255);
    int cross_size = 12;
    SDL_RenderDrawLine(renderer_, center_x_ - cross_size, center_y_, 
                       center_x_ + cross_size, center_y_);
    SDL_RenderDrawLine(renderer_, center_x_, center_y_ - cross_size,
                       center_x_, center_y_ + cross_size);
}

void RadarPlayer::draw_replies_at_time(double time) {
    if (!show_replies_) return;
    
    double current_range_m = current_range_km_ * 1000.0;
    
    for (const auto& reply : replies_) {
        double az_deg = reply.azimuth * (360.0 / 4096.0);
        if (!is_visible_by_beam(reply.time_sec, az_deg)) continue;
        if (reply.is_garble) continue;
        
        double range_bin_m;
        if (reply.type == "RBS_A" || reply.type == "RBS_C") {
            range_bin_m = 30.0;
        } else {
            range_bin_m = 60.0;
        }
        double range_m = reply.range * range_bin_m;
        if (range_m > current_range_m) continue;
        
        double rad = reply.azimuth * (360.0 / 4096.0) * M_PI / 180.0;
        int x = center_x_ + static_cast<int>(range_m * scale_ * sin(rad));
        int y = center_y_ - static_cast<int>(range_m * scale_ * cos(rad));
        
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            if (reply.type == "RBS_A" || reply.type == "RBS_C") {
                SDL_SetRenderDrawColor(renderer_, 100, 100, 255, 200);
            } else {
                SDL_SetRenderDrawColor(renderer_, 255, 100, 100, 200);
            }
            SDL_Rect rect = {x - 2, y - 2, 4, 4};
            SDL_RenderFillRect(renderer_, &rect);
        }
    }
}

void RadarPlayer::draw_plots_at_time(double time) {
    if (!show_plots_) return;
    
    double current_range_m = current_range_km_ * 1000.0;
    
    for (const auto& plot : plots_) {
        if (!is_visible_by_beam(plot.time_sec, plot.azimuth_deg)) continue;
        
        double range_m = plot.range_km * 1000.0;
        if (range_m > current_range_m) continue;
        
        int x = center_x_ + static_cast<int>(plot.x_km * 1000.0 * scale_);
        int y = center_y_ - static_cast<int>(plot.y_km * 1000.0 * scale_);
        
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            int radius = 6 + std::min(plot.reply_count / 5, 4);
            double garble_ratio = (plot.reply_count > 0) ? 
                static_cast<double>(plot.garble_count) / plot.reply_count : 0.0;
            
            if (plot.type == "RBS" || plot.type == "RBS_A" || plot.type == "RBS_C") {
                if (garble_ratio > 0.2) {
                    SDL_SetRenderDrawColor(renderer_, 255, 165, 0, 255);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 255, 255, 0, 255);
                }
            } else {
                if (garble_ratio > 0.2) {
                    SDL_SetRenderDrawColor(renderer_, 255, 100, 255, 255);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 255);
                }
            }
            draw_circle(renderer_, x, y, radius, false);
            draw_circle(renderer_, x, y, radius - 2, true);
        }
    }
}

void RadarPlayer::draw_track_history() {
    if (!show_tracks_) return;
    
    double current_range_m = current_range_km_ * 1000.0;
    
    for (const auto& [track_id, history] : track_history_) {
        if (history.empty()) continue;
        
        std::vector<std::pair<int, int>> screen_points;
        for (const auto& t : history) {
            if (t.time_sec > current_time_) break;
            if (t.state != 1) continue;
            
            double range_m = std::sqrt(t.x_km*t.x_km + t.y_km*t.y_km) * 1000.0;
            if (range_m > current_range_m) continue;
            
            int x = center_x_ + static_cast<int>(t.x_km * 1000.0 * scale_);
            int y = center_y_ - static_cast<int>(t.y_km * 1000.0 * scale_);
            
            if (x >= 0 && x < width_ && y >= 0 && y < height_) {
                screen_points.push_back({x, y});
            }
        }
        
        std::string first_type = history[0].type;
        if (first_type == "RBS") {
            SDL_SetRenderDrawColor(renderer_, 0, 100, 0, 80);
        } else {
            SDL_SetRenderDrawColor(renderer_, 0, 0, 100, 80);
        }
        
        for (size_t i = 1; i < screen_points.size(); ++i) {
            SDL_RenderDrawLine(renderer_, 
                screen_points[i-1].first, screen_points[i-1].second,
                screen_points[i].first, screen_points[i].second);
        }
    }
}

void RadarPlayer::draw_current_tracks_at_time(double time) {
    if (!show_tracks_) return;
    
    double current_range_m = current_range_km_ * 1000.0;
    
    for (const auto& [track_id, history] : track_history_) {
        if (history.empty()) continue;
        
        const TrackData* latest = nullptr;
        for (size_t i = 0; i < history.size(); ++i) {
            if (history[i].time_sec <= time) {
                latest = &history[i];
            }
        }
        
        if (!latest || latest->state != 1) continue;
        
        double range_m = std::sqrt(latest->x_km*latest->x_km + latest->y_km*latest->y_km) * 1000.0;
        if (range_m > current_range_m) continue;
        
        int x = center_x_ + static_cast<int>(latest->x_km * 1000.0 * scale_);
        int y = center_y_ - static_cast<int>(latest->y_km * 1000.0 * scale_);
        
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            int green = static_cast<int>(latest->confidence * 255);
            
            if (latest->type == "RBS") {
                SDL_SetRenderDrawColor(renderer_, 0, green, 0, 255);
            } else {
                SDL_SetRenderDrawColor(renderer_, 0, green, green, 255);
            }
            draw_circle(renderer_, x, y, 12, false);
            
            if (latest->type == "RBS") {
                SDL_SetRenderDrawColor(renderer_, 0, green, 0, 200);
            } else {
                SDL_SetRenderDrawColor(renderer_, 0, green, green, 200);
            }
            draw_circle(renderer_, x, y, 5, true);
            
            double speed = (latest->speed_km_s > 0.01) ? latest->speed_km_s : 0.0;
            if (speed > 0.01 && latest->course_deg > 0) {
                double speed_pixels = std::min(80.0, speed * 400.0);
                double course_rad = latest->course_deg * M_PI / 180.0;
                int x2 = x + static_cast<int>(speed_pixels * sin(course_rad));
                int y2 = y - static_cast<int>(speed_pixels * cos(course_rad));
                
                if (latest->type == "RBS") {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
                } else {
                    SDL_SetRenderDrawColor(renderer_, 0, 255, 255, 255);
                }
                SDL_RenderDrawLine(renderer_, x, y, x2, y2);
                draw_arrow(renderer_, x, y, x2, y2);
            }
            
            TrackLabel label;
            label.track_id = latest->track_id;
            label.code = std::to_string(latest->code_data);
            label.altitude = latest->altitude;
            label.confidence = latest->confidence;
            label.hit_count = latest->hit_count;
            label.speed = latest->speed_km_s;
            label.course = latest->course_deg;
            label.type = latest->type;
            label.code_reliable = latest->code_reliable;
            label.altitude_reliable = latest->altitude_reliable;
            
            draw_track_label(x, y, label);
        }
    }
}

void RadarPlayer::draw_track_label(int x, int y, const TrackLabel& label) {
    if (!font_) return;
    
    std::string code_str;
    if (label.type == "RBS") {
        std::stringstream ss;
        ss << std::oct << std::stoul(label.code);
        code_str = ss.str();
        while (code_str.length() < 4) code_str = "0" + code_str;
        code_str = "0" + code_str;
    } else {
        code_str = label.code;
    }
    
    double speed_kmh = label.speed * 3600.0;
    
    char text[256];
    snprintf(text, sizeof(text),
            "Track %d\n"
            "Code: %s [%s]\n"
            "Alt: %d m [%s]\n"
            "Speed: %.0f km/h\n"
            "Course: %.1f deg\n"
            "Conf: %.0f%% Hits: %d",
            label.track_id,
            code_str.c_str(),
            label.code_reliable ? "OK" : "??",
            label.altitude,
            label.altitude_reliable ? "OK" : "??",
            speed_kmh,
            label.course,
            label.confidence * 100,
            label.hit_count);
    
    std::vector<std::string> lines;
    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line, '\n')) {
        lines.push_back(line);
    }
    
    SDL_Color color = (label.type == "RBS") ? 
        SDL_Color{0, 255, 0, 255} : SDL_Color{0, 255, 255, 255};
    
    // Создаем текстуры через кеш
    std::vector<TextureCache::CachedTexture*> cached_textures;
    int max_width = 0;
    int line_height = 18;
    
    for (const auto& l : lines) {
        auto* cached = texture_cache_.get_or_create(renderer_, font_, l, color);
        if (cached && cached->texture) {
            cached_textures.push_back(cached);
            max_width = std::max(max_width, cached->width);
        }
    }
    
    if (cached_textures.empty()) return;
    
    int padding = 8;
    int total_height = lines.size() * line_height + padding * 2;
    int label_x = x + 15;
    int label_y = y - total_height / 2;
    
    if (label_x + max_width + padding * 2 > width_) {
        label_x = x - max_width - padding * 2 - 15;
    }
    if (label_y < 10) label_y = 10;
    if (label_y + total_height > height_ - 10) {
        label_y = height_ - total_height - 10;
    }
    
    SDL_Color bg_color = (label.type == "RBS") ?
        SDL_Color{0, 40, 0, 200} : SDL_Color{0, 0, 40, 200};
    
    SDL_SetRenderDrawColor(renderer_, bg_color.r, bg_color.g, bg_color.b, bg_color.a);
    SDL_Rect bg_rect = {label_x - padding, label_y - padding, 
                        max_width + padding * 2, total_height};
    SDL_RenderFillRect(renderer_, &bg_rect);
    SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, 100);
    SDL_RenderDrawRect(renderer_, &bg_rect);
    
    for (size_t i = 0; i < cached_textures.size(); ++i) {
        SDL_Rect rect = {label_x, static_cast<int>(label_y + i * line_height), 
                         cached_textures[i]->width, cached_textures[i]->height};
        SDL_RenderCopy(renderer_, cached_textures[i]->texture.get(), nullptr, &rect);
    }
}

void RadarPlayer::draw_scan_line(double time) {
    double progress = fmod(time / revolution_time_, 1.0);
    int azimuth = static_cast<int>(progress * 4096) % 4096;
    
    double current_range_m = current_range_km_ * 1000.0;
    double rad = azimuth * (360.0 / 4096.0) * M_PI / 180.0;
    int x2 = center_x_ + static_cast<int>(current_range_m * scale_ * sin(rad));
    int y2 = center_y_ - static_cast<int>(current_range_m * scale_ * cos(rad));
    
    for (int i = 0; i <= 20; ++i) {
        double factor = i / 20.0;
        int alpha = static_cast<int>(200 * (1 - factor * 0.8));
        SDL_SetRenderDrawColor(renderer_, 0, 255, 0, alpha);
        int x_step = center_x_ + static_cast<int>(x2 - center_x_) * factor;
        int y_step = center_y_ + static_cast<int>(y2 - center_y_) * factor;
        SDL_RenderDrawLine(renderer_, center_x_, center_y_, x_step, y_step);
    }
    
    SDL_SetRenderDrawColor(renderer_, 0, 255, 0, 255);
    SDL_RenderDrawLine(renderer_, center_x_, center_y_, x2, y2);
    draw_circle(renderer_, x2, y2, 6, false);
    draw_circle(renderer_, x2, y2, 3, true);
}

void RadarPlayer::draw_info() {
    if (!font_) return;
    
    char info[512];
    snprintf(info, sizeof(info), 
             "Time: %.1fs / %.1fs | Speed: %.1fx | Range: %.1f km | "
             "Replies: %zu | Plots: %zu | Tracks: %zu | Az: %.1f°",
             current_time_, max_time_, play_speed_, current_range_km_,
             replies_.size(), plots_.size(), track_history_.size(),
             get_current_azimuth(current_time_));
    
    SDL_Color color = {200, 200, 200, 255};
    auto* cached = texture_cache_.get_or_create(renderer_, font_, info, color);
    if (cached && cached->texture) {
        SDL_Rect rect = {10, 10, cached->width, cached->height};
        SDL_RenderCopy(renderer_, cached->texture.get(), nullptr, &rect);
    }
    
    const char* help = "SPACE: Pause | UP/DOWN: Speed | 1:Replies 2:Plots 3:Tracks | "
                       "R:Reset | Wheel:Zoom | ESC:Exit";
    auto* cached_help = texture_cache_.get_or_create(renderer_, font_, help, 
                                                      SDL_Color{150, 150, 150, 255});
    if (cached_help && cached_help->texture) {
        SDL_Rect rect = {10, height_ - 30, cached_help->width, cached_help->height};
        SDL_RenderCopy(renderer_, cached_help->texture.get(), nullptr, &rect);
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char* argv[]) {
    std::string replies_file = "replies.txt";
    std::string plots_file = "plots_combined.txt";
    std::string tracks_file = "tracks_combined.txt";
    
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
