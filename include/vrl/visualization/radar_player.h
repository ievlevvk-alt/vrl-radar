// include/vrl/visualization/radar_player.h
#pragma once

#include "../radar/core/replies.h"
#include "../radar/processing/tracker.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <vector>
#include <map>
#include <string>

namespace vrl {
namespace visualization {

// ============================================================================
// RADAR PLAYER
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
    // Внутренние структуры данных
    struct ReplyData;
    struct PlotData;
    struct TrackData;
    struct TrackLabel;
    
    // Загрузка данных
    void load_replies(const std::string& filename);
    void load_plots(const std::string& filename);
    void load_tracks(const std::string& filename);
    
    // Отрисовка
    void draw_grid();
    void draw_replies_at_time(double time);
    void draw_plots_at_time(double time);
    void draw_track_history();
    void draw_current_tracks_at_time(double time);
    void draw_scan_line(double time);
    void draw_info();
    void draw_track_label(int x, int y, const TrackLabel& label);
    
    // Вспомогательные методы
    void draw_circle(int cx, int cy, int radius, bool fill);
    void draw_arrow(int x1, int y1, int x2, int y2);
    double get_current_azimuth(double time);
    bool is_visible_by_beam(double object_time, double object_azimuth_deg);
    void init_sdl();
    void zoom_in();
    void zoom_out();
    void update_scale();
    
    // Параметры отображения
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
    bool running_;
    bool show_replies_;
    bool show_plots_;
    bool show_tracks_;
    
    // SDL
    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    TTF_Font* font_{nullptr};
    bool ttf_initialized_{false};
    
    // Данные
    std::vector<ReplyData> replies_;
    std::vector<PlotData> plots_;
    std::vector<TrackData> tracks_;
    std::map<int, std::vector<TrackData>> track_history_;
};

} // namespace visualization
} // namespace vrl
