// include/vrl/radar/processing/kalman_filter.h
#pragma once

#include <Eigen/Dense>
#include <optional>

namespace vrl {
namespace radar {

// ============================================================================
// KALMAN FILTER FOR REVOLUTION-BASED TRACKING
// ============================================================================

class RevolutionKalmanFilter {
public:
    RevolutionKalmanFilter();
    RevolutionKalmanFilter(double process_noise, double measurement_noise);
    
    void init(double x, double y, uint32_t revolution);
    void predict(uint32_t delta_revolutions);
    void update(double x, double y, uint32_t revolution);
    
    double get_x() const { return x_; }
    double get_y() const { return y_; }
    double get_vx() const { return vx_; }
    double get_vy() const { return vy_; }
    double get_speed() const { return std::sqrt(vx_*vx_ + vy_*vy_); }
    double get_course() const { return std::atan2(vx_, vy_) * 180.0 / M_PI; }
    
    std::pair<double, double> predict_position(uint32_t delta_revolutions) const;
    bool is_initialized() const { return initialized_; }
    
private:
    double x_{0.0}, y_{0.0};
    double vx_{0.0}, vy_{0.0};
    double P_[4][4];
    double Q_;
    double R_;
    uint32_t last_revolution_{0};
    bool initialized_{false};
    
    void update_matrices();
};

} // namespace radar
} // namespace vrl
