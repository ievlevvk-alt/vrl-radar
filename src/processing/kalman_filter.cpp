// src/processing/kalman_filter.cpp
#include "vrl/radar/processing/kalman_filter.h"
#include <cmath>
#include <cstring>
#include <memory>   // <-- ДОБАВЛЯЕМ

namespace vrl {
namespace radar {

RevolutionKalmanFilter::RevolutionKalmanFilter() 
    : Q_(0.1), R_(1.0), initialized_(false) {
    update_matrices();
}

RevolutionKalmanFilter::RevolutionKalmanFilter(double process_noise, double measurement_noise)
    : Q_(process_noise), R_(measurement_noise), initialized_(false) {
    update_matrices();
}

void RevolutionKalmanFilter::update_matrices() {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P_[i][j] = (i == j) ? 100.0 : 0.0;
        }
    }
}

void RevolutionKalmanFilter::init(double x, double y, uint32_t revolution) {
    x_ = x;
    y_ = y;
    vx_ = 0.0;
    vy_ = 0.0;
    last_revolution_ = revolution;
    
    for (int i = 0; i < 4; ++i) {
        P_[i][i] = 100.0;
    }
    
    initialized_ = true;
}

void RevolutionKalmanFilter::reset() {
    initialized_ = false;
    x_ = 0.0;
    y_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
    last_revolution_ = 0;
    update_matrices();
}

void RevolutionKalmanFilter::predict(uint32_t delta_revolutions) {
    if (!initialized_ || delta_revolutions == 0) return;
    
    double dt = static_cast<double>(delta_revolutions);
    
    x_ = x_ + vx_ * dt;
    y_ = y_ + vy_ * dt;
    
    for (int i = 0; i < 4; ++i) {
        P_[i][i] += Q_ * dt;
    }
}

void RevolutionKalmanFilter::update(double x, double y, uint32_t revolution) {
    if (!initialized_) {
        init(x, y, revolution);
        return;
    }
    
    uint32_t delta_rev = revolution - last_revolution_;
    if (delta_rev > 0) {
        predict(delta_rev);
    }
    
    double dx = x - x_;
    double dy = y - y_;
    
    double K_x = P_[0][0] / (P_[0][0] + R_);
    double K_y = P_[1][1] / (P_[1][1] + R_);
    
    x_ = x_ + K_x * dx;
    y_ = y_ + K_y * dy;
    
    if (delta_rev > 0) {
        vx_ = (x_ - (x_ - vx_ * delta_rev)) / delta_rev;
        vy_ = (y_ - (y_ - vy_ * delta_rev)) / delta_rev;
    }
    
    P_[0][0] = (1 - K_x) * P_[0][0];
    P_[1][1] = (1 - K_y) * P_[1][1];
    
    last_revolution_ = revolution;
}

std::pair<double, double> RevolutionKalmanFilter::predict_position(uint32_t delta_revolutions) const {
    if (!initialized_) return {0.0, 0.0};
    return {x_ + vx_ * delta_revolutions, y_ + vy_ * delta_revolutions};
}

void RevolutionKalmanFilter::get_covariance(double P[4][4]) const {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P[i][j] = P_[i][j];
        }
    }
}

void RevolutionKalmanFilter::set_covariance(const double P[4][4]) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            P_[i][j] = P[i][j];
        }
    }
}

std::unique_ptr<ITrackerFilter> RevolutionKalmanFilter::clone() const {
    auto clone = std::make_unique<RevolutionKalmanFilter>(Q_, R_);
    if (initialized_) {
        clone->init(x_, y_, last_revolution_);
        clone->set_covariance(P_);
    }
    return clone;
}

} // namespace radar
} // namespace vrl
