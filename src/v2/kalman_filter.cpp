// src/v2/kalman_filter.cpp
#include "vrl/radar/v2/kalman_filter.hpp"

namespace vrl {
namespace radar {
namespace v2 {

KalmanFilter::KalmanFilter(double process_noise, double measurement_noise) {
    F_ = Eigen::Matrix<double, 4, 4>::Identity();
    P_ = Eigen::Matrix<double, 4, 4>::Identity() * 1000.0;
    Q_ = Eigen::Matrix<double, 4, 4>::Identity() * process_noise;
    R_ = Eigen::Matrix<double, 2, 2>::Identity() * measurement_noise;
    
    H_ = Eigen::Matrix<double, 2, 4>::Zero();
    H_(0, 0) = 1.0;
    H_(1, 1) = 1.0;
    
    state_ = Eigen::Vector4d::Zero();
}

void KalmanFilter::init(double x, double y, uint64_t time_maia) {
    state_(0) = x;
    state_(1) = y;
    state_(2) = 0.0;
    state_(3) = 0.0;
    
    P_ = Eigen::Matrix<double, 4, 4>::Identity() * 1000.0;
    last_time_ = time_maia;
    initialized_ = true;
}

void KalmanFilter::predict(uint64_t delta_maia) {
    if (!initialized_) return;
    
    double dt = static_cast<double>(delta_maia) / 4096.0 * revolution_time_s_;
    
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    state_ = F_ * state_;
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(double x, double y, uint64_t time_maia) {
    if (!initialized_) {
        init(x, y, time_maia);
        return;
    }
    
    uint64_t delta = time_maia - last_time_;
    if (delta > 0) {
        predict(delta);
    }
    
    Eigen::Vector2d z;
    z(0) = x;
    z(1) = y;
    
    Eigen::Vector2d y_innov = z - H_ * state_;
    Eigen::Matrix<double, 2, 2> S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 4, 2> K = P_ * H_.transpose() * S.inverse();
    
    state_ = state_ + K * y_innov;
    P_ = (Eigen::Matrix<double, 4, 4>::Identity() - K * H_) * P_;
    
    last_time_ = time_maia;
}

std::pair<double, double> KalmanFilter::predict_position(uint64_t delta_maia) const {
    if (!initialized_) {
        return {0.0, 0.0};
    }
    
    double dt = static_cast<double>(delta_maia) / 4096.0 * revolution_time_s_;
    return {state_(0) + state_(2) * dt, state_(1) + state_(3) * dt};
}

void KalmanFilter::reset() {
    state_.setZero();
    P_ = Eigen::Matrix<double, 4, 4>::Identity() * 1000.0;
    last_time_ = 0;
    initialized_ = false;
}

std::unique_ptr<KalmanFilter> KalmanFilter::clone() const {
    auto clone = std::make_unique<KalmanFilter>();
    if (initialized_) {
        clone->state_ = state_;
        clone->P_ = P_;
        clone->last_time_ = last_time_;
        clone->initialized_ = true;
    }
    return clone;
}

} // namespace v2
} // namespace radar
} // namespace vrl
