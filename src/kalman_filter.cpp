// file: src/kalman_filter.cpp
#include "radar/kalman_filter.h"
#include <cmath>

namespace radar {

KalmanFilter::KalmanFilter(double dt, double process_noise, double measurement_noise) 
    : last_timestamp_ms_(0), initialized_(false) {
    
    // Инициализация матриц
    update_matrices(dt);
    
    // Шум процесса (постоянная скорость)
    Q_ = Eigen::Matrix4d::Identity() * process_noise;
    Q_(0,0) = Q_(1,1) = process_noise * dt * dt;
    Q_(2,2) = Q_(3,3) = process_noise;
    
    // Шум измерений
    R_ = Eigen::Matrix2d::Identity() * measurement_noise;
}

void KalmanFilter::update_matrices(double dt) {
    // Матрица перехода
    F_ = Eigen::Matrix4d::Identity();
    F_(0,2) = dt;
    F_(1,3) = dt;
    
    // Матрица наблюдения (измеряем только позицию)
    H_ = Eigen::Matrix<double, 2, 4>::Zero();
    H_(0,0) = 1.0;
    H_(1,1) = 1.0;
}

void KalmanFilter::init(double x, double y, double timestamp_ms) {
    state_ = Eigen::Vector4d(x, y, 0.0, 0.0);
    covariance_ = Eigen::Matrix4d::Identity() * 100.0;  // Большая начальная неопределенность
    last_timestamp_ms_ = timestamp_ms;
    initialized_ = true;
}

void KalmanFilter::predict(double dt) {
    if (!initialized_) return;
    
    update_matrices(dt);
    state_ = F_ * state_;
    covariance_ = F_ * covariance_ * F_.transpose() + Q_;
}

void KalmanFilter::update(double x, double y, double timestamp_ms) {
    if (!initialized_) {
        init(x, y, timestamp_ms);
        return;
    }
    
    // Время с последнего обновления
    double dt = (timestamp_ms - last_timestamp_ms_) / 1000.0;
    if (dt > 0.01) {  // Минимальное время
        predict(dt);
    }
    
    // Инновация
    Eigen::Vector2d z(x, y);
    Eigen::Vector2d hx = H_ * state_;
    Eigen::Vector2d y_vec = z - hx;
    
    // Ковариация инновации
    Eigen::Matrix2d S = H_ * covariance_ * H_.transpose() + R_;
    
    // Калмановский коэффициент
    Eigen::Matrix<double, 4, 2> K = covariance_ * H_.transpose() * S.inverse();
    
    // Обновление состояния
    state_ = state_ + K * y_vec;
    covariance_ = (Eigen::Matrix4d::Identity() - K * H_) * covariance_;
    
    last_timestamp_ms_ = timestamp_ms;
}

std::pair<double, double> KalmanFilter::predict_position(double dt) const {
    if (!initialized_) return {0.0, 0.0};
    
    double px = state_(0) + state_(2) * dt;
    double py = state_(1) + state_(3) * dt;
    return {px, py};
}

} // namespace radar
