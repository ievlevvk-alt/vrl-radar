// src/processing/extended_kalman_filter.cpp
#include "vrl/radar/processing/extended_kalman_filter.h"
#include <cmath>
#include <iostream>

namespace vrl {
namespace radar {

ExtendedKalmanFilter::ExtendedKalmanFilter() 
    : process_noise_(0.1), measurement_noise_(1.0), use_acceleration_(false) {
    state_.setZero();
    P_.setIdentity();
    Q_.setIdentity();
    R_.setIdentity();
    H_.setZero();
    initialized_ = false;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(double process_noise, double measurement_noise)
    : process_noise_(process_noise), measurement_noise_(measurement_noise), use_acceleration_(false) {
    state_.setZero();
    P_.setIdentity() * 100.0;
    Q_.setIdentity() * process_noise;
    R_.setIdentity() * measurement_noise;
    H_.setZero();
    initialized_ = false;
}

ExtendedKalmanFilter::ExtendedKalmanFilter(double process_noise, double measurement_noise, bool use_acceleration)
    : process_noise_(process_noise), measurement_noise_(measurement_noise), use_acceleration_(use_acceleration) {
    state_.setZero();
    P_.setIdentity() * 100.0;
    Q_.setIdentity() * process_noise;
    R_.setIdentity() * measurement_noise;
    H_.setZero();
    initialized_ = false;
}

void ExtendedKalmanFilter::init(double x, double y, uint32_t revolution) {
    state_.setZero();
    state_(0) = x;
    state_(1) = y;
    
    P_.setIdentity() * 100.0;
    last_revolution_ = revolution;
    initialized_ = true;
}

void ExtendedKalmanFilter::reset() {
    initialized_ = false;
    state_.setZero();
    P_.setIdentity() * 100.0;
    last_revolution_ = 0;
}

void ExtendedKalmanFilter::update_matrices(uint32_t delta_revolutions) {
    double dt = static_cast<double>(delta_revolutions);
    
    if (use_acceleration_) {
        // Модель с постоянным ускорением: [x, y, vx, vy, ax, ay]
        // F = [1, 0, dt, 0, 0.5*dt², 0]
        //     [0, 1, 0, dt, 0, 0.5*dt²]
        //     [0, 0, 1, 0, dt, 0]
        //     [0, 0, 0, 1, 0, dt]
        //     [0, 0, 0, 0, 1, 0]
        //     [0, 0, 0, 0, 0, 1]
        
        Eigen::Matrix<double, 6, 6> F;
        F.setIdentity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        F(0, 4) = 0.5 * dt * dt;
        F(1, 5) = 0.5 * dt * dt;
        F(2, 4) = dt;
        F(3, 5) = dt;
        
        // Q для модели с ускорением
        Eigen::Matrix<double, 6, 6> Q;
        Q.setZero();
        Q(0, 0) = process_noise_ * dt * dt * dt / 3.0;
        Q(1, 1) = process_noise_ * dt * dt * dt / 3.0;
        Q(2, 2) = process_noise_ * dt;
        Q(3, 3) = process_noise_ * dt;
        Q(4, 4) = process_noise_ * dt * dt;
        Q(5, 5) = process_noise_ * dt * dt;
        
        // Предсказание
        state_ = F * state_;
        P_ = F * P_ * F.transpose() + Q;
        
        // Обновляем H для измерения [x, y]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        
    } else {
        // Модель с постоянной скоростью: [x, y, vx, vy]
        Eigen::Matrix<double, 4, 4> F;
        F.setIdentity();
        F(0, 2) = dt;
        F(1, 3) = dt;
        
        Eigen::Matrix<double, 4, 4> Q;
        Q.setZero();
        Q(0, 0) = process_noise_ * dt * dt * dt / 3.0;
        Q(1, 1) = process_noise_ * dt * dt * dt / 3.0;
        Q(2, 2) = process_noise_ * dt;
        Q(3, 3) = process_noise_ * dt;
        
        // Для 6-мерного состояния используем только первые 4 компонента
        Eigen::Matrix<double, 6, 6> F_full;
        F_full.setIdentity();
        F_full(0, 2) = dt;
        F_full(1, 3) = dt;
        
        Eigen::Matrix<double, 6, 6> Q_full;
        Q_full.setZero();
        Q_full(0, 0) = process_noise_ * dt * dt * dt / 3.0;
        Q_full(1, 1) = process_noise_ * dt * dt * dt / 3.0;
        Q_full(2, 2) = process_noise_ * dt;
        Q_full(3, 3) = process_noise_ * dt;
        
        // Предсказание
        state_ = F_full * state_;
        P_ = F_full * P_ * F_full.transpose() + Q_full;
        
        // Обновляем H для измерения [x, y]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
    }
}

void ExtendedKalmanFilter::predict(uint32_t delta_revolutions) {
    if (!initialized_ || delta_revolutions == 0) return;
    update_matrices(delta_revolutions);
    last_revolution_ += delta_revolutions;
}

void ExtendedKalmanFilter::update(double x, double y, uint32_t revolution) {
    if (!initialized_) {
        init(x, y, revolution);
        return;
    }
    
    uint32_t delta_rev = revolution - last_revolution_;
    if (delta_rev > 0) {
        predict(delta_rev);
    }
    
    // Измерение
    Eigen::Matrix<double, 2, 1> z;
    z(0) = x;
    z(1) = y;
    
    // Инновация
    Eigen::Matrix<double, 2, 1> y_innov = z - H_ * state_;
    
    // Ковариация инновации
    Eigen::Matrix<double, 2, 2> S = H_ * P_ * H_.transpose() + R_;
    
    // Калмановское усиление
    Eigen::Matrix<double, 6, 2> K = P_ * H_.transpose() * S.inverse();
    
    // Обновление состояния
    state_ = state_ + K * y_innov;
    
    // Обновление ковариации
    P_ = (Eigen::Matrix<double, 6, 6>::Identity() - K * H_) * P_;
    
    // Ограничение скорости
    double vx = state_(2);
    double vy = state_(3);
    double speed = std::sqrt(vx * vx + vy * vy);
    if (speed > MAX_SPEED) {
        state_(2) = state_(2) / speed * MAX_SPEED;
        state_(3) = state_(3) / speed * MAX_SPEED;
    }
    
    // Ограничение ускорения
    if (use_acceleration_) {
        double ax = state_(4);
        double ay = state_(5);
        double acc = std::sqrt(ax * ax + ay * ay);
        if (acc > MAX_ACCELERATION) {
            state_(4) = state_(4) / acc * MAX_ACCELERATION;
            state_(5) = state_(5) / acc * MAX_ACCELERATION;
        }
    }
    
    last_revolution_ = revolution;
}

double ExtendedKalmanFilter::get_speed() const {
    double vx = state_(2);
    double vy = state_(3);
    return std::sqrt(vx * vx + vy * vy);
}

double ExtendedKalmanFilter::get_course() const {
    double vx = state_(2);
    double vy = state_(3);
    double course = std::atan2(vx, vy) * 180.0 / M_PI;
    if (course < 0) course += 360.0;
    return course;
}

std::pair<double, double> ExtendedKalmanFilter::predict_position(uint32_t delta_revolutions) const {
    if (!initialized_) return {0.0, 0.0};
    
    double dt = static_cast<double>(delta_revolutions);
    double x = state_(0) + state_(2) * dt;
    double y = state_(1) + state_(3) * dt;
    
    if (use_acceleration_) {
        x += 0.5 * state_(4) * dt * dt;
        y += 0.5 * state_(5) * dt * dt;
    }
    
    return {x, y};
}

std::unique_ptr<ITrackerFilter> ExtendedKalmanFilter::clone() const {
    auto clone = std::make_unique<ExtendedKalmanFilter>(process_noise_, measurement_noise_, use_acceleration_);
    if (initialized_) {
        clone->state_ = state_;
        clone->P_ = P_;
        clone->last_revolution_ = last_revolution_;
        clone->initialized_ = true;
    }
    return clone;
}

} // namespace radar
} // namespace vrl
