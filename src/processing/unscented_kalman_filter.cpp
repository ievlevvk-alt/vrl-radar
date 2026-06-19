// src/processing/unscented_kalman_filter.cpp
#include "vrl/radar/processing/unscented_kalman_filter.h"
#include <cmath>
#include <iostream>
#include <limits>

namespace vrl {
namespace radar {

UnscentedKalmanFilter::UnscentedKalmanFilter() 
    : process_noise_(0.1), measurement_noise_(1.0) {
    lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
    weight_mean_ = lambda_ / (STATE_DIM + lambda_);
    weight_covariance_ = lambda_ / (STATE_DIM + lambda_) + (1 - alpha_ * alpha_ + beta_);
    
    state_.setZero();
    P_.setIdentity() * 100.0;
    Q_.setIdentity() * process_noise_;
    R_.setIdentity() * measurement_noise_;
    initialized_ = false;
    
    sigma_points_.resize(NUM_SIGMA_POINTS);
    predicted_sigma_points_.resize(NUM_SIGMA_POINTS);
    measurement_sigma_points_.resize(NUM_SIGMA_POINTS);
}

UnscentedKalmanFilter::UnscentedKalmanFilter(double process_noise, double measurement_noise)
    : process_noise_(process_noise), measurement_noise_(measurement_noise) {
    lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
    weight_mean_ = lambda_ / (STATE_DIM + lambda_);
    weight_covariance_ = lambda_ / (STATE_DIM + lambda_) + (1 - alpha_ * alpha_ + beta_);
    
    state_.setZero();
    P_.setIdentity() * 100.0;
    Q_.setIdentity() * process_noise_;
    R_.setIdentity() * measurement_noise_;
    initialized_ = false;
    
    sigma_points_.resize(NUM_SIGMA_POINTS);
    predicted_sigma_points_.resize(NUM_SIGMA_POINTS);
    measurement_sigma_points_.resize(NUM_SIGMA_POINTS);
}

UnscentedKalmanFilter::UnscentedKalmanFilter(double process_noise, double measurement_noise,
                                             double alpha, double beta, double kappa)
    : process_noise_(process_noise), measurement_noise_(measurement_noise),
      alpha_(alpha), beta_(beta), kappa_(kappa) {
    lambda_ = alpha_ * alpha_ * (STATE_DIM + kappa_) - STATE_DIM;
    weight_mean_ = lambda_ / (STATE_DIM + lambda_);
    weight_covariance_ = lambda_ / (STATE_DIM + lambda_) + (1 - alpha_ * alpha_ + beta_);
    
    state_.setZero();
    P_.setIdentity() * 100.0;
    Q_.setIdentity() * process_noise_;
    R_.setIdentity() * measurement_noise_;
    initialized_ = false;
    
    sigma_points_.resize(NUM_SIGMA_POINTS);
    predicted_sigma_points_.resize(NUM_SIGMA_POINTS);
    measurement_sigma_points_.resize(NUM_SIGMA_POINTS);
}

void UnscentedKalmanFilter::init(double x, double y, uint32_t revolution) {
    state_.setZero();
    state_(0) = x;
    state_(1) = y;
    
    P_.setIdentity() * 100.0;
    last_revolution_ = revolution;
    initialized_ = true;
}

void UnscentedKalmanFilter::reset() {
    initialized_ = false;
    state_.setZero();
    P_.setIdentity() * 100.0;
    last_revolution_ = 0;
}

void UnscentedKalmanFilter::generate_sigma_points() {
    const double EPS = 1e-9;
    
    // Проверяем, что P положительно определена
    for (int i = 0; i < STATE_DIM; ++i) {
        if (P_(i, i) < EPS) {
            P_(i, i) = EPS;
        }
    }
    
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_stable = P_;
    for (int i = 0; i < STATE_DIM; ++i) {
        P_stable(i, i) += EPS;
    }
    
    Eigen::LLT<Eigen::Matrix<double, STATE_DIM, STATE_DIM>> llt(P_stable);
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> sqrt_P = llt.matrixL();
    
    sigma_points_[0] = state_;
    
    double sqrt_factor = std::sqrt(STATE_DIM + lambda_);
    for (int i = 0; i < STATE_DIM; ++i) {
        sigma_points_[i + 1] = state_ + sqrt_factor * sqrt_P.col(i);
        sigma_points_[i + 1 + STATE_DIM] = state_ - sqrt_factor * sqrt_P.col(i);
    }
}

void UnscentedKalmanFilter::predict_sigma_points(uint32_t delta_revolutions) {
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        predicted_sigma_points_[i] = state_transition_function(sigma_points_[i], delta_revolutions);
    }
}

Eigen::Matrix<double, 5, 1> UnscentedKalmanFilter::state_transition_function(
    const Eigen::Matrix<double, 5, 1>& state, 
    uint32_t delta_revolutions) const {
    
    double dt = static_cast<double>(delta_revolutions);
    Eigen::Matrix<double, 5, 1> new_state = state;
    
    new_state(0) = state(0) + state(2) * dt;
    new_state(1) = state(1) + state(3) * dt;
    
    return new_state;
}

void UnscentedKalmanFilter::compute_predicted_mean_covariance() {
    predicted_state_.setZero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        predicted_state_ += weight_mean_ * predicted_sigma_points_[i];
    }
    
    predicted_covariance_.setZero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::Matrix<double, STATE_DIM, 1> diff = predicted_sigma_points_[i] - predicted_state_;
        double weight = (i == 0) ? weight_covariance_ : weight_mean_;
        predicted_covariance_ += weight * diff * diff.transpose();
    }
    predicted_covariance_ += Q_;
}

void UnscentedKalmanFilter::compute_measurement_sigma_points() {
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        measurement_sigma_points_[i] = measurement_function(predicted_sigma_points_[i]);
    }
}

Eigen::Matrix<double, 2, 1> UnscentedKalmanFilter::measurement_function(
    const Eigen::Matrix<double, 5, 1>& state) const {
    
    Eigen::Matrix<double, 2, 1> meas;
    meas(0) = state(0);
    meas(1) = state(1);
    return meas;
}

void UnscentedKalmanFilter::update_with_measurement(double x, double y) {
    const double EPS = 1e-9;
    
    predicted_measurement_.setZero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        predicted_measurement_ += weight_mean_ * measurement_sigma_points_[i];
    }
    
    measurement_covariance_.setZero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::Matrix<double, MEAS_DIM, 1> diff = measurement_sigma_points_[i] - predicted_measurement_;
        double weight = (i == 0) ? weight_covariance_ : weight_mean_;
        measurement_covariance_ += weight * diff * diff.transpose();
    }
    measurement_covariance_ += R_;
    
    // Регуляризация
    for (int i = 0; i < MEAS_DIM; ++i) {
        if (measurement_covariance_(i, i) < EPS) {
            measurement_covariance_(i, i) = EPS;
        }
    }
    
    cross_covariance_.setZero();
    for (int i = 0; i < NUM_SIGMA_POINTS; ++i) {
        Eigen::Matrix<double, STATE_DIM, 1> diff_state = predicted_sigma_points_[i] - predicted_state_;
        Eigen::Matrix<double, MEAS_DIM, 1> diff_meas = measurement_sigma_points_[i] - predicted_measurement_;
        double weight = (i == 0) ? weight_covariance_ : weight_mean_;
        cross_covariance_ += weight * diff_state * diff_meas.transpose();
    }
    
    // Проверяем, что measurement_covariance не сингулярна
    double det = measurement_covariance_.determinant();
    if (std::abs(det) < EPS) {
        // Добавляем регуляризацию
        for (int i = 0; i < MEAS_DIM; ++i) {
            measurement_covariance_(i, i) += 0.01;
        }
    }
    
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = cross_covariance_ * measurement_covariance_.inverse();
    
    Eigen::Matrix<double, MEAS_DIM, 1> z;
    z(0) = x;
    z(1) = y;
    Eigen::Matrix<double, MEAS_DIM, 1> innovation = z - predicted_measurement_;
    
    state_ = predicted_state_ + K * innovation;
    
    // Обновление ковариации с гарантией положительной определенности
    P_ = predicted_covariance_ - K * measurement_covariance_ * K.transpose();
    for (int i = 0; i < STATE_DIM; ++i) {
        if (P_(i, i) < EPS) {
            P_(i, i) = EPS;
        }
    }
}

void UnscentedKalmanFilter::predict(uint32_t delta_revolutions) {
    if (!initialized_ || delta_revolutions == 0) return;
    
    generate_sigma_points();
    predict_sigma_points(delta_revolutions);
    compute_predicted_mean_covariance();
    
    state_ = predicted_state_;
    P_ = predicted_covariance_;
    
    last_revolution_ += delta_revolutions;
}

void UnscentedKalmanFilter::update(double x, double y, uint32_t revolution) {
    if (!initialized_) {
        init(x, y, revolution);
        return;
    }
    
    uint32_t delta_rev = revolution - last_revolution_;
    if (delta_rev > 0) {
        predict(delta_rev);
    }
    
    compute_measurement_sigma_points();
    update_with_measurement(x, y);
    
    last_revolution_ = revolution;
}

double UnscentedKalmanFilter::get_speed() const {
    double vx = state_(2);
    double vy = state_(3);
    double speed = std::sqrt(vx * vx + vy * vy);
    if (std::isnan(speed) || speed < 0.001) {
        return 0.0;
    }
    return speed;
}

double UnscentedKalmanFilter::get_course() const {
    double vx = state_(2);
    double vy = state_(3);
    double speed = get_speed();
    if (speed < 0.001) return 0.0;
    double course = std::atan2(vx, vy) * 180.0 / M_PI;
    if (course < 0) course += 360.0;
    return course;
}

std::pair<double, double> UnscentedKalmanFilter::predict_position(uint32_t delta_revolutions) const {
    if (!initialized_) return {0.0, 0.0};
    
    double dt = static_cast<double>(delta_revolutions);
    double x = state_(0) + state_(2) * dt;
    double y = state_(1) + state_(3) * dt;
    return {x, y};
}

std::unique_ptr<ITrackerFilter> UnscentedKalmanFilter::clone() const {
    auto clone = std::make_unique<UnscentedKalmanFilter>(process_noise_, measurement_noise_, 
                                                          alpha_, beta_, kappa_);
    if (initialized_) {
        clone->state_ = state_;
        clone->P_ = P_;
        clone->last_revolution_ = last_revolution_;
        clone->initialized_ = true;
    }
    return clone;
}

void UnscentedKalmanFilter::get_params(double& alpha, double& beta, double& kappa) const {
    alpha = alpha_;
    beta = beta_;
    kappa = kappa_;
}

} // namespace radar
} // namespace vrl
