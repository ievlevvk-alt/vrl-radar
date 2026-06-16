// file: include/radar/kalman_filter.h
#pragma once

#include <Eigen/Dense>
#include <optional>

namespace radar {

class KalmanFilter {
public:
    // Состояние: [x, y, vx, vy]
    KalmanFilter(double dt, double process_noise, double measurement_noise);
    
    // Инициализация первым измерением
    void init(double x, double y, double timestamp_ms);
    
    // Прогноз на следующий шаг
    void predict(double dt);
    
    // Обновление по измерению
    void update(double x, double y, double timestamp_ms);
    
    // Получить текущее состояние
    Eigen::Vector4d get_state() const { return state_; }
    Eigen::Matrix4d get_covariance() const { return covariance_; }
    
    // Получить скорость
    double get_vx() const { return state_(2); }
    double get_vy() const { return state_(3); }
    double get_speed() const { return std::sqrt(state_(2)*state_(2) + state_(3)*state_(3)); }
    double get_course() const { return std::atan2(state_(2), state_(3)) * 180.0 / M_PI; }
    
    // Предсказать позицию на заданное время
    std::pair<double, double> predict_position(double dt) const;
    
private:
    Eigen::Vector4d state_;          // [x, y, vx, vy]
    Eigen::Matrix4d covariance_;     // ковариационная матрица
    
    // Матрицы модели
    Eigen::Matrix4d F_;              // матрица перехода
    Eigen::Matrix4d Q_;              // ковариация шума процесса
    Eigen::Matrix<double, 2, 4> H_;  // матрица наблюдения
    Eigen::Matrix2d R_;              // ковариация шума измерений
    
    double last_timestamp_ms_;
    bool initialized_{false};
    
    void update_matrices(double dt);
};

} // namespace radar
