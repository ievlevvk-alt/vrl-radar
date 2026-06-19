// include/vrl/radar/processing/extended_kalman_filter.h
#pragma once

#include "i_tracker_filter.h"
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

namespace vrl {
namespace radar {

/**
 * @brief Extended Kalman Filter для нелинейной модели движения
 * 
 * Использует линеаризацию через матрицу Якоби для нелинейных моделей.
 * Поддерживает движение с постоянной скоростью и ускорением.
 */
class ExtendedKalmanFilter : public ITrackerFilter {
public:
    /**
     * @brief Конструктор с параметрами по умолчанию
     */
    ExtendedKalmanFilter();
    
    /**
     * @brief Конструктор с настройками шума
     * @param process_noise шум процесса (Q)
     * @param measurement_noise шум измерений (R)
     */
    ExtendedKalmanFilter(double process_noise, double measurement_noise);
    
    /**
     * @brief Конструктор с полной конфигурацией
     * @param process_noise шум процесса
     * @param measurement_noise шум измерений
     * @param use_acceleration использовать модель с ускорением
     */
    ExtendedKalmanFilter(double process_noise, double measurement_noise, bool use_acceleration);
    
    // === Реализация ITrackerFilter ===
    
    void init(double x, double y, uint32_t revolution) override;
    void predict(uint32_t delta_revolutions) override;
    void update(double x, double y, uint32_t revolution) override;
    
    double get_x() const override { return state_(0); }
    double get_y() const override { return state_(1); }
    double get_vx() const override { return state_(2); }
    double get_vy() const override { return state_(3); }
    double get_speed() const override;
    double get_course() const override;
    
    std::pair<double, double> predict_position(uint32_t delta_revolutions) const override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;
    
    std::unique_ptr<ITrackerFilter> clone() const override;
    std::string get_name() const override { return "ExtendedKalmanFilter"; }
    
    // === Дополнительные методы ===
    
    /**
     * @brief Получить матрицу ковариации
     */
    Eigen::Matrix<double, 6, 6> get_covariance() const { return P_; }
    
    /**
     * @brief Установить матрицу ковариации
     */
    void set_covariance(const Eigen::Matrix<double, 6, 6>& P) { P_ = P; }
    
    /**
     * @brief Получить состояние (x, y, vx, vy, ax, ay)
     */
    Eigen::Matrix<double, 6, 1> get_state() const { return state_; }
    
    /**
     * @brief Включить/выключить модель с ускорением
     */
    void set_use_acceleration(bool use) { use_acceleration_ = use; }
    
    /**
     * @brief Получить ускорение по X
     */
    double get_ax() const { return use_acceleration_ ? state_(4) : 0.0; }
    
    /**
     * @brief Получить ускорение по Y
     */
    double get_ay() const { return use_acceleration_ ? state_(5) : 0.0; }
    
private:
    /**
     * @brief Обновить матрицы перехода и шума
     */
    void update_matrices(uint32_t delta_revolutions);
    
    /**
     * @brief Вычислить матрицу Якоби для функции измерения
     */
    Eigen::Matrix<double, 2, 6> compute_measurement_jacobian() const;
    
    /**
     * @brief Нелинейная функция измерения
     */
    Eigen::Matrix<double, 2, 1> measurement_function() const;
    
    /**
     * @brief Предсказание для модели с постоянной скоростью
     */
    void predict_constant_velocity(uint32_t delta_revolutions);
    
    /**
     * @brief Предсказание для модели с постоянным ускорением
     */
    void predict_constant_acceleration(uint32_t delta_revolutions);
    
    Eigen::Matrix<double, 6, 1> state_;      // [x, y, vx, vy, ax, ay]
    Eigen::Matrix<double, 6, 6> P_;          // Ковариация
    Eigen::Matrix<double, 6, 6> Q_;          // Шум процесса
    Eigen::Matrix<double, 2, 2> R_;          // Шум измерений
    Eigen::Matrix<double, 2, 6> H_;          // Матрица Якоби
    
    double process_noise_;
    double measurement_noise_;
    bool use_acceleration_{false};
    bool initialized_{false};
    uint32_t last_revolution_{0};
    
    // Константы
    static constexpr double MAX_SPEED = 1000.0;   // м/с
    static constexpr double MIN_SPEED = 0.001;    // м/с
    static constexpr double MAX_ACCELERATION = 50.0; // м/с²
};

} // namespace radar
} // namespace vrl
