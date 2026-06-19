// include/vrl/radar/processing/kalman_filter.h
#pragma once

#include "i_tracker_filter.h"
#include <Eigen/Dense>
#include <optional>
#include <memory>   // <-- ДОБАВЛЯЕМ
#include <string>
#include <cmath>

namespace vrl {
namespace radar {

/**
 * @brief Фильтр Калмана для трекинга на основе оборотов антенны
 * 
 * Реализует стандартный линейный фильтр Калмана для отслеживания
 * положения и скорости цели в декартовых координатах.
 * Время измеряется в оборотах антенны.
 */
class RevolutionKalmanFilter : public ITrackerFilter {
public:
    /**
     * @brief Конструктор с параметрами по умолчанию
     */
    RevolutionKalmanFilter();
    
    /**
     * @brief Конструктор с настройками шума
     * @param process_noise шум процесса (Q)
     * @param measurement_noise шум измерений (R)
     */
    RevolutionKalmanFilter(double process_noise, double measurement_noise);
    
    // === Реализация ITrackerFilter ===
    
    void init(double x, double y, uint32_t revolution) override;
    void predict(uint32_t delta_revolutions) override;
    void update(double x, double y, uint32_t revolution) override;
    
    double get_x() const override { return x_; }
    double get_y() const override { return y_; }
    double get_vx() const override { return vx_; }
    double get_vy() const override { return vy_; }
    double get_speed() const override { return std::sqrt(vx_*vx_ + vy_*vy_); }
    double get_course() const override { return std::atan2(vx_, vy_) * 180.0 / M_PI; }
    
    std::pair<double, double> predict_position(uint32_t delta_revolutions) const override;
    bool is_initialized() const override { return initialized_; }
    void reset() override;
    
    std::unique_ptr<ITrackerFilter> clone() const override;
    std::string get_name() const override { return "RevolutionKalmanFilter"; }
    
    // === Дополнительные методы ===
    
    /**
     * @brief Получить матрицу ковариации
     */
    void get_covariance(double P[4][4]) const;
    
    /**
     * @brief Установить матрицу ковариации
     */
    void set_covariance(const double P[4][4]);
    
    /**
     * @brief Получить шум процесса
     */
    double get_process_noise() const { return Q_; }
    
    /**
     * @brief Получить шум измерений
     */
    double get_measurement_noise() const { return R_; }
    
private:
    void update_matrices();
    
    double x_{0.0}, y_{0.0};      // Положение
    double vx_{0.0}, vy_{0.0};    // Скорость
    double P_[4][4];               // Матрица ковариации
    double Q_;                     // Шум процесса
    double R_;                     // Шум измерений
    uint32_t last_revolution_{0};  // Последний оборот
    bool initialized_{false};
};

} // namespace radar
} // namespace vrl
