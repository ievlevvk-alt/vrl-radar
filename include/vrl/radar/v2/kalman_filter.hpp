// include/vrl/radar/v2/kalman_filter.hpp
#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <cmath>
#include <memory>

namespace vrl {
namespace radar {
namespace v2 {

/**
 * @brief Фильтр Калмана для сопровождения цели
 * 
 * Состояние: [x, y, vx, vy]
 * Измерение: [x, y]
 */
class KalmanFilter {
public:
    KalmanFilter() = default;
    explicit KalmanFilter(double process_noise, double measurement_noise);
    
    /**
     * @brief Инициализация фильтра
     */
    void init(double x, double y, uint64_t time_maia);
    
    /**
     * @brief Предсказание на заданное количество отсчетов
     */
    void predict(uint64_t delta_maia);
    
    /**
     * @brief Обновление по измерению
     */
    void update(double x, double y, uint64_t time_maia);
    
    /**
     * @brief Предсказание позиции на заданное количество отсчетов
     */
    std::pair<double, double> predict_position(uint64_t delta_maia) const;
    
    // Геттеры
    double get_x() const { return state_(0); }
    double get_y() const { return state_(1); }
    double get_vx() const { return state_(2); }
    double get_vy() const { return state_(3); }
    double get_speed() const { return std::sqrt(state_(2)*state_(2) + state_(3)*state_(3)); }
    double get_course() const { return std::atan2(state_(2), state_(3)) * 180.0 / M_PI; }
    
    bool is_initialized() const { return initialized_; }
    void reset();
    
    /**
     * @brief Клонирование фильтра
     */
    std::unique_ptr<KalmanFilter> clone() const;

private:
    // Матрицы состояния
    Eigen::Matrix<double, 4, 4> F_;  // Матрица перехода
    Eigen::Matrix<double, 4, 4> P_;  // Ковариация
    Eigen::Matrix<double, 4, 4> Q_;  // Шум процесса
    Eigen::Matrix<double, 2, 2> R_;  // Шум измерения
    Eigen::Matrix<double, 2, 4> H_;  // Матрица измерения
    
    Eigen::Vector4d state_;
    
    uint64_t last_time_{0};
    double revolution_time_s_{5.0};
    bool initialized_{false};
};

} // namespace v2
} // namespace radar
} // namespace vrl
