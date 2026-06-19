// include/vrl/radar/processing/unscented_kalman_filter.h
#pragma once

#include "i_tracker_filter.h"
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace vrl {
namespace radar {

/**
 * @brief Unscented Kalman Filter для нелинейной модели движения
 * 
 * Использует сигма-точки для более точной аппроксимации нелинейностей.
 * Не требует вычисления матрицы Якоби.
 */
class UnscentedKalmanFilter : public ITrackerFilter {
public:
    /**
     * @brief Конструктор с параметрами по умолчанию
     */
    UnscentedKalmanFilter();
    
    /**
     * @brief Конструктор с настройками шума
     * @param process_noise шум процесса (Q)
     * @param measurement_noise шум измерений (R)
     */
    UnscentedKalmanFilter(double process_noise, double measurement_noise);
    
    /**
     * @brief Конструктор с полной конфигурацией
     * @param process_noise шум процесса
     * @param measurement_noise шум измерений
     * @param alpha параметр распределения сигма-точек (обычно 0.001 - 1)
     * @param beta параметр для распределения (обычно 2 для гауссова)
     * @param kappa параметр масштабирования (обычно 0)
     */
    UnscentedKalmanFilter(double process_noise, double measurement_noise, 
                          double alpha, double beta, double kappa);
    
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
    std::string get_name() const override { return "UnscentedKalmanFilter"; }
    
    // === Дополнительные методы ===
    
    /**
     * @brief Получить матрицу ковариации
     */
    Eigen::Matrix<double, 5, 5> get_covariance() const { return P_; }
    
    /**
     * @brief Установить матрицу ковариации
     */
    void set_covariance(const Eigen::Matrix<double, 5, 5>& P) { P_ = P; }
    
    /**
     * @brief Получить состояние (x, y, vx, vy, speed)
     */
    Eigen::Matrix<double, 5, 1> get_state() const { return state_; }
    
    /**
     * @brief Получить параметры UKF
     */
    void get_params(double& alpha, double& beta, double& kappa) const;
    
private:
    /**
     * @brief Сгенерировать сигма-точки
     */
    void generate_sigma_points();
    
    /**
     * @brief Предсказать сигма-точки через нелинейную функцию
     */
    void predict_sigma_points(uint32_t delta_revolutions);
    
    /**
     * @brief Вычислить предсказанное среднее и ковариацию
     */
    void compute_predicted_mean_covariance();
    
    /**
     * @brief Вычислить сигма-точки для измерения
     */
    void compute_measurement_sigma_points();
    
    /**
     * @brief Обновить состояние с использованием измерения
     */
    void update_with_measurement(double x, double y);
    
    /**
     * @brief Нелинейная функция перехода состояния
     */
    Eigen::Matrix<double, 5, 1> state_transition_function(
        const Eigen::Matrix<double, 5, 1>& state, 
        uint32_t delta_revolutions) const;
    
    /**
     * @brief Нелинейная функция измерения
     */
    Eigen::Matrix<double, 2, 1> measurement_function(
        const Eigen::Matrix<double, 5, 1>& state) const;
    
    Eigen::Matrix<double, 5, 1> state_;      // [x, y, vx, vy, speed]
    Eigen::Matrix<double, 5, 5> P_;          // Ковариация
    Eigen::Matrix<double, 5, 5> Q_;          // Шум процесса
    Eigen::Matrix<double, 2, 2> R_;          // Шум измерений
    
    // Параметры UKF
    double alpha_{0.001};
    double beta_{2.0};
    double kappa_{0.0};
    double lambda_;
    double weight_mean_;
    double weight_covariance_;
    
    // Сигма-точки
    std::vector<Eigen::Matrix<double, 5, 1>> sigma_points_;
    std::vector<Eigen::Matrix<double, 5, 1>> predicted_sigma_points_;
    std::vector<Eigen::Matrix<double, 2, 1>> measurement_sigma_points_;
    
    // Предсказанные значения
    Eigen::Matrix<double, 5, 1> predicted_state_;
    Eigen::Matrix<double, 5, 5> predicted_covariance_;
    Eigen::Matrix<double, 2, 1> predicted_measurement_;
    Eigen::Matrix<double, 2, 2> measurement_covariance_;
    Eigen::Matrix<double, 5, 2> cross_covariance_;
    
    double process_noise_;
    double measurement_noise_;
    bool initialized_{false};
    uint32_t last_revolution_{0};
    
    // Константы
    static constexpr int STATE_DIM = 5;
    static constexpr int MEAS_DIM = 2;
    static constexpr int NUM_SIGMA_POINTS = 2 * STATE_DIM + 1;
};

} // namespace radar
} // namespace vrl
