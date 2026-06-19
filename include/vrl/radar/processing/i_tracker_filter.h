// include/vrl/radar/processing/i_tracker_filter.h
#pragma once

#include <cstdint>
#include <memory>   // <-- ДОБАВЛЯЕМ
#include <string>   // <-- ДОБАВЛЯЕМ
#include <utility>

namespace vrl {
namespace radar {

/**
 * @brief Интерфейс для фильтров трекинга
 * 
 * Определяет контракт для всех реализаций фильтров, используемых в TrackManager.
 * Позволяет легко подменять реализацию фильтра (Kalman, EKF, UKF, Particle и т.д.)
 */
class ITrackerFilter {
public:
    virtual ~ITrackerFilter() = default;
    
    /**
     * @brief Инициализировать фильтр начальным положением
     * @param x координата X в метрах
     * @param y координата Y в метрах
     * @param revolution номер оборота антенны
     */
    virtual void init(double x, double y, uint32_t revolution) = 0;
    
    /**
     * @brief Предсказать состояние на delta_revolutions вперед
     * @param delta_revolutions количество оборотов
     */
    virtual void predict(uint32_t delta_revolutions) = 0;
    
    /**
     * @brief Обновить состояние по новому измерению
     * @param x координата X в метрах
     * @param y координата Y в метрах
     * @param revolution номер оборота антенны
     */
    virtual void update(double x, double y, uint32_t revolution) = 0;
    
    /**
     * @brief Получить текущую оценку положения X
     */
    virtual double get_x() const = 0;
    
    /**
     * @brief Получить текущую оценку положения Y
     */
    virtual double get_y() const = 0;
    
    /**
     * @brief Получить текущую оценку скорости по X
     */
    virtual double get_vx() const = 0;
    
    /**
     * @brief Получить текущую оценку скорости по Y
     */
    virtual double get_vy() const = 0;
    
    /**
     * @brief Получить текущую оценку скорости (модуль)
     */
    virtual double get_speed() const = 0;
    
    /**
     * @brief Получить текущий курс в градусах
     */
    virtual double get_course() const = 0;
    
    /**
     * @brief Предсказать положение через delta_revolutions
     * @param delta_revolutions количество оборотов вперед
     * @return пара (x, y) в метрах
     */
    virtual std::pair<double, double> predict_position(uint32_t delta_revolutions) const = 0;
    
    /**
     * @brief Проверить, инициализирован ли фильтр
     */
    virtual bool is_initialized() const = 0;
    
    /**
     * @brief Сбросить фильтр в начальное состояние
     */
    virtual void reset() = 0;
    
    /**
     * @brief Создать клон фильтра
     * @return unique_ptr с копией фильтра
     */
    virtual std::unique_ptr<ITrackerFilter> clone() const = 0;
    
    /**
     * @brief Получить имя фильтра для логирования
     */
    virtual std::string get_name() const = 0;
};

} // namespace radar
} // namespace vrl
