// include/vrl/radar/processing/tracker.h
#pragma once

#include "../core/types.h"
#include "../core/replies.h"
#include "../core/config.h"
#include "i_tracker_filter.h"
#include "kalman_filter.h"
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <cstddef>
#include <array>    // <-- ДОБАВЛЯЕМ для кольцевого буфера

namespace vrl {
namespace radar {

// ============================================================================
// КОЛЬЦЕВОЙ БУФЕР ДЛЯ ИСТОРИИ ТРЕКОВ
// ============================================================================

/**
 * @brief Кольцевой буфер фиксированного размера для хранения истории треков
 * 
 * Преимущества:
 * - O(1) добавление и удаление
 * - Фиксированное использование памяти
 * - Без переаллокаций
 */
template<typename T, size_t MaxSize = 10>
class CircularHistory {
public:
    static constexpr size_t MAX_SIZE = MaxSize;
    
    CircularHistory() = default;
    
    /**
     * @brief Добавить элемент в историю
     * @param item элемент для добавления
     * @return true если элемент был добавлен
     */
    bool push(const T& item) {
        if (size_ < MAX_SIZE) {
            // Заполняем до MAX_SIZE
            data_[size_++] = item;
            return true;
        } else {
            // Перезаписываем самый старый элемент
            data_[head_] = item;
            head_ = (head_ + 1) % MAX_SIZE;
            return true;
        }
    }
    
    /**
     * @brief Добавить элемент с перемещением
     */
    bool push(T&& item) {
        if (size_ < MAX_SIZE) {
            data_[size_++] = std::move(item);
            return true;
        } else {
            data_[head_] = std::move(item);
            head_ = (head_ + 1) % MAX_SIZE;
            return true;
        }
    }
    
    /**
     * @brief Получить все элементы в порядке добавления
     */
    std::vector<T> get_all() const {
        std::vector<T> result;
        result.reserve(size_);
        
        if (size_ == 0) return result;
        
        // Идем с head_ до конца
        for (size_t i = 0; i < size_; ++i) {
            size_t idx = (head_ + i) % MAX_SIZE;
            result.push_back(data_[idx]);
        }
        
        return result;
    }
    
    /**
     * @brief Получить последний добавленный элемент
     */
    const T* back() const {
        if (size_ == 0) return nullptr;
        size_t idx = (head_ + size_ - 1) % MAX_SIZE;
        return &data_[idx];
    }
    
    /**
     * @brief Получить первый (самый старый) элемент
     */
    const T* front() const {
        if (size_ == 0) return nullptr;
        return &data_[head_];
    }
    
    /**
     * @brief Получить элемент по индексу (0 = самый старый)
     */
    const T* get(size_t index) const {
        if (index >= size_) return nullptr;
        size_t idx = (head_ + index) % MAX_SIZE;
        return &data_[idx];
    }
    
    /**
     * @brief Получить размер истории
     */
    size_t size() const { return size_; }
    
    /**
     * @brief Проверить, пуста ли история
     */
    bool empty() const { return size_ == 0; }
    
    /**
     * @brief Получить максимальный размер
     */
    constexpr size_t max_size() const { return MAX_SIZE; }
    
    /**
     * @brief Очистить историю
     */
    void clear() {
        size_ = 0;
        head_ = 0;
        // Не нужно очищать data_, она будет перезаписана
    }
    
    /**
     * @brief Проверить, заполнена ли история
     */
    bool is_full() const { return size_ == MAX_SIZE; }
    
    /**
     * @brief Итератор для поддержки range-based for
     */
    class Iterator {
    public:
        Iterator(const CircularHistory* history, size_t pos)
            : history_(history), pos_(pos) {}
        
        Iterator& operator++() {
            if (pos_ < history_->size_) {
                pos_++;
            }
            return *this;
        }
        
        const T& operator*() const {
            size_t idx = (history_->head_ + pos_) % history_->MAX_SIZE;
            return history_->data_[idx];
        }
        
        bool operator!=(const Iterator& other) const {
            return pos_ != other.pos_ || history_ != other.history_;
        }
        
    private:
        const CircularHistory* history_;
        size_t pos_;
    };
    
    Iterator begin() const { return Iterator(this, 0); }
    Iterator end() const { return Iterator(this, size_); }
    
private:
    std::array<T, MAX_SIZE> data_{};
    size_t head_{0};
    size_t size_{0};
};

// ============================================================================
// TRACK
// ============================================================================

struct Track {
    // Константы по умолчанию
    static constexpr size_t DEFAULT_MAX_HISTORY = 20;
    
    uint64_t id{0};
    TrackState state{TrackState::NEW};
    
    double x{0.0}, y{0.0};
    double azimuth_deg{0.0};
    double range_m{0.0};
    
    double vx{0.0}, vy{0.0};
    double ground_speed{0.0};
    double course_deg{0.0};
    
    uint16_t mode3a_code{0};
    uint32_t uvd_data20{0};
    uint16_t altitude{0};
    bool spi{false};
    
    uint32_t first_revolution{0};
    uint32_t last_revolution{0};
    uint32_t last_update_revolution{0};
    uint32_t coast_count{0};
    uint32_t hit_count{0};
    
    double confidence{0.0};
    double position_error{0.0};
    bool code_reliable{true};
    bool altitude_reliable{true};
    
    // ИСПРАВЛЕНО: используем кольцевой буфер вместо вектора
    CircularHistory<TargetReport, DEFAULT_MAX_HISTORY> history;
    
    /**
     * @brief Добавить отчет в историю
     * @param report отчет для добавления
     */
    void add_history(const TargetReport& report) {
        history.push(report);
    }
    
    /**
     * @brief Получить всю историю как вектор
     */
    std::vector<TargetReport> get_history() const {
        return history.get_all();
    }
    
    /**
     * @brief Получить последний отчет
     */
    const TargetReport* get_last_report() const {
        return history.back();
    }
    
    /**
     * @brief Проверить, подтвержден ли трек
     */
    bool is_confirmed() const {
        return hit_count >= 3 && state == TrackState::ACTIVE;
    }
    
    /**
     * @brief Получить размер истории
     */
    size_t history_size() const {
        return history.size();
    }
    
    /**
     * @brief Очистить историю
     */
    void clear_history() {
        history.clear();
    }
};

// ============================================================================
// TRACK MANAGER
// ============================================================================

class TrackManager {
public:
    /**
     * @brief Конструктор с конфигурацией
     * @param config конфигурация трекера
     */
    explicit TrackManager(const TrackerConfig& config = TrackerConfig());
    
    /**
     * @brief Конструктор с пользовательским фильтром
     * @param config конфигурация трекера
     * @param filter уникальный указатель на фильтр
     */
    explicit TrackManager(const TrackerConfig& config, 
                          std::unique_ptr<ITrackerFilter> filter);
    
    ~TrackManager() = default;
    
    /**
     * @brief Обработать цели на текущем обороте
     * @param targets вектор отчетов о целях
     * @param revolution номер оборота
     */
    void process_targets(const std::vector<TargetReport>& targets, uint32_t revolution);
    
    /**
     * @brief Получить активные треки
     * @return вектор треков (отсортирован по уверенности)
     */
    std::vector<Track> get_active_tracks() const;
    
    /**
     * @brief Получить подтвержденные треки
     * @return вектор подтвержденных треков
     */
    std::vector<Track> get_confirmed_tracks() const;
    
    /**
     * @brief Сбросить все треки
     */
    void reset();
    
    /**
     * @brief Включить/выключить отладочный режим
     */
    void set_debug(bool enable) { config_.debug_mode = enable; }
    
    /**
     * @brief Получить конфигурацию
     */
    const TrackerConfig& get_config() const { return config_; }
    
    /**
     * @brief Заменить фильтр на новый
     * @param filter новый фильтр
     */
    void set_filter(std::unique_ptr<ITrackerFilter> filter);
    
    /**
     * @brief Получить текущий фильтр
     */
    ITrackerFilter* get_filter() const { return default_filter_.get(); }
    
    /**
     * @brief Установить максимальный размер истории для новых треков
     * @param max_size максимальный размер истории
     */
    void set_max_history_size(size_t max_size) {
        max_history_size_ = max_size;
    }
    
    /**
     * @brief Получить максимальный размер истории
     */
    size_t get_max_history_size() const { return max_history_size_; }
    
private:
    struct TrackWithFilter {
        Track track;
        std::unique_ptr<ITrackerFilter> filter;
        
        TrackWithFilter() = default;
        TrackWithFilter(const Track& t, std::unique_ptr<ITrackerFilter> f)
            : track(t), filter(std::move(f)) {}
    };
    
    void update_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void create_new_tracks(const std::vector<TargetReport>& targets, uint32_t revolution);
    void manage_track_states(uint32_t revolution);
    double calculate_distance(const TargetReport& target, const Track& track) const;
    bool is_code_match(const TargetReport& target, const Track& track) const;
    double calculate_azimuth_diff(double az1, double az2) const;
    
    /**
     * @brief Создать новый фильтр для трека
     */
    std::unique_ptr<ITrackerFilter> create_filter() const;
    
    TrackerConfig config_;
    uint64_t next_id_{1};
    std::map<uint64_t, TrackWithFilter> tracks_;
    
    // Фильтр по умолчанию (используется для создания новых)
    std::unique_ptr<ITrackerFilter> default_filter_;
    
    // Максимальный размер истории
    size_t max_history_size_{Track::DEFAULT_MAX_HISTORY};
};

} // namespace radar
} // namespace vrl
