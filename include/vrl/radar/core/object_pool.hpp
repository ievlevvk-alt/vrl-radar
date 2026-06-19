// include/vrl/radar/core/object_pool.hpp
#pragma once

#include <vector>
#include <queue>
#include <memory>
#include <mutex>
#include <optional>
#include <functional>
#include <type_traits>
#include <atomic>
#include <cstring>

namespace vrl {
namespace radar {
namespace core {

// Вспомогательный трейт для проверки наличия метода reset()
template<typename T, typename = void>
struct has_reset : std::false_type {};

template<typename T>
struct has_reset<T, std::void_t<decltype(std::declval<T>().reset())>> 
    : std::true_type {};

template<typename T>
static constexpr bool has_reset_v = has_reset<T>::value;

/**
 * @brief Пул объектов для переиспользования памяти
 * 
 * Уменьшает количество аллокаций путем переиспользования объектов.
 * Потокобезопасен.
 */
template<typename T>
class ObjectPool {
public:
    /**
     * @brief Конструктор
     * @param initial_size начальный размер пула
     * @param max_size максимальный размер пула
     */
    explicit ObjectPool(size_t initial_size = 16, size_t max_size = 1024)
        : max_size_(max_size) {
        reserve(initial_size);
    }
    
    ~ObjectPool() {
        clear();
    }
    
    /**
     * @brief Зарезервировать место в пуле
     */
    void reserve(size_t count) {
        std::lock_guard<std::mutex> lock(mutex_);
        size_t current_size = pool_.size();
        for (size_t i = current_size; i < count && i < max_size_; ++i) {
            pool_.push(new T());
        }
    }
    
    /**
     * @brief Получить объект из пула
     * @return указатель на объект (nullptr если пул пуст)
     */
    T* acquire() {
        std::lock_guard<std::mutex> lock(mutex_);
        total_acquired_++;
        
        if (!pool_.empty()) {
            T* ptr = pool_.front();
            pool_.pop();
            return ptr;
        }
        
        // Если пул пуст, создаем новый объект
        total_created_++;
        return new T();
    }
    
    /**
     * @brief Вернуть объект в пул
     */
    void release(T* ptr) {
        if (!ptr) return;
        
        total_released_++;
        
        // Сбрасываем состояние объекта через reset() если есть, иначе через присваивание
        if constexpr (has_reset_v<T>) {
            ptr->reset();
        } else if constexpr (std::is_default_constructible_v<T>) {
            *ptr = T();
        }
        
        std::lock_guard<std::mutex> lock(mutex_);
        if (pool_.size() < max_size_) {
            pool_.push(ptr);
        } else {
            delete ptr;
        }
    }
    
    /**
     * @brief Размер пула
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return pool_.size();
    }
    
    /**
     * @brief Максимальный размер пула
     */
    size_t max_size() const {
        return max_size_;
    }
    
    /**
     * @brief Очистить пул
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!pool_.empty()) {
            delete pool_.front();
            pool_.pop();
        }
    }
    
    /**
     * @brief Статистика использования пула
     */
    struct Stats {
        size_t pool_size;
        size_t max_size;
        size_t total_acquired;
        size_t total_released;
        size_t total_created;
    };
    
    Stats get_stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Stats stats;
        stats.pool_size = pool_.size();
        stats.max_size = max_size_;
        stats.total_acquired = total_acquired_.load();
        stats.total_released = total_released_.load();
        stats.total_created = total_created_.load();
        return stats;
    }
    
private:
    mutable std::mutex mutex_;
    std::queue<T*> pool_;
    size_t max_size_;
    
    std::atomic<size_t> total_acquired_{0};
    std::atomic<size_t> total_released_{0};
    std::atomic<size_t> total_created_{0};
};

/**
 * @brief RAII обертка для объекта из пула
 */
template<typename T>
class PooledObject {
public:
    PooledObject(ObjectPool<T>& pool, T* ptr = nullptr)
        : pool_(pool), ptr_(ptr) {}
    
    ~PooledObject() {
        if (ptr_) {
            pool_.release(ptr_);
            ptr_ = nullptr;
        }
    }
    
    // Запрещаем копирование
    PooledObject(const PooledObject&) = delete;
    PooledObject& operator=(const PooledObject&) = delete;
    
    // Разрешаем перемещение
    PooledObject(PooledObject&& other) noexcept
        : pool_(other.pool_), ptr_(other.ptr_) {
        other.ptr_ = nullptr;
    }
    
    PooledObject& operator=(PooledObject&& other) noexcept {
        if (this != &other) {
            if (ptr_) {
                pool_.release(ptr_);
            }
            ptr_ = other.ptr_;
            other.ptr_ = nullptr;
        }
        return *this;
    }
    
    T* operator->() { return ptr_; }
    const T* operator->() const { return ptr_; }
    T& operator*() { return *ptr_; }
    const T& operator*() const { return *ptr_; }
    T* get() { return ptr_; }
    const T* get() const { return ptr_; }
    explicit operator bool() const { return ptr_ != nullptr; }
    
    void reset(T* ptr = nullptr) {
        if (ptr_) {
            pool_.release(ptr_);
        }
        ptr_ = ptr;
    }
    
private:
    ObjectPool<T>& pool_;
    T* ptr_;
};

} // namespace core
} // namespace radar
} // namespace vrl
