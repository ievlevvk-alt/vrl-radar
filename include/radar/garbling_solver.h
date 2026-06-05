// file: include/radar/garbling_solver.h
#pragma once

#include "replies.h"
#include "types.h"
#include <vector>
#include <array>
#include <optional>
#include <bitset>
#include <memory>  // Добавлено для std::unique_ptr
#include <string>

namespace radar {

// Результат разделения перекрытия
template<typename ReplyType>
struct SeparationResult {
    std::vector<ReplyType> separated_replies;  // разделенные ответы
    double confidence{0.0};                      // уверенность в результате (0-1)
    bool ambiguous{false};                       // неоднозначный результат
    std::string method_used;                      // использованный метод
};

// Базовый класс для алгоритмов разделения
class GarblingSolver {
public:
    GarblingSolver(const RadarConfig& config);
    virtual ~GarblingSolver() = default;
    
    // Основные методы разделения
    virtual SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) = 0;
    
    virtual SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) = 0;
    
    // Вспомогательные методы
    void set_debug(bool enable) { debug_ = enable; }
    
protected:
    RadarConfig config_;
    bool debug_{false};
    
    // Логирование для отладки
    void log(const std::string& msg) const;
};

// Реализация 1: Амплитудный анализ с порогами
class ThresholdGarblingSolver : public GarblingSolver {
public:
    ThresholdGarblingSolver(const RadarConfig& config, uint8_t threshold = 50);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
private:
    uint8_t threshold_;
    
    // Вспомогательные методы для RBS
    std::vector<uint16_t> detect_possible_codes_rbs(
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture);
    
    bool check_code_presence_rbs(
        uint16_t code,
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& mixture);
    
    // Вспомогательные методы для UVD
    std::vector<uint32_t> detect_possible_data_uvd(
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& mixture);
};

// Реализация 2: Корреляционный анализ с шаблонами
class CorrelationGarblingSolver : public GarblingSolver {
public:
    CorrelationGarblingSolver(const RadarConfig& config);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
    // Загрузить библиотеку известных кодов
    void load_code_library(const std::vector<uint16_t>& codes);
    void load_data_library(const std::vector<uint32_t>& data);
    
private:
    std::vector<uint16_t> known_codes_;
    std::vector<uint32_t> known_data_;
    
    // Шаблоны для RBS (18 позиций)
    std::array<uint8_t, RBSReply::ETHER_POSITIONS> create_template_rbs(uint16_t code, bool spi = false);
    
    // Шаблоны для UVD (80 позиций)
    std::array<uint8_t, UVDReply::ETHER_POSITIONS> create_template_uvd(uint32_t data);
    
    // Вычисление корреляции
    double compute_correlation(
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& signal,
        const std::array<uint8_t, RBSReply::ETHER_POSITIONS>& pattern);
    
    double compute_correlation(
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& signal,
        const std::array<uint8_t, UVDReply::ETHER_POSITIONS>& pattern);
};

// Реализация 3: Итеративное вычитание (CLEAN алгоритм)
class IterativeSubtractionSolver : public GarblingSolver {
public:
    IterativeSubtractionSolver(const RadarConfig& config, 
                               double min_amplitude_ratio = 0.3,
                               int max_iterations = 5);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
private:
    double min_amplitude_ratio_;
    int max_iterations_;
    
    // Найти доминирующий ответ в смеси
    template<typename ReplyType>
    std::optional<ReplyType> find_dominant_reply(
        const std::vector<ReplyType>& mixture);
    
    // Вычесть ответ из смеси
    template<typename ReplyType>
    std::vector<ReplyType> subtract_reply(
        const std::vector<ReplyType>& mixture,
        const ReplyType& to_subtract);
    
    // Оценить качество разделения
    double evaluate_separation_quality(
        const std::vector<RBSReply>& original,
        const std::vector<RBSReply>& separated);
};

// Комбинированный решатель (использует все методы)
class CompositeGarblingSolver : public GarblingSolver {
public:
    CompositeGarblingSolver(const RadarConfig& config);
    
    // Добавить метод с приоритетом
    void add_method(std::unique_ptr<GarblingSolver> method, int priority);
    
    SeparationResult<RBSReply> separate_rbs(
        const std::vector<RBSReply>& mixture,
        const std::vector<uint16_t>& expected_codes = {}) override;
    
    SeparationResult<UVDReply> separate_uvd(
        const std::vector<UVDReply>& mixture,
        const std::vector<uint32_t>& expected_data = {}) override;
    
private:
    struct MethodWithPriority {
        std::unique_ptr<GarblingSolver> method;
        int priority;
    };
    
    std::vector<MethodWithPriority> methods_;
    
    // Выбрать лучший результат
    template<typename ReplyType>
    SeparationResult<ReplyType> select_best_result(
        std::vector<SeparationResult<ReplyType>>& results);
};

// Вспомогательные функции для RBSReply
namespace utils {
    inline size_t bit_position(int bit_idx) {
        static constexpr uint8_t bit_to_ether[12] = {1,2,3,4,5,6,8,9,10,11,12,13};
        if (bit_idx >= 0 && bit_idx < 12) {
            return bit_to_ether[bit_idx];
        }
        return 0;
    }
    
    inline bool has_overlaps(const RBSReply& reply) {
        // Простая проверка: если много позиций с амплитудой выше средней
        uint16_t sum = 0;
        for (auto amp : reply.ether_amplitudes) {
            sum += amp;
        }
        uint8_t avg = sum / reply.ether_amplitudes.size();
        
        int high_amps = 0;
        for (auto amp : reply.ether_amplitudes) {
            if (amp > avg * 1.5) high_amps++;
        }
        
        return high_amps > reply.ether_amplitudes.size() / 3;
    }
}

} // namespace radar