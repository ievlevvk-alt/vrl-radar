// file: include/radar/grouping.h
#pragma once

#include "replies.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>

namespace radar {

// Структура для хранения сырых ответов с контекстом сканирования
struct RawReply {
    enum class Type { RBS, UVD } type;
    uint16_t azimuth;
    uint16_t range;
    uint32_t scan_number;      // номер оборота
    uint32_t timestamp_ms;     // время от начала оборота
    
    union {
        RBSReply rbs;
        UVDReply uvd;
    };
    
    // Конструкторы
    RawReply(const RBSReply& r, uint32_t scan, uint32_t ts) 
        : type(Type::RBS), azimuth(r.azimuth), range(r.range), 
          scan_number(scan), timestamp_ms(ts), rbs(r) {}
    
    RawReply(const UVDReply& u, uint32_t scan, uint32_t ts) 
        : type(Type::UVD), azimuth(u.azimuth), range(u.range), 
          scan_number(scan), timestamp_ms(ts), uvd(u) {}
};

// Кластер потенциально связанных ответов (для пост-обработки)
struct ReplyCluster {
    uint32_t scan_number{0};           // номер оборота
    uint16_t start_azimuth{0};         // начальный азимут кластера
    uint16_t end_azimuth{0};           // конечный азимут кластера
    
    // Центр кластера (средние координаты)
    double avg_azimuth{0.0};
    double avg_range{0.0};
    
    // Разброс координат
    double azimuth_span{0.0};
    double range_span{0.0};
    
    // Собранные ответы
    std::vector<RawReply> replies;
    
    // Признаки перекрытия
    bool has_overlap{false};
    
    // Индексы перекрывающихся ответов внутри кластера
    std::vector<std::pair<size_t, size_t>> overlapping_pairs;
    
    // Добавить ответ в кластер
    void add_reply(const RawReply& reply);
    
    // Проверить перекрытия внутри кластера
    void detect_overlaps(const RadarConfig& cfg);
};

// Класс для пост-обработки завершённого оборота
class ScanPostProcessor {
public:
    explicit ScanPostProcessor(const RadarConfig& cfg);
    
    // Добавить ответ в текущий собираемый оборот
    void add_reply(const RBSReply& reply, uint32_t scan_number, uint32_t timestamp_ms);
    void add_reply(const UVDReply& reply, uint32_t scan_number, uint32_t timestamp_ms);
    
    // Завершить оборот и получить кластеры
    std::vector<ReplyCluster> finish_scan(uint32_t scan_number);
    
    // Очистить данные оборота
    void reset();
    
private:
    RadarConfig config_;
    std::vector<RawReply> current_scan_;
    uint32_t current_scan_number_{0};
    
    // Кластеризация методом связных компонент
    std::vector<std::vector<size_t>> cluster_replies(const std::vector<RawReply>& replies) const;
    
    // Поиск соседей для заданного ответа
    std::vector<size_t> find_neighbors(size_t idx, const std::vector<RawReply>& replies) const;
};

} // namespace radar