// file: include/radar/stream_grouper.h
#pragma once

#include "replies.h"
#include "utils.h"
#include <vector>
#include <deque>
#include <functional>
#include <memory>
#include <algorithm>
#include <cmath>

namespace radar {

// Контекст сканирования
struct ScanInfo {
    uint32_t scan_number{0};     // номер оборота (сбрасывается при инициализации)
    uint16_t azimuth{0};          // текущий азимут (0-4095)
    bool scan_start{false};       // признак начала нового оборота
    bool scan_end{false};         // признак завершения оборота
    uint32_t timestamp_ms{0};     // опционально: время от начала оборота в мс
};

// Обработанный кластер для выдачи
template<typename ReplyType>
struct ProcessedCluster {
    std::vector<ReplyType> replies;     // все ответы в кластере
    double avg_azimuth{0.0};             // средний азимут
    double avg_range{0.0};               // средняя дальность
    double azimuth_span{0.0};             // размах по азимуту
    double range_span{0.0};               // размах по дальности
    bool has_overlaps{false};             // есть перекрытия
    
    // Для RBS добавляем информацию о кодах
    std::vector<uint16_t> detected_codes; // обнаруженные коды после разделения
};

// Интерфейс обработчика кластеров
template<typename ReplyType>
using ClusterHandler = std::function<void(const ProcessedCluster<ReplyType>&)>;

// Потоковый группировщик
template<typename ReplyType>
class StreamGrouper {
public:
    StreamGrouper(const RadarConfig& cfg, 
                  double beamwidth_az_bins,  // ширина ДН в дискретах азимута
                  ClusterHandler<ReplyType> handler)
        : config_(cfg)
        , beamwidth_(beamwidth_az_bins)
        , cluster_handler_(handler)
        , current_scan_(0)
        , last_azimuth_(0)
        , wrap_around_detected_(false) {}
    
    // Обработка нового ответа
    void process_reply(const ReplyType& reply, const ScanInfo& scan) {
        // Проверяем смену оборота
        if (scan.scan_start) {
            start_new_scan(scan);
        }
        
        // Проверяем, не пора ли закрыть активные кластеры
        check_and_close_clusters(scan.azimuth);
        
        // Добавляем ответ в подходящий кластер или создаём новый
        add_to_clusters(reply, scan);
        
        last_azimuth_ = scan.azimuth;
        
        // Если это конец оборота - принудительно закрываем всё
        if (scan.scan_end) {
            flush();
        }
    }
    
    // Принудительно завершить все кластеры
    void flush() {
        for (auto& cluster : active_clusters_) {
            if (!cluster.replies.empty() && !cluster.closed) {
                finish_cluster(cluster);
            }
        }
        active_clusters_.clear();
    }
    
    // Сброс состояния
    void reset() {
        flush();
        current_scan_ = 0;
        last_azimuth_ = 0;
        wrap_around_detected_ = false;
    }
    
private:
    struct ActiveCluster {
        std::vector<ReplyType> replies;
        uint16_t min_azimuth{4096};
        uint16_t max_azimuth{0};
        uint16_t min_range{65535};
        uint16_t max_range{0};
        uint16_t start_azimuth{0};    // азимут первого ответа в кластере
        bool closed{false};
        bool crossed_zero{false};      // был ли переход через 0
        
        void add_reply(const ReplyType& reply) {
            if (replies.empty()) {
                start_azimuth = reply.azimuth;
                min_azimuth = max_azimuth = reply.azimuth;
                min_range = max_range = reply.range;
            } else {
                // Обновляем границы с учётом цикличности азимута
                update_bounds(reply);
            }
            replies.push_back(reply);
        }
        
        bool is_within_beam(uint16_t azimuth, double beamwidth) const {
            if (replies.empty()) return true;
            
            // Рассчитываем расстояние от начала кластера до текущего азимута
            int16_t distance_from_start = azimuth - start_azimuth;
            if (distance_from_start < 0) distance_from_start += 4096;
            
            // Кластер активен, пока мы не ушли дальше beamwidth от его начала
            return distance_from_start <= beamwidth * 1.5;  // с запасом 50%
        }
        
        bool is_nearby(const ReplyType& reply, const RadarConfig& cfg) const {
            if (replies.empty()) return true;
            
            // Проверяем близость к любому ответу в кластере
            for (const auto& existing : replies) {
                if (utils::is_potential_overlap(
                        existing.azimuth, existing.range,
                        reply.azimuth, reply.range,
                        cfg)) {
                    return true;
                }
            }
            return false;
        }
        
    private:
        void update_bounds(const ReplyType& reply) {
            uint16_t az = reply.azimuth;
            
            // Проверяем переход через 0
            if (!crossed_zero) {
                // Если новый азимут сильно меньше предыдущего минимума
                // и при этом разница большая - значит перешли через 0
                if (min_azimuth > 2048 && az < 2048 && 
                    static_cast<int>(min_azimuth) - static_cast<int>(az) > 2048) {
                    crossed_zero = true;
                }
            }
            
            if (crossed_zero) {
                // После перехода через 0 логика меняется
                if (az < 2048) {
                    // Это уже новый сектор
                    max_azimuth = std::max(max_azimuth, az);
                }
                // Азимуты больше 2048 игнорируем в max, они были "до перехода"
            } else {
                // Обычный режим
                min_azimuth = std::min(min_azimuth, az);
                max_azimuth = std::max(max_azimuth, az);
            }
            
            min_range = std::min(min_range, reply.range);
            max_range = std::max(max_range, reply.range);
        }
    };
    
    void start_new_scan(const ScanInfo& scan) {
        // Завершаем все кластеры предыдущего оборота
        flush();
        
        current_scan_ = scan.scan_number;
        wrap_around_detected_ = false;
    }
    
    void check_and_close_clusters(uint16_t current_az) {
        auto it = active_clusters_.begin();
        while (it != active_clusters_.end()) {
            if (!it->is_within_beam(current_az, beamwidth_)) {
                // Кластер больше не попадает в луч - завершаем его
                finish_cluster(*it);
                it = active_clusters_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void add_to_clusters(const ReplyType& reply, const ScanInfo& scan) {
        // Ищем подходящий активный кластер
        for (auto& cluster : active_clusters_) {
            if (cluster.is_nearby(reply, config_)) {
                cluster.add_reply(reply);
                return;
            }
        }
        
        // Создаём новый кластер
        ActiveCluster new_cluster;
        new_cluster.add_reply(reply);
        active_clusters_.push_back(std::move(new_cluster));
    }
    
    void finish_cluster(ActiveCluster& cluster) {
        if (cluster.replies.empty() || cluster.closed) return;
        
        ProcessedCluster<ReplyType> result;
        result.replies = cluster.replies;
        
        // Вычисляем средние координаты
        double sum_az = 0, sum_range = 0;
        double min_az_val = 4096, max_az_val = 0;
        double min_range_val = 1e9, max_range_val = 0;
        
        for (const auto& r : cluster.replies) {
            sum_az += r.azimuth;
            sum_range += r.range;
            
            min_az_val = std::min(min_az_val, static_cast<double>(r.azimuth));
            max_az_val = std::max(max_az_val, static_cast<double>(r.azimuth));
            min_range_val = std::min(min_range_val, static_cast<double>(r.range));
            max_range_val = std::max(max_range_val, static_cast<double>(r.range));
        }
        
        result.avg_azimuth = sum_az / cluster.replies.size();
        result.avg_range = sum_range / cluster.replies.size();
        
        // Учитываем цикличность азимута при вычислении размаха
        if (max_az_val - min_az_val > 2048) {
            result.azimuth_span = (4096 - max_az_val) + min_az_val;
        } else {
            result.azimuth_span = max_az_val - min_az_val;
        }
        result.range_span = max_range_val - min_range_val;
        
        // Проверяем перекрытия
        for (size_t i = 0; i < cluster.replies.size() - 1; ++i) {
            for (size_t j = i + 1; j < cluster.replies.size(); ++j) {
                if (utils::is_potential_overlap(
                        cluster.replies[i].azimuth, cluster.replies[i].range,
                        cluster.replies[j].azimuth, cluster.replies[j].range,
                        config_)) {
                    result.has_overlaps = true;
                    break;
                }
            }
        }
        
        // Вызываем обработчик
        if (cluster_handler_) {
            cluster_handler_(result);
        }
        
        cluster.closed = true;
    }
    
    RadarConfig config_;
    double beamwidth_;  // ширина ДН в дискретах азимута
    ClusterHandler<ReplyType> cluster_handler_;
    
    uint32_t current_scan_;
    uint16_t last_azimuth_;
    bool wrap_around_detected_;
    
    std::vector<ActiveCluster> active_clusters_;
};

// Удобные алиасы для конкретных типов
using RBSStreamGrouper = StreamGrouper<RBSReply>;
using UVDStreamGrouper = StreamGrouper<UVDReply>;

} // namespace radar