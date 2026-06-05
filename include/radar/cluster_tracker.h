// file: include/radar/cluster_tracker.h
#pragma once

#include "replies.h"
#include <vector>
#include <map>
#include <cstdint>
#include <algorithm>

namespace radar {

// Ответы на одном зондировании (все с одинаковым азимутом)
struct ScanReplies {
    uint16_t azimuth{0};                    // текущий азимут
    std::vector<RBSReply> rbs_replies;      // все RBS ответы на этом зондировании
    std::vector<UVDReply> uvd_replies;      // все УВД ответы на этом зондировании
    uint32_t timestamp_ms{0};                // время зондирования
    
    ScanReplies() = default;
    ScanReplies(uint16_t az, uint32_t ts) : azimuth(az), timestamp_ms(ts) {}
    
    // Проверка на наличие ответов
    bool has_replies() const {
        return !rbs_replies.empty() || !uvd_replies.empty();
    }
    
    // Количество ответов
    size_t reply_count() const {
        return rbs_replies.size() + uvd_replies.size();
    }
};

// Кластер целей (может содержать несколько целей, идущих рядом)
struct TargetCluster {
    std::vector<ScanReplies> scans;          // все зондирования в кластере
    uint16_t start_azimuth{0};                // первый азимут с ответами
    uint16_t last_reply_azimuth{0};           // последний азимут, где были ответы
    uint16_t last_processed_azimuth{0};       // последний обработанный азимут (может быть пустым)
    uint16_t min_range{65535};                // минимальная дальность в кластере
    uint16_t max_range{0};                    // максимальная дальность в кластере
    uint32_t first_timestamp{0};              // время первого ответа
    uint32_t last_timestamp{0};               // время последнего ответа
    
    // Статистика по азимутам для быстрого доступа
    std::map<uint16_t, std::vector<RBSReply>> rbs_by_azimuth;
    std::map<uint16_t, std::vector<UVDReply>> uvd_by_azimuth;
    
    TargetCluster() = default;
    
    // Добавить результаты зондирования в кластер
    void add_scan(const ScanReplies& scan) {
        if (scans.empty()) {
            start_azimuth = scan.azimuth;
            first_timestamp = scan.timestamp_ms;
            min_range = 65535;
            max_range = 0;
        }
        
        scans.push_back(scan);
        last_processed_azimuth = scan.azimuth;
        
        if (scan.has_replies()) {
            last_reply_azimuth = scan.azimuth;
            last_timestamp = scan.timestamp_ms;
        }
        
        // Обновляем диапазон дальности и индексы
        for (const auto& reply : scan.rbs_replies) {
            rbs_by_azimuth[scan.azimuth].push_back(reply);
            min_range = std::min(min_range, reply.range);
            max_range = std::max(max_range, reply.range);
        }
        
        for (const auto& reply : scan.uvd_replies) {
            uvd_by_azimuth[scan.azimuth].push_back(reply);
            min_range = std::min(min_range, reply.range);
            max_range = std::max(max_range, reply.range);
        }
    }
    
    // Проверка, активен ли ещё кластер (не было ли слишком большого пропуска)
    bool is_active(uint16_t current_azimuth, int max_gap_azimuth) const {
        if (scans.empty()) return false;
        
        // Используем last_reply_azimuth, а не last_processed_azimuth
        // чтобы пустые зондирования не продлевали жизнь кластера бесконечно
        int16_t gap = current_azimuth - last_reply_azimuth;
        if (gap < 0) gap += 4096;
        
        return gap <= max_gap_azimuth;
    }
    
    // Получить все RBS ответы в кластере
    std::vector<RBSReply> get_all_rbs() const {
        std::vector<RBSReply> result;
        for (const auto& scan : scans) {
            result.insert(result.end(), scan.rbs_replies.begin(), scan.rbs_replies.end());
        }
        return result;
    }
    
    // Получить все UVD ответы в кластере
    std::vector<UVDReply> get_all_uvd() const {
        std::vector<UVDReply> result;
        for (const auto& scan : scans) {
            result.insert(result.end(), scan.uvd_replies.begin(), scan.uvd_replies.end());
        }
        return result;
    }
    
    // Проверка перекрытий внутри каждого зондирования
    std::vector<std::pair<size_t, size_t>> find_garbled_pairs_in_scan(size_t scan_index) const {
        std::vector<std::pair<size_t, size_t>> pairs;
        if (scan_index >= scans.size()) return pairs;
        
        const auto& scan = scans[scan_index];
        
        // Проверяем перекрытия между RBS ответами в этом зондировании
        for (size_t i = 0; i < scan.rbs_replies.size(); ++i) {
            for (size_t j = i + 1; j < scan.rbs_replies.size(); ++j) {
                const auto& r1 = scan.rbs_replies[i];
                const auto& r2 = scan.rbs_replies[j];
                
                // Перекрытие возможно только при одинаковом азимуте
                if (r1.azimuth != r2.azimuth) continue;
                
                // Проверяем по дальности (в зависимости от размера дискрета)
                int range_diff = std::abs(static_cast<int>(r1.range) - static_cast<int>(r2.range));
                
                // Если разница меньше длины ответа в дискретах
                // Длина RBS ответа примерно 8 дискретов (18 позиций по ~0.45 дискрета)
                if (range_diff < 10) {
                    pairs.emplace_back(i, j);
                }
            }
        }
        
        return pairs;
    }
    
    // Ширина кластера в дискретах азимута (только там, где были ответы)
    uint16_t azimuth_span() const {
        if (scans.empty()) return 0;
        
        int16_t span = last_reply_azimuth - start_azimuth;
        if (span < 0) span += 4096;
        return static_cast<uint16_t>(span);
    }
    
    // Полная ширина кластера (включая пустые промежутки)
    uint16_t total_azimuth_span() const {
        if (scans.empty()) return 0;
        
        int16_t span = last_processed_azimuth - start_azimuth;
        if (span < 0) span += 4096;
        return static_cast<uint16_t>(span);
    }
    
    // Ширина кластера по дальности
    uint16_t range_span() const {
        return max_range - min_range;
    }
    
    // Количество зондирований с ответами
    size_t reply_scans_count() const {
        size_t count = 0;
        for (const auto& scan : scans) {
            if (scan.has_replies()) count++;
        }
        return count;
    }
};

// Основной класс для отслеживания кластеров
class ClusterTracker {
public:
    ClusterTracker(int max_gap_azimuth = 8, int range_window = 30)
        : max_gap_azimuth_(max_gap_azimuth)
        , range_window_(range_window) {}
    
    // Обработать новое зондирование
    void process_scan(const ScanReplies& scan) {
        // 1. Обновляем существующие кластеры
        update_existing_clusters(scan);
        
        // 2. Если есть новые ответы, не попавшие в кластеры - создаём новые
        if (scan.has_replies()) {
            try_create_new_clusters(scan);
        }
        
        // 3. Проверяем, какие кластеры завершились
        complete_expired_clusters(scan.azimuth);
    }
    
    // Получить завершённые кластеры
    std::vector<TargetCluster> get_completed_clusters() {
        auto result = std::move(completed_clusters_);
        completed_clusters_.clear();
        return result;
    }
    
    // Получить активные кластеры (для отладки)
    const std::vector<TargetCluster>& get_active_clusters() const {
        return active_clusters_;
    }
    
    // Сброс состояния
    void reset() {
        active_clusters_.clear();
        completed_clusters_.clear();
    }
    
    // Настройка параметров
    void set_max_gap_azimuth(int gap) { max_gap_azimuth_ = gap; }
    void set_range_window(int window) { range_window_ = window; }
    
private:
    void update_existing_clusters(const ScanReplies& scan) {
        for (auto& cluster : active_clusters_) {
            // Проверяем, относится ли этот скан к кластеру по дальности
            bool range_match = false;
            
            if (scan.has_replies()) {
                // Если есть ответы, проверяем их дальность
                for (const auto& reply : scan.rbs_replies) {
                    if (reply.range >= cluster.min_range - range_window_ &&
                        reply.range <= cluster.max_range + range_window_) {
                        range_match = true;
                        break;
                    }
                }
                
                if (!range_match) {
                    for (const auto& reply : scan.uvd_replies) {
                        if (reply.range >= cluster.min_range - range_window_ &&
                            reply.range <= cluster.max_range + range_window_) {
                            range_match = true;
                            break;
                        }
                    }
                }
            } else {
                // Для пустых сканов считаем, что они могут относиться к любому кластеру
                // если азимут ещё в пределах окна
                if (cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
                    range_match = true;
                }
            }
            
            if (range_match && cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
                cluster.add_scan(scan);
            }
        }
    }
    
    void try_create_new_clusters(const ScanReplies& scan) {
        // Проверяем, можно ли отнести ответы к существующему кластеру
        for (auto& cluster : active_clusters_) {
            if (!cluster.is_active(scan.azimuth, max_gap_azimuth_)) {
                continue;
            }
            
            for (const auto& reply : scan.rbs_replies) {
                if (reply.range >= cluster.min_range - range_window_ &&
                    reply.range <= cluster.max_range + range_window_) {
                    // Уже добавлено в update_existing_clusters
                    return;
                }
            }
            
            for (const auto& reply : scan.uvd_replies) {
                if (reply.range >= cluster.min_range - range_window_ &&
                    reply.range <= cluster.max_range + range_window_) {
                    return;
                }
            }
        }
        
        // Создаём новый кластер
        TargetCluster new_cluster;
        new_cluster.add_scan(scan);
        active_clusters_.push_back(std::move(new_cluster));
    }
    
    void complete_expired_clusters(uint16_t current_azimuth) {
        auto it = active_clusters_.begin();
        while (it != active_clusters_.end()) {
            if (!it->is_active(current_azimuth, max_gap_azimuth_)) {
                completed_clusters_.push_back(std::move(*it));
                it = active_clusters_.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    std::vector<TargetCluster> active_clusters_;
    std::vector<TargetCluster> completed_clusters_;
    
    int max_gap_azimuth_;   // максимальный пропуск по азимуту (в дискретах)
    int range_window_;      // окно по дальности для объединения (в дискретах)
};

} // namespace radar