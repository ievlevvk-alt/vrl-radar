// file: src/cluster_processor.cpp
#include "radar/cluster_processor.h"
#include "radar/utils.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <iostream>  // Добавлено для cout

namespace radar {

ClusterProcessor::ClusterProcessor(const RadarConfig& config) 
    : config_(config), reply_processor_(config) {}


std::vector<ClusterProcessor::RangeGroup> ClusterProcessor::group_by_range(const TargetCluster& cluster) {
    std::vector<RangeGroup> groups;
    std::map<uint16_t, RangeGroup> range_map;
    
    // Собираем все RBS ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.rbs_replies) {
            bool added = false;
            // Ищем существующую группу с близкой дальностью
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_) {
                    group.add_rbs(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_rbs(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    // Собираем все УВД ответы
    for (const auto& scan : cluster.scans) {
        for (const auto& reply : scan.uvd_replies) {
            bool added = false;
            for (auto& [nominal, group] : range_map) {
                if (std::abs(static_cast<int16_t>(reply.range - nominal)) <= range_tolerance_) {
                    group.add_uvd(&reply);
                    added = true;
                    break;
                }
            }
            if (!added) {
                RangeGroup new_group;
                new_group.nominal_range = reply.range;
                new_group.add_uvd(&reply);
                range_map[reply.range] = new_group;
            }
        }
    }
    
    // Преобразуем map в вектор
    for (auto& [_, group] : range_map) {
        groups.push_back(std::move(group));
    }
    
    return groups;
}

double ClusterProcessor::average_azimuth(const std::vector<uint16_t>& azimuths) {
    if (azimuths.empty()) return 0.0;
    
    // Проверяем, есть ли переход через 0
    bool has_wraparound = false;
    for (size_t i = 1; i < azimuths.size(); ++i) {
        if (std::abs(static_cast<int16_t>(azimuths[i] - azimuths[i-1])) > 2048) {
            has_wraparound = true;
            break;
        }
    }
    
    if (!has_wraparound) {
        // Обычное среднее
        double sum = 0.0;
        for (auto az : azimuths) sum += az;
        return sum / azimuths.size();
    } else {
        // С учётом цикличности
        double sum_sin = 0.0, sum_cos = 0.0;
        for (auto az : azimuths) {
            double rad = az * config_.azimuth_per_bin * M_PI / 180.0;
            sum_sin += std::sin(rad);
            sum_cos += std::cos(rad);
        }
        double avg_rad = std::atan2(sum_sin, sum_cos);
        double avg_deg = avg_rad * 180.0 / M_PI;
        if (avg_deg < 0) avg_deg += 360.0;
        return avg_deg / config_.azimuth_per_bin;
    }
}

bool ClusterProcessor::check_sidelobe(const RBSReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

bool ClusterProcessor::check_sidelobe(const UVDReply& reply) const {
    return utils::is_sidelobe(reply, 3.0);
}

// Декодирование УВД информации из 20 бит согласно структуре из replies.h
void ClusterProcessor::decode_uvd_info(uint32_t data20, TargetReport& report) {
    report.uvd.raw_data20 = data20;
    
    // Декодирование пятизначного восьмеричного кода
    uint32_t octal_part = data20 & 0x7FFFF;  // 20 бит
    
    // Преобразуем в восьмеричное представление
    for (int i = 4; i >= 0; --i) {
        report.uvd.octal_id[i] = (octal_part >> (i * 3)) & 0x7;
    }
    
    // Высота в футах (примерная раскладка бит)
    report.uvd.altitude = (data20 >> 3) & 0x7FF;  // 11 бит высоты
    
    // Топливо (пример)
    report.uvd.fuel = (data20 >> 14) & 0x0F;
    
    // Признак барометрической высоты
    report.uvd.pressure_ref = (data20 >> 18) & 0x01;
}

std::optional<TargetReport> ClusterProcessor::process_rbs_group(const RangeGroup& group) {
    if (group.rbs_replies.empty()) return std::nullopt;
    
    TargetReport report;
    report.type = TargetReport::SourceType::RBS;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    // Собираем азимуты для усреднения
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.rbs_replies) {
        azimuths.push_back(reply->azimuth);
        report.sources.push_back(reply);
    }
    
    // Вычисляем средний азимут и дальность
    report.azimuth_deg = average_azimuth(azimuths) * config_.azimuth_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_rbs;
    
    // Преобразование в декартовы координаты
    radar::polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    // Анализ кодов
    std::map<uint16_t, int> code_counts;
    std::map<uint16_t, const RBSReply*> code_to_reply;
    std::map<uint16_t, bool> code_spi;
    std::map<uint16_t, double> code_confidence;  // <--- ДОБАВИТЬ
    
    for (const auto* reply : group.rbs_replies) {
        code_counts[reply->code12]++;
        code_to_reply[reply->code12] = reply;
        if (reply->spi) {
            code_spi[reply->code12] = true;
        }
        
        // Анализ качества ответа через ReplyProcessor
        ReplyFeatures features = reply_processor_.analyze_rbs(*reply);
        code_confidence[reply->code12] = std::max(code_confidence[reply->code12], features.confidence);
    }
    
    // Выбираем код с максимальным количеством появлений И высокой уверенностью
    uint16_t best_code = 0;
    int max_count = 0;
    double best_confidence = 0;
    
    for (const auto& [code, count] : code_counts) {
        double confidence = code_confidence[code];
        // Приоритет: сначала уверенность, потом количество
        if (confidence > best_confidence || 
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_code = code;
        }
    }
    
    if (max_count > 0 && best_confidence > 0.3) {  // Порог уверенности 0.3
        const auto* best_reply = code_to_reply[best_code];
        report.rbs.mode3a_code = best_code;
        report.rbs.spi = code_spi[best_code];
        
        // Используем уверенность из ReplyProcessor
        report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
        
        // Проверяем SLS
        report.is_sls_blanked = check_sidelobe(*best_reply);
        
        // Вместо utils::validate_rbs используем confidence
        report.is_garbled = (best_confidence < 0.5);  // Если уверенность < 0.5 - считаем искаженным
    } else {
        // Недостаточная уверенность - не формируем отчет
        return std::nullopt;
    }
    
    return report;
}

std::optional<TargetReport> ClusterProcessor::process_uvd_group(const RangeGroup& group) {
    if (group.uvd_replies.empty()) return std::nullopt;
    
    TargetReport report;
    report.type = TargetReport::SourceType::UVD;
    report.signal_strength = 0;
    report.is_reflection = false;
    report.is_sls_blanked = false;
    report.is_garbled = false;
    
    std::vector<uint16_t> azimuths;
    for (const auto* reply : group.uvd_replies) {
        azimuths.push_back(reply->azimuth);
        report.sources.push_back(reply);
    }
    
    report.azimuth_deg = average_azimuth(azimuths) * config_.azimuth_per_bin;
    report.range_m = group.nominal_range * config_.range_bin_uvd;
    radar::polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
    
    std::map<uint32_t, int> data_counts;
    std::map<uint32_t, const UVDReply*> data_to_reply;
    std::map<uint32_t, double> data_confidence;
    
    for (const auto* reply : group.uvd_replies) {
        data_counts[reply->data20]++;
        data_to_reply[reply->data20] = reply;
        
        ReplyFeatures features = reply_processor_.analyze_uvd(*reply);
        data_confidence[reply->data20] = std::max(data_confidence[reply->data20], features.confidence);
    }
    
    uint32_t best_data = 0;
    int max_count = 0;
    double best_confidence = 0;
    
    for (const auto& [data, count] : data_counts) {
        double confidence = data_confidence[data];
        if (confidence > best_confidence ||
            (confidence == best_confidence && count > max_count)) {
            best_confidence = confidence;
            max_count = count;
            best_data = data;
        }
    }
    
    if (max_count > 0 && best_confidence > 0.3) {
        const auto* best_reply = data_to_reply[best_data];
        decode_uvd_info(best_data, report);
        report.signal_strength = static_cast<uint8_t>(best_confidence * 255);
        report.is_sls_blanked = check_sidelobe(*best_reply);
        report.is_garbled = (best_confidence < 0.5) || (best_reply->error_mask != 0);
    } else {
        return std::nullopt;
    }
    
    return report;
}


std::vector<TargetReport> ClusterProcessor::process_garbled_group(const RangeGroup& group) {
    std::vector<TargetReport> reports;
    
    if (!garbling_solver_ || group.rbs_replies.empty()) {
        return reports;
    }
    
    // Собираем все RBS ответы из группы
    std::vector<RBSReply> all_rbs;
    for (const auto* ptr : group.rbs_replies) {
        all_rbs.push_back(*ptr);
    }
    
    // Пытаемся разделить перекрытие
    auto result = garbling_solver_->separate_rbs(all_rbs);
    
    if (result.confidence > 0.5 && !result.separated_replies.empty()) {
        // Для каждого разделенного ответа создаем отчет
        for (const auto& separated : result.separated_replies) {
            TargetReport report;
            report.type = TargetReport::SourceType::RBS;
            report.azimuth_deg = separated.azimuth * config_.azimuth_per_bin;
            report.range_m = separated.range * config_.range_bin_rbs;
            report.rbs.mode3a_code = separated.code12;
            report.rbs.spi = separated.spi;
            report.is_garbled = false;  // Успешно разделили
            report.is_sls_blanked = check_sidelobe(separated);
            
            radar::polar_to_xy(report.range_m, report.azimuth_deg, report.x, report.y);
            reports.push_back(report);
        }
        
        // Логируем успешное разделение
        if (debug_) {
            std::cout << "Split " << all_rbs.size() << " replies into " 
                      << reports.size() << " targets (confidence=" 
                      << result.confidence << ")\n";
        }
    }
    
    return reports;
}

std::vector<TargetReport> ClusterProcessor::process_cluster(const TargetCluster& cluster) {
    std::vector<TargetReport> reports;
    
    // Группируем ответы по дальности
    auto range_groups = group_by_range(cluster);
    
    for (const auto& group : range_groups) {
        // Проверяем минимальное количество попаданий
        if (group.total_replies() < static_cast<size_t>(min_hits_)) {
            continue;
        }
        
        // Проверяем, есть ли перекрытия в группе
        bool has_overlaps = false;
        if (!group.rbs_replies.empty() && group.rbs_replies.size() > 1) {
            // Простая проверка: если много ответов на близкой дальности
            has_overlaps = true;
        }
        
        if (has_overlaps && split_garbled_ && garbling_solver_) {
            // Пытаемся разделить перекрытие
            auto split_reports = process_garbled_group(group);
            reports.insert(reports.end(), split_reports.begin(), split_reports.end());
        } else {
            // Обычная обработка
            if (!group.rbs_replies.empty()) {
                auto report = process_rbs_group(group);
                if (report) reports.push_back(*report);
            }
            
            if (!group.uvd_replies.empty()) {
                auto report = process_uvd_group(group);
                if (report) reports.push_back(*report);
            }
        }
    }
    
    return reports;
}

} // namespace radar