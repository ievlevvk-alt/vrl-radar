// include/vrl/radar/core/source_visitor.hpp
#pragma once

#include "replies.h"
#include <variant>

namespace vrl {
namespace radar {

/**
 * @brief Вспомогательный класс для обхода источников ответов
 * 
 * Использование:
 * SourceVisitor visitor;
 * for (const auto& source : report.sources) {
 *     visitor.visit(source);
 * }
 * 
 * // Или с лямбдой:
 * report.visit_sources([](auto* source) {
 *     // source может быть RBSReply* или UVDReply*
 * });
 */
class SourceVisitor {
public:
    virtual ~SourceVisitor() = default;
    
    virtual void visit(const RBSReply* reply) = 0;
    virtual void visit(const UVDReply* reply) = 0;
    
    void visit(const ReplySource& source) {
        std::visit([this](const auto* ptr) {
            if constexpr (std::is_same_v<decltype(ptr), const RBSReply*>) {
                visit(ptr);
            } else if constexpr (std::is_same_v<decltype(ptr), const UVDReply*>) {
                visit(ptr);
            }
        }, source);
    }
    
    void visit_all(const std::vector<ReplySource>& sources) {
        for (const auto& source : sources) {
            visit(source);
        }
    }
};

// Шаблонный вариант для лямбд
template<typename Visitor>
void visit_sources(const std::vector<ReplySource>& sources, Visitor&& visitor) {
    for (const auto& source : sources) {
        std::visit([&visitor](const auto* ptr) {
            if constexpr (std::is_same_v<decltype(ptr), const RBSReply*>) {
                visitor(ptr);
            } else if constexpr (std::is_same_v<decltype(ptr), const UVDReply*>) {
                visitor(ptr);
            }
        }, source);
    }
}

} // namespace radar
} // namespace vrl
