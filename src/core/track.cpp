// src/core/track.cpp
#include "vrl/radar/core/track.hpp"
#include <algorithm>

namespace vrl {
namespace radar {

void Track::add_history(const TargetReport& report) {
    history_.push_back(report);
    if (history_.size() > MAX_HISTORY) {
        history_.erase(history_.begin());
    }
}

std::vector<TargetReport> Track::get_history() const {
    return history_;
}

const TargetReport* Track::get_last_report() const {
    if (history_.empty()) return nullptr;
    return &history_.back();
}

void Track::clear_history() {
    history_.clear();
}

} // namespace radar
} // namespace vrl
