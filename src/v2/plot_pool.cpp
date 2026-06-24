// src/v2/plot_pool.cpp
#include "vrl/radar/v2/plot_pool.hpp"
#include "vrl/radar/utils/logger.h"
#include <iostream>

using namespace vrl::radar::utils;

namespace vrl {
namespace radar {
namespace v2 {

PlotPool& PlotPool::instance() {
    static PlotPool instance;
    return instance;
}

void PlotPool::init(size_t max_plots) {
    if (initialized_) {
        VRL_LOG_WARN(modules::CORE, "PlotPool already initialized");
        return;
    }
    
    if (max_plots == 0) {
        VRL_LOG_ERROR(modules::CORE, "max_plots must be > 0");
        return;
    }
    
    max_plots_ = max_plots;
    plots_.resize(max_plots_);
    head_ = 0;
    initialized_ = true;
    
    VRL_LOG_INFO(modules::CORE, "PlotPool initialized with " + 
                 std::to_string(max_plots_) + " slots");
}

bool PlotPool::is_valid_index(uint64_t index) const {
    if (index == 0) return false;
    size_t slot = static_cast<size_t>(index - 1);
    return slot < max_plots_;
}

uint64_t PlotPool::add_plot(const Plot& plot) {
    if (!initialized_) {
        VRL_LOG_ERROR(modules::CORE, "PlotPool not initialized");
        return 0;
    }
    
    size_t slot = head_ % max_plots_;
    head_ = (head_ + 1) % max_plots_;
    
    plots_[slot] = plot;
    uint64_t index = static_cast<uint64_t>(slot + 1);
    
#ifdef CMAKE_BUILD_TYPE_DEBUG
    std::cout << "[PlotPool] add_plot: index=" << index 
              << ", slot=" << slot << std::endl;
#endif
    
    VRL_LOG_DEBUG(modules::CORE, "Plot added, index=" + std::to_string(index));
    
    return index;
}

const Plot* PlotPool::get_plot(uint64_t index) const {
    if (!is_valid_index(index)) {
        return nullptr;
    }
    
    size_t slot = static_cast<size_t>(index - 1);
    return &plots_[slot];
}

void PlotPool::clear() {
#ifdef CMAKE_BUILD_TYPE_DEBUG
    std::cout << "[PlotPool] clear: called" << std::endl;
#endif
    
    for (size_t i = 0; i < max_plots_; ++i) {
        plots_[i] = Plot{};
    }
    head_ = 0;
    
    VRL_LOG_INFO(modules::CORE, "PlotPool cleared");
}

} // namespace v2
} // namespace radar
} // namespace vrl
