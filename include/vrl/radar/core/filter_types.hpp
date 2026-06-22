// include/vrl/radar/core/filter_types.hpp
#pragma once

namespace vrl {
namespace radar {

enum class FilterType {
    KALMAN,
    EXTENDED_KALMAN,
    UNSCENTED_KALMAN
};

} // namespace radar
} // namespace vrl
