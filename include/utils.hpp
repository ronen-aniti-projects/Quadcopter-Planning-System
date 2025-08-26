#pragma once 

#include <vector> 

namespace planning_config{
    constexpr double GLOBAL_RESOLUTION = 25.0;
    constexpr double EDGE_VALIDATION_STEP_SIZE = 5.0;
    constexpr double FLOATING_POINT_COMPARISON_TOLERANCE = 1e-6;
} // namespace planning_config


namespace helpers{

    [[nodiscard]] double distance_between_points(const std::vector<double>& point1, const std::vector<double>& point2) noexcept;

} // namespace helpers



