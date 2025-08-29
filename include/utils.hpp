#pragma once 

#include <vector> 
#include <string>

namespace demo {
    constexpr int NUM_MONTE_CARLO_TRIALS_GLOBAL_PLANNING = 50;
    constexpr int NUM_MONTE_CARLO_TRIALS_LOCAL_PLANNING = 100;
    constexpr double MONTE_CARLO_PERCENT_DIFF_TOLERANCE = 1.0;
} // namespace demo

namespace planning_config{
    constexpr double GLOBAL_RESOLUTION = 25.0;
    constexpr double EDGE_VALIDATION_STEP_SIZE = 5.0;
    constexpr double FLOATING_POINT_COMPARISON_TOLERANCE = 1e-6;
    constexpr double RRT_STEP_SIZE = 1.0;
    constexpr double RRT_EDGE_VALIDATION_STEP_SIZE = 1.0;
    constexpr double RRT_ITERATIONS = 10000;
    constexpr double RRT_GOAL_BIAS =  0.5;
    constexpr double LOCAL_PATH_SPACING = 5.0;
} // namespace planning_config


// Forward declaration: lets this header know ObstacleProcessor class exists. 
//  A full definition is not needed because I am only using a reference. 
namespace obstacle_processor {
    class ObstacleProcessor;
} // namespace obstacle_processor

namespace helpers{

    [[nodiscard]] double distance_between_points(const std::vector<double>& point1, const std::vector<double>& point2) noexcept;
    [[nodiscard]] double ask_double(const std::string& message);

} // namespace helpers



