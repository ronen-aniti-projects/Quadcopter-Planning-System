/**
 * @file utils.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides constants and helper functions used project wide
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once 

#include <vector> 
#include <string>

/**
 * @namespace demo
 * @brief Constants tied to Monte Carlo performance assessments
 * 
 */
namespace demo {
    constexpr int NUM_MONTE_CARLO_TRIALS_GLOBAL_PLANNING = 50;
    constexpr int NUM_MONTE_CARLO_TRIALS_LOCAL_PLANNING = 100;
    constexpr double MONTE_CARLO_PERCENT_DIFF_TOLERANCE = 1.0;
} // namespace demo


/**
 * @namespace planning_config
 * @brief Provides default settings applied in local, global, and trajectory planning modules
 */
namespace planning_config{
    constexpr double GLOBAL_RESOLUTION = 25.0;
    constexpr double EDGE_VALIDATION_STEP_SIZE = 5.0;
    constexpr double FLOATING_POINT_COMPARISON_TOLERANCE = 1e-6;
    constexpr double RRT_STEP_SIZE = 1.0;
    constexpr double RRT_EDGE_VALIDATION_STEP_SIZE = 1.0;
    constexpr double RRT_ITERATIONS = 10000;
    constexpr double RRT_GOAL_BIAS =  0.5;
    constexpr double LOCAL_PATH_SPACING = 5.0;
    constexpr double HEURISTIC_SPEED_CONSTANT = 5.0;
    constexpr int COEFFS_PER_SEGMENT = 8;
    constexpr int NUM_BOUNDARY_CONSTRAINTS = 6;
    constexpr double CHAIN_RULE_COMPONENT = 2.0;
    constexpr int TRAJ_POLYNOMIAL_ORDER = 7;
    constexpr int NUM_AXES = 3;
    constexpr double TRAJECTORY_TIME_POINT_SPACING = 0.01;
} // namespace planning_config


// Forward declaration
namespace obstacle_processor {
    class ObstacleProcessor;
} // namespace obstacle_processor

/**
 * @namespace helpers
 * @brief Provides helper functions used across the project
 * 
 */
namespace helpers{

    [[nodiscard]] double distance_between_points(const std::vector<double>& point1, const std::vector<double>& point2) noexcept;
    [[nodiscard]] double ask_double(const std::string& message);

} // namespace helpers



