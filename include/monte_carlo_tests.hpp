/**
 * @file monte_carlo_tests.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Header file where Monte Carlo performance assessment functions are declared.
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once 
#include "obstacle_processor.hpp"
#include "local_planning.hpp"
#include "global_planning.hpp"
#include <vector>

/**
 * @namespace monte_carlo_trials
 * @brief Implements tools for Monte Carlo performance assessment of path planners
 */
namespace monte_carlo_trials{

    /**
     * @brief Prints waypoint coordinates to the console
     * 
     * @param points Input points
     */
    void display_points(const std::vector<std::vector<double>>& points) noexcept;

    /**
     * @brief Implements Monte Carlo assessment of the global planner
     * 
     * @param obstacles The object representing the obstacles
     * @param free_space The object representing the free space
     */
    void evaluate_global_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space);
    
    /**
     * @brief Implements Monte Carlo assessment of the local planner
     * 
     * @param obstacles The object representing the obstacles
     * @param free_space The object representing the free space
     * @param goal_bias The probability of sampling the goal node
     */
    void evaluate_local_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space, const double goal_bias);
} // namespace monte_carlo_trials
