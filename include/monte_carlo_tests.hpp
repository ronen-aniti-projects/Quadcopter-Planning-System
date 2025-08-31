/**
 * @file monte_carlo_tests.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides Monte Carlo-based performance assessment tools for planner benchmarking.
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
 * @brief Provides tools to assess performance of global and local planner modules.
 */
namespace monte_carlo_trials{

    /**
     * @brief Prints elements of a vector container to the console using a consistent format
     * 
     * @param points (std::vector<std::vector<double>>): Waypoint plan
     */
    void display_points(const std::vector<std::vector<double>>& points) noexcept;

    /**
     * @brief Assesses the performance of the global planning module. Examines the time it takes to complete A* between valid waypoint plans spaced several hundered meters apart.
     * 
     * @param obstacles (ObstacleProcessor): Obstaclestructure
     * @param free_space (FreeSpaceGraph): Free space graph object
     */
    void evaluate_global_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space);
    
    /**
     * @brief Assesses the performance of the local planning module. Analyzes the runtime for RRT to complete searches between valid start/goal configrations. 
     * 
     * @param obstacles (ObstacleProcessor): Obstacle structure
     * @param free_space (FreeSpaceGraph): Free space graph object
     * @param goal_bias RRT goal bias setting
     */
    void evaluate_local_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space, const double goal_bias);

} // namespace monte_carlo_trials
