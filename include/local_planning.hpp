/**
 * @file local_planning.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides tools for local path planning, including an RRT implementation. 
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once 

#include "utils.hpp"
#include "obstacle_processor.hpp"
#include "KDTree.hpp"
#include <optional>


/**
 * @namespace local_planning
 * @brief Provides tools for local path planning.
 * 
 */
namespace local_planning{ 

    /**
     * @brief Searches for a path between start and goal using RRT algorithm with goal bias.
     * 
     * @param obstacles (ObstacleProcessor): Obstacle data structure
     * @param start_point (std::vector<double>): Start point
     * @param goal_point (std::vector<double>): Goal point
     * @param step_size (double): RRT branch expansion step size
     * @param edge_validation_step_size (double): Collision check discretization step size
     * @param max_iterations (size_t): Limit to RRT iterations
     * @param goal_bias (double): Decimal value from [0, 1] for how often RRT samples the goal relative to taking a random sample
     * @return (std::optional<std::vector<std::vector<double>>>): Contains a waypoint plan when it's able to find one, otherwise empty.
     */
    [[nodiscard]] std::optional<std::vector<std::vector<double>>> rrt(const obstacle_processor::ObstacleProcessor& obstacles, 
            const std::vector<double>& start_point, 
            const std::vector<double>& goal_point,
            const double step_size = planning_config::RRT_STEP_SIZE,
            const double edge_validation_step_size = planning_config::RRT_EDGE_VALIDATION_STEP_SIZE,
            const size_t max_iterations = planning_config::RRT_ITERATIONS,
            const double goal_bias = planning_config::RRT_GOAL_BIAS);


    /**
     * @brief Determines whether two points are within tolerance distance of one another. Used to represent a goal zone around the desired goal. 
     * 
     * @param point1 (std::vector<double>): First point
     * @param point2 (std::vector<double>): Second point
     * @param proximity_tolerance (double): Meters of tolerance
     * @return (bool): Whether the two points are within tolerance distance
     */
    [[nodiscard]] bool is_near(const std::vector<double>& point1, const std::vector<double>& point2, const double proximity_tolerance = planning_config::RRT_STEP_SIZE) noexcept;
    
    /**
     * @brief Determines if the straight-line path between two points is collision free. Completes the check by stepping along the segment, from `point1` to `point2` with
     * step size `step_size`. Returns `false` as soon as any step point collides with any obstacle, otherwise returns `true`. 
     * 
     * @param obstacles (ObstacleProcessor): Obstacle data structure
     * @param point1 (std::vector<double>): First point
     * @param point2 (std::vector<double>): Second point
     * @param step_size (double): Collision check step size
     * @return (bool): Determines whether the straight-line path between two points is collision-free. 
     */
    [[nodiscard]] bool is_path_clear(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<double>& point1, const std::vector<double>& point2, const double step_size = planning_config::RRT_STEP_SIZE);
    
    /**
     * @brief Generates a point along the straight-line segment from `base_point` to `target_point`, `step_size` away from `base_point` and in the direction of `target_point`.
     * 
     * @param base_point (std::vector<double>): Base point
     * @param target_point (std::vector<double>): Target point
     * @param step_size (double): Distance from base point at which to generate the new point
     * @return (std::vector<double>): The point along the straight-line segment from `base_point` to `target_point`, `step_size` away from `base_point` and in the direction of `target_point`.
     */
    [[nodiscard]] std::vector<double> step_forward(const std::vector<double>& base_point, const std::vector<double>& target_point, const double step_size = planning_config::RRT_STEP_SIZE) noexcept;
    
    /**
     * @brief Prunes the input path for shortcuts. Implements a greedy shortcutting algorithm. 
     * 
     * @param obstacles (ObstacleProcessor): Obstacle data structure
     * @param path (std::vector<std::vector<double>>): Input waypoint plan
     * @return (std::vector<std::vector<double>>): New waypoint plan, shortcuts applied 
     */
    [[nodiscard]] std::vector<std::vector<double>> shortcut(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<std::vector<double>>& path);
    
    /**
     * @brief Refines an existing waypoint plan by interpolating additional points along concecutive straight-line segments 
     * 
     * @param path (std::vector<std::vector<double>>): Input waypoint plan
     * @param path_spacing (double): Meters of spacing between interpolated points 
     * @return (std::vector<std::vector<double>>): The new waypoint plan
     */
    [[nodiscard]] std::vector<std::vector<double>> discretize_path(const std::vector<std::vector<double>>& path, const double path_spacing = planning_config::LOCAL_PATH_SPACING);
    



}