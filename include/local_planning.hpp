/**
 * @file local_planning.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Header file for Local Planning module of C++ integrated path and trajectory planning project.
 * @version 0.1
 * @date 2025-08-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once 

#include <optional>
#include "utils.hpp"
#include "obstacle_processor.hpp"
#include "KDTree.hpp"

/**
 * @namespace local_planning
 * @brief Provides tools for local path plannning
 * 
 */
namespace local_planning{ 

    /**
     * @brief Implements Rapidly Exploring Random Tree (RRT) pathfinding
     * 
     * @param obstacles The object containing obstacle data
     * @param start_point The 3D start point
     * @param goal_point The 3D goal point
     * @param step_size The RRT step size
     * @param edge_validation_step_size The step size used to whether an edge is collision-free
     * @param max_iterations The maximum number of iterations before RRT returns no path
     * @param goal_bias The goal bias, a probability of sampling the goal node instead of a random node
     * @return (std::optional<std::vector<std::vector<double>>>): A path of 3D points from start to goal 
     */
    [[nodiscard]] std::optional<std::vector<std::vector<double>>> rrt(const obstacle_processor::ObstacleProcessor& obstacles, 
            const std::vector<double>& start_point, 
            const std::vector<double>& goal_point,
            const double step_size = planning_config::RRT_STEP_SIZE,
            const double edge_validation_step_size = planning_config::RRT_EDGE_VALIDATION_STEP_SIZE,
            const size_t max_iterations = planning_config::RRT_ITERATIONS,
            const double goal_bias = planning_config::RRT_GOAL_BIAS);


    /**
     * @brief Determines whether two 3D points are within a tolerance distance of one another.
     * 
     * @param point1 The first 3D point
     * @param point2 The second 3D point
     * @param proximity_tolerance The tolerance distance
     * @return true 
     * @return false 
     */
    [[nodiscard]] bool is_near(const std::vector<double>& point1, const std::vector<double>& point2, const double proximity_tolerance = planning_config::RRT_STEP_SIZE) noexcept;
    
    /**
     * @brief Determines whether the straight line path between two points is collision free.
     * 
     * @param obstacles The obstacle data object referenced for collision checking
     * @param point1 The first 3D point
     * @param point2 The second 3D point
     * @param step_size The step size used for collision-checking
     * @return true 
     * @return false 
     */
    [[nodiscard]] bool is_path_clear(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<double>& point1, const std::vector<double>& point2, const double step_size = planning_config::RRT_STEP_SIZE);
    
    /**
     * @brief Given a base point and a target point, returns a third point, a step size away from 
     * the base point, in the direction of the taget point.  
     * 
     * @param base_point The 3D base point
     * @param target_point The 3D target point 
     * @param step_size The size (meters) of the step
     * @return (std::vector<double>): The new 3D point 
     */
    [[nodiscard]] std::vector<double> step_forward(const std::vector<double>& base_point, const std::vector<double>& target_point, const double step_size = planning_config::RRT_STEP_SIZE) noexcept;
    
    /**
     * @brief Implements a greedy path shortcutting algorithm
     * 
     * @param obstacles The obstacle data object 
     * @param path The input path
     * @return (std::vector<std::vector<double>>): The output shortcut path 
     */
    [[nodiscard]] std::vector<std::vector<double>> shortcut(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<std::vector<double>>& path);
    
    /**
     * @brief Given an input path of 3D points, returns a path having the same general shape but ensures
     * the presence of intermediate waypoints spaced `path_spacing` along each segment. 
     * 
     * @param path The input path of 3D waypoint 
     * @param path_spacing The spacing between intermediate waypoints of the output path
     * @return (std::vector<std::vector<double>>): The output waypoint path
     */
    [[nodiscard]] std::vector<std::vector<double>> discretize_path(const std::vector<std::vector<double>>& path, const double path_spacing = planning_config::LOCAL_PATH_SPACING);
    



}