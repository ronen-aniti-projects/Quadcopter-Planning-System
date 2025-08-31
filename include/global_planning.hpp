/**
 * @file global_planning.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides tools for global path planning, including free-space graph generation and A* pathfinding. 
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
 * @namespace global_planning
 * @brief Provides tools for global path planning.
 * 
 */
namespace global_planning{

    /**
     * @class FreeSpaceGraph
     * @brief This class is a tool for constructing cubic lattice graph representations of free space given 3D bounding box maps of obstacle space
     */
    class FreeSpaceGraph {
    
        public: 

            /**
             * @brief Constructs a new Free Space Graph object.
             * 
             * @param obstacles (ObstacleProcessor): Obstacle data object
             * @param resolution (double): Desired meters of spacing between nodes of free space
             */
            FreeSpaceGraph(const obstacle_processor::ObstacleProcessor& obstacles, const double resolution = planning_config::GLOBAL_RESOLUTION);
            
            
            /**
             * @brief Destroys the Free Space Graph object.
             * 
             */
            ~FreeSpaceGraph() = default;
            
            /**
             * @brief Implements A* search through the constructed graph representation of free space.
             * 
             * @param start (std::vector<std::vector<double>>): 3D start point
             * @param goal (std::vector<std::vector<double>>): 3D goal point
             * @return (std::optional<std::vector<std::vector<double>>>): Waypoint sequence from start to goal
             */
            [[nodiscard]] std::optional<std::vector<std::vector<double>>> search(const std::vector<double>& start, const std::vector<double>& goal) const;
            
            /**
             * @brief Gets nodes of the free space graph.
             * 
             * @return (std::vector<std::vector<double>>): Graph node coordinates 
             */
            [[nodiscard]] std::vector<std::vector<double>> get_nodes() const{return nodes_;}

            /**
             * @brief Gets the weighted edges of the free space graph, with weights corresponding to Euclidean length of the edge.
             * 
             * @return (std::vector<std::vector<std::pair<size_t, double>>>): Weighted graph edges
             */
            [[nodiscard]] std::vector<std::vector<std::pair<size_t, double>>> get_edges() const{return edges_;}
    
        private: 

            /**
             * @brief Spacing beteween adjacent graph nodes,
             * 
             */
            double resolution_; 

            /**
             * @brief Obstacle data object, which is used for collision checking
             * 
             */
            obstacle_processor::ObstacleProcessor obstacles_;

            /**
             * @brief Nodes of the graph structure
             * 
             */
            std::vector<std::vector<double>> nodes_;

            /**
             * @brief KDTree of the graph nodes. Used for fast collision checking
             * 
             */
            mutable KDTree nodes_kd_;

            /**
             * @brief Weighted edges of the graph structure
             * 
             */
            std::vector<std::vector<std::pair<size_t, double>>> edges_;

            /**
             * @brief Aids in collision checking along the straight-line distance between two points.
             * 
             * @param point_1 (std::vector<double>): First point
             * @param point_2 (std::vector<double>): Second point
             * @param step_size (double): Step size used for collision checking.
             * @return (bool): Whether the straight-line distance between two points is collision-free 
             */
            [[nodiscard]] bool validate_edge(const std::vector<double> &point_1, const std::vector<double> &point_2, const double step_size = planning_config::EDGE_VALIDATION_STEP_SIZE) const;
            
            /**
             * @brief Backtracks a waypoint plan when A* finds the goal.
             * 
             * @param parents (std::unordered_map<size_t, size_t>): Mapping of each graph node to its parent
             * @param goal_idx (size_t): Node index of goal
             * @param start_idx (size_t): Node index of start
             * @return (std::vector<std::vector<double>>): Waypoint plan
             */
            [[nodiscard]] std::vector<std::vector<double>> backtrack(std::unordered_map<size_t, size_t> parents, size_t goal_idx, size_t start_idx) const;
            
    };


}