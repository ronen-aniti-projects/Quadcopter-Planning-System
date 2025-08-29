/**
 * @file global_planning.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Header file for Global Planning module of C++ path and trajectory planning project.
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
 * @namespace global_planning
 * @brief Tools for global path planning
 * 
 */
namespace global_planning{

    /**
     * @class FreeSpaceGraph
     * @brief Tool for constructing a graph representation of free space
     */
    class FreeSpaceGraph {
    
        public: 

            /**
             * @brief Constructs a new Free Space Graph object.
             * 
             * @param obstacles (ObstacleProcessor): Obstacle data object
             * @param resolution (double): Meters of spacing between nodes
             */
            FreeSpaceGraph(const obstacle_processor::ObstacleProcessor& obstacles, const double resolution = planning_config::GLOBAL_RESOLUTION);
            
            
            /**
             * @brief Destroys the Free Space Graph object.
             * 
             */
            ~FreeSpaceGraph() = default;
            
            /**
             * @brief Implements A* graph search. 
             * 
             * @param start (std::vector<std::vector<double>>): The 3D start point
             * @param goal (std::vector<std::vector<double>>): The 3D goal point
             * @return (std::optional<std::vector<std::vector<double>>>): A path of 3D points if a path exists
             */
            [[nodiscard]] std::optional<std::vector<std::vector<double>>> search(const std::vector<double>& start, const std::vector<double>& goal) const;
            
            /**
             * @brief Gets the vector of 3D nodal coordinates of all graph nodes.
             * 
             * @return (std::vector<std::vector<double>>): The vector of 3D points of all nodal graph coordinates 
             */
            [[nodiscard]] std::vector<std::vector<double>> get_nodes() const{return nodes_;}

            /**
             * @brief Gets the data structure defining the edge connectivity and weights of the graph.
             * 
             * @return (std::vector<std::vector<std::pair<size_t, double>>>): The data structure defining the edge connectivity and weights of the graph.
             */
            [[nodiscard]] std::vector<std::vector<std::pair<size_t, double>>> get_edges() const{return edges_;}
    
        private: 
            double resolution_; 
            obstacle_processor::ObstacleProcessor obstacles_;
            std::vector<std::vector<double>> nodes_;
            mutable KDTree nodes_kd_;
            std::vector<std::vector<std::pair<size_t, double>>> edges_;
            [[nodiscard]] bool validate_edge(const std::vector<double> &point_1, const std::vector<double> &point_2, const double step_size = planning_config::EDGE_VALIDATION_STEP_SIZE) const;
            [[nodiscard]] std::vector<std::vector<double>> backtrack(std::unordered_map<size_t, size_t> parents, size_t goal_idx, size_t start_idx) const;
            
    };


}