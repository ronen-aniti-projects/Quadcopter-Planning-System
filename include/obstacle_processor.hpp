/**
 * @file obstacle_processor.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Provides tools for accessing and processing obstacle geometry data using in path planning.
 * @version 1.0
 * @date 2025-07-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#pragma once

#include "utils.hpp"
#include "KDTree.hpp"

#include <string> 
#include <string_view>
#include <vector> 


/**
 * @namespace obstacle_processor
 * @brief Provides tools for processing obstacle geometry data
 * 
 */
namespace obstacle_processor {
    
    /**
     * @brief Provides tools to read, organize, and process data files containing obstacle bounding box geometry.
     * 
     */
    class ObstacleProcessor{

        public: 
            /**
             * @brief Construct a new Obstacle Processor object
             * 
             * @param filename (std::string_view): Name of obstacle data file
             */
            ObstacleProcessor(std::string_view filename);

            /**
             * @brief Destroy the Obstacle Processor object
             * 
             */
            ~ObstacleProcessor() = default; 

            /**
             * @brief Determines if `query_point` is bounded by any obstacle bouding box.
             * 
             * @param query_point (std::vector<double>): Query point
             * @return (bool): Whether or not `query_point` lies within any obstacle bounding box
             */
            [[nodiscard]] bool is_collision(const std::vector<double>& query_point) const;
            
            /**
             * @brief Determine the Euclidean distance between `query_point` and its nearest obstacle. 
             * 
             * @param query_point (std::vector<double>): Query point
             * @return (double): The Euclidean distance between `query_point` and its nearest obstacle. 
             */
            [[nodiscard]] double distance_from_obstacle(const std::vector<double>& query_point) const;

            /**
             * @brief Get the (xmin, xmax) bounds over the entire space of obstacles
             * 
             * @return (std::vector<double>): (xmin, xmax) of obstacle space 
             */
            [[nodiscard]] std::vector<double> get_x_lim() const {return x_lim_;}

            /**
             * @brief Get the (ymin, ymax) bounds over the entire space of obstacles
             * 
             * @return (std::vector<double>): (ymin, ymax) of obstacle space 
             */
            [[nodiscard]] std::vector<double> get_y_lim() const {return y_lim_;}

            /** 
             * @brief Get the (zmin, zmax) bounds over the entire space of obstacles
             * 
             * @return (std::vector<double>): (zmin, zmax) of obstacle space 
             */
            [[nodiscard]] std::vector<double> get_z_lim() const {return z_lim_;}
        
        private: 
            std::vector<std::vector<double>> obstacle_ground_centers_;
            std::vector<std::vector<double>> obstacle_ground_center_halfsizes_;
            std::vector<double> obstacle_heights_;
            std::vector<double> obstacle_z_halfsizes_;
            std::vector<std::vector<double>> obstacle_centers_3d_;
            std::vector<double> x_lim_;
            std::vector<double> y_lim_;
            std::vector<double> z_lim_;
            mutable KDTree ground_centers_kd_;
            std::vector<std::vector<std::vector<double>>> obstacle_corners_;

    }; // class ObstacleData

} // namespace ObstacleData