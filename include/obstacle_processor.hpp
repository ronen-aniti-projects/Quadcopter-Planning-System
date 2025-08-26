#pragma once

/**
 * @file obstacle_processor.hpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief Function declarations for ObstacleProcessor motion planning module
 * @version 1.0
 * @date 2025-07-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <string> 
#include <string_view>
#include <vector> 

#include "utils.hpp"
#include "KDTree.hpp"

namespace obstacle_processor {
    
    class ObstacleProcessor{

        public: 
            ObstacleProcessor(std::string_view filename);
            ~ObstacleProcessor() = default; 
            [[nodiscard]] bool is_collision(const std::vector<double>& query_point) const;
            [[nodiscard]] double distance_from_obstacle(const std::vector<double>& query_point) const;
            [[nodiscard]] std::vector<double> get_x_lim() {return x_lim_;}
            [[nodiscard]] std::vector<double> get_y_lim() {return y_lim_;}
            [[nodiscard]] std::vector<double> get_z_lim() {return z_lim_;}
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