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

#include "types.hpp"
#include "KDTree.hpp"

namespace obstacle_processor {
    

    class ObstacleData{

        public: 

            /**
             * @brief Construct a new Obstacle Data object.
             * 
             * @param filename The filename for the obstacle geometry data.
             */
            ObstacleData(std::string_view filename);

            /**
             * @brief Destroy the Obstacle Data object
             * 
             */
            ~ObstacleData() = default; 

            /**
             * @brief Determine if a point is inside any obstacle bounding box.
             * 
             * @param query_point The query point (x, y, z) coordinates
             * @return bool
             */
            [[nodiscard]] bool is_collision(const types::Point3D& query_point) const;

            /**
             * @brief Calculate the Euclidean distance to the obstacle nearest to a point. 
             * 
             * @param query_point The (x, y, z) coordinates of the point. 
             * @return double 
             */
            [[nodiscard]] double distance_from_obstacle(const types::Point3D& query_point) const;

        private: 

            /**
             * @brief The ground center (x, y) coordinates of each obstacle bounding box
             * 
             */
            std::vector<types::Point2D> obstacle_ground_centers_;

            /**
             * @brief The halfsize dimensions (hx, hy) for each obstacle bounding box
             * 
             */
            std::vector<types::Point2D> obstacle_ground_center_halfsizes_;

            /**
             * @brief The height of each obstacle bounding box
             * 
             */
            std::vector<double> obstacle_heights_;

            /**
             * @brief The halfsize dimension in the z-direction of each obstacle bounding box
             * 
             */
            std::vector<double> obstacle_z_halfsizes_;

            /**
             * @brief The (x, y, z) center coordinates of each obstacle bounding box
             * 
             */
            std::vector<types::Point3D> obstacle_centers_3d_;

            /**
             * @brief The [xmin, xmax] bounds of the entire obstacle data set
             * 
             */
            types::Point2D x_lim_;

            /**
             * @brief The [ymin, ymax] bounds of the entire obstacle data set
             * 
             */
            types::Point2D y_lim_;

            /**
             * @brief The [zmin, zmax] bounds of the entire obstacle data set
             * 
             */
            types::Point2D z_lim_;

            /**
             * @brief A KDTree 
             * 
             */
            KDTree ground_centers_kd_;

    }; // class ObstacleData

} // namespace ObstacleData