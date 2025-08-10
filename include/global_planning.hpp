#pragma once 

#include <optional>
#include "types.hpp"
#include "obstacle_processor.hpp"

namespace GlobalPlanning{

    class FreeSpaceGraph {
    
        public: 
            /**
             * @brief Construct a 3D cubic lattice graph representation of free space
             * 
             * @param obstacles The obstacle data object 
             * @param resolution The resolution of the graph
             */
            FreeSpaceGraph(const obstacle_processor::ObstacleData& obstacles, const double resolution = planning_config::GLOBAL_RESOLUTION);
            
            /**
             * @brief Destroy the FreeSpaceGraph object
             * 
             */
            ~FreeSpaceGraph() = default;

            /**
             * @brief Searches the 
             * 
             * @param start 
             * @param goal 
             * @return std::vector<types::Point3D> 
             */
            [[nodiscard]] std::optional<std::vector<types::Point3D>> search(const types::Point3D& start, const types::Point3D goal) const;
        
        private: 

            /**
             * @brief All of the (x, y, z) nodes of the graph structure 
             * 
             */
            std::vector<types::Point3D> nodes_;

            /**
             * @brief A data structure describing the connectivity of the graph
             * @note Each edge is weighted with the Euclidean distance between its nodes. 
             * 
             */
            std::vector<std::vector<std::pair<int, double>>> edges_;



    };


}