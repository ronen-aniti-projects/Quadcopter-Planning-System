#pragma once 

#include <optional>
#include "utils.hpp"
#include "obstacle_processor.hpp"
#include "KDTree.hpp"

namespace global_planning{

    class FreeSpaceGraph {
    
        public: 
            FreeSpaceGraph(const obstacle_processor::ObstacleProcessor& obstacles, const double resolution = planning_config::GLOBAL_RESOLUTION);
            ~FreeSpaceGraph() = default;
            [[nodiscard]] std::optional<std::vector<std::vector<double>>> search(const std::vector<double>& start, const std::vector<double>& goal) const;
            [[nodiscard]] std::vector<std::vector<double>> get_nodes() const{return nodes_;}
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