#pragma once 

#include <optional>
#include "utils.hpp"
#include "obstacle_processor.hpp"
#include "KDTree.hpp"

namespace local_planning{ 

    [[nodiscard]] std::optional<std::vector<std::vector<double>>> rrt(const obstacle_processor::ObstacleProcessor& obstacles, 
            const std::vector<double>& start_point, 
            const std::vector<double>& goal_point,
            const double step_size = planning_config::RRT_STEP_SIZE,
            const double edge_validation_step_size = planning_config::RRT_EDGE_VALIDATION_STEP_SIZE,
            const size_t max_iterations = planning_config::RRT_ITERATIONS,
            const double goal_bias = planning_config::RRT_GOAL_BIAS);


    
    [[nodiscard]] bool is_near(const std::vector<double>& point1, const std::vector<double>& point2, const double proximity_tolerance = planning_config::RRT_STEP_SIZE) noexcept;
    [[nodiscard]] bool is_path_clear(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<double>& point1, const std::vector<double>& point2, const double step_size = planning_config::RRT_STEP_SIZE);
    [[nodiscard]] std::vector<double> step_forward(const std::vector<double>& base_point, const std::vector<double>& target_point, const double step_size = planning_config::RRT_STEP_SIZE) noexcept;
    [[nodiscard]] std::vector<std::vector<double>> shortcut(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<std::vector<double>>& path);
    [[nodiscard]] std::vector<std::vector<double>> discretize_path(const std::vector<std::vector<double>>& path, const double path_spacing = planning_config::LOCAL_PATH_SPACING);
    



}