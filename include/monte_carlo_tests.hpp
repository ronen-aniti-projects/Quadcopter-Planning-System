#include "obstacle_processor.hpp"
#include "local_planning.hpp"
#include "global_planning.hpp"
#include <vector>

namespace monte_carlo_trials{
    void display_points(const std::vector<std::vector<double>>& points) noexcept;
    void evaluate_global_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space);
    void evaluate_local_planner(const obstacle_processor::ObstacleProcessor& obstacles, const global_planning::FreeSpaceGraph& free_space, const double goal_bias);
} // namespace monte_carlo_trials
