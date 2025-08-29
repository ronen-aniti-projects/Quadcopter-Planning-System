#include <iostream>
#include <string>
#include "obstacle_processor.hpp"
#include "global_planning.hpp"
#include "monte_carlo_tests.hpp"
#include "utils.hpp"

int main(){
   
   // Generate an ObstacleProcessor class from the obstacle data
   const obstacle_processor::ObstacleProcessor obstacles{obstacle_processor::ObstacleProcessor("data/obstacle-data.csv")};
   
   // Prompt the user for some planning parameters
   const double global_resolution{helpers::ask_double("Enter Global Planner Resolution (Recommended: 25): ")};
   const double local_goal_bias{helpers::ask_double("Enter RRT Goal Bias Ratio (0 to 1): ")};

   // Generate a cubic-lattice free space representation
   std::cout << "Generating a representation of free space.." << '\n';
   const global_planning::FreeSpaceGraph free_space{global_planning::FreeSpaceGraph(obstacles, global_resolution)};
   std::cout << "Complete" << '\n';

   // Run the Monte Carlo performance assessment
   monte_carlo_trials::evaluate_global_planner(obstacles, free_space);
   monte_carlo_trials::evaluate_local_planner(obstacles, free_space, local_goal_bias);

   return 0;
}