#include <iostream>
#include "obstacle_processor.hpp"
#include "global_planning.hpp"

int main(){
   auto obs{obstacle_processor::ObstacleProcessor("data/obstacle-data.csv")};
   auto free_space{global_planning::FreeSpaceGraph(obs, planning_config::GLOBAL_RESOLUTION)};

   
   return 0;
}