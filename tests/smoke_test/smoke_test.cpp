/**
 * @file smoke_test.cpp
 * @author Ronen Aniti (raniti@umd.edu)
 * @brief This file contains assertion based tests to aid with the development of the Quadcopter Planning System C++ project.
 * @version 0.1
 * @date 2025-08-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "obstacle_processor.hpp" 
#include "global_planning.hpp"
#include "utils.hpp"

int main() {

    try{
        
        // Test that the object can be constructed given an obstacle data file
        auto obs{obstacle_processor::ObstacleProcessor("data/obstacle-data.csv")};

        // Test of `is_collision` method: 3 test cases
        assert(!obs.is_collision(std::vector<double>{0.0, 0.0, 0.0}) && "expected no collision");
        assert(!obs.is_collision(std::vector<double>{0.0, 0.0, 200.0}) && "expected no collision");
        assert(obs.is_collision(std::vector<double>{-310.2389, -439.2315, 85.5}) && "expected collision");
        
        // Test of `distance_from_obstacles` method: 1 test case
        double dist = obs.distance_from_obstacle(std::vector<double>{-310.2389, -439.2315, 85.5});
        assert(std::abs(dist - 0.0) < 1e-9 && "expected distance 0.0 for colliding point");



        auto free_space{global_planning::FreeSpaceGraph(obs)};
        std::vector<std::vector<double>> nodes{free_space.get_nodes()};
        std::vector<std::vector<std::pair<size_t, double>>> edges{free_space.get_edges()};
        std::vector<double> start{0.0, 0.0, 0.0};
        std::vector<double> goal{200.0, 200.0, 212.0};
        const auto& path{free_space.search(start, goal)};
        
        assert(path.has_value() && "Expected valid path");
        if (path.has_value()){
            for (size_t i{0}; i < path.value().size(); ++i){
                std::cout << "(";
                for (size_t j{0}; j < path.value().at(i).size(); ++j){
                    if (j == path.value().at(i).size()-1){
                        std::cout << path.value().at(i).at(j) << "";
                    } else {
                        std::cout << path.value().at(i).at(j) << ", ";
                    }
                    
                }
                std::cout << ")" << '\n';
            }
        }

        // Prints this message when all tests pass, otherwise will abort with message after failing any assert.
        std::cout << "[PASS] all checks\n";


    } catch (const std::exception& e){

        // Shows this error when my implementation of any of the tested methods throws an exception
        std::cerr << "[FAIL] exception: " << e.what() <<  '\n';
    }
    
    return 0;

}