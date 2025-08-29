#include <optional>
#include <cmath>
#include <random>
#include <limits>
#include "utils.hpp"
#include "obstacle_processor.hpp"
#include "local_planning.hpp"
#include "KDTree.hpp"

namespace local_planning{
    
    std::optional<std::vector<std::vector<double>>>rrt(const obstacle_processor::ObstacleProcessor& obstacles, 
            const std::vector<double>& start_point, 
            const std::vector<double>& goal_point,
            const double step_size,
            const double edge_validation_step_size,
            const size_t max_iterations,
            const double goal_bias){
                

                // Initialize RRT Data Structures
                std::vector<int> parents;
                std::vector<std::vector<double>> nodes;
                
                // Extract Obstacle Bounds
                const double xlow{obstacles.get_x_lim().at(0)};
                const double xhigh{obstacles.get_x_lim().at(1)};
                const double ylow{obstacles.get_y_lim().at(0)};
                const double yhigh{obstacles.get_y_lim().at(1)};
                const double zlow{obstacles.get_z_lim().at(0)};
                const double zhigh{obstacles.get_z_lim().at(1)};
                
                // Create Random Number Generator
                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_real_distribution<double> dist(0.0, 1.0);

                // Add the start node to the tree
                nodes.push_back(start_point); // start is index 0
                parents.push_back(-1); // start's parent is special index -1

                // Flag for whether goal is found
                bool is_goal_found{false};

                // RRT main loop
                std::vector<double> sample_point(3);
                std::vector<double> advanced_point(3);
                double dist_to_sample{};
                for (size_t i{0}; i < max_iterations; ++i){
                    
                    if (dist(gen) < goal_bias){
                        sample_point = goal_point;
                    } else {
                        sample_point = {xlow + dist(gen) * (xhigh - xlow), ylow + dist(gen) * (yhigh - ylow), zlow + dist(gen) * (zhigh - zlow)};
                    }
                    

                    // Find coordinates and index of the closest point to the sample_point
                    double min_dist{std::numeric_limits<double>::infinity()};
                    size_t min_idx{};
                    for (size_t j{0}; j < nodes.size(); ++j){
                        dist_to_sample = helpers::distance_between_points(sample_point, nodes.at(j));
                        if (dist_to_sample < min_dist){
                            min_dist = dist_to_sample;
                            min_idx = j;
                        }

                    }

                    // Advance the nearest point forward by one step
                    advanced_point = step_forward(nodes.at(min_idx), sample_point, step_size);
                    
                    // If the advanced point is in collision with obstacle, SKIP this RRT iteration
                    if (!is_path_clear(obstacles, nodes.at(min_idx), advanced_point, edge_validation_step_size)){
                        continue;
                    }

                    // If there's not a collision with obstacle, add the advanced point to the tree
                    nodes.push_back(advanced_point);
                    parents.push_back(min_idx);
                    const int parent_of_goal_idx = nodes.size() - 1;

                    // If the advanced point is close enough to the goal point, then stop searching and return the path.
                    if (is_near(advanced_point, goal_point, step_size)){
                        is_goal_found = true;

                        // Add the goal to the tree
                        // Assume collision free with close enough point for the time being (reasonable for now because step_size will be small)
                        std::vector<std::vector<double>> path{};
                        nodes.push_back(goal_point);
                        parents.push_back(parent_of_goal_idx);
                       
                        break;
                    }



                } // RRT main loop

            
            // If a goal is found, return the path after backtracking.
            if (is_goal_found){

                std::vector<std::vector<double>> path;
                int current_idx{static_cast<int>(nodes.size()) - 1};

                while(current_idx != -1){
                    path.push_back(nodes.at(current_idx));
                    current_idx = parents.at(current_idx);
                }
                std::reverse(path.begin(), path.end());
                return path;

            }

            // If the goal is not found, do not return a path
            return std::nullopt;



            } // RRT



    bool is_near(const std::vector<double>& point1, const std::vector<double>& point2, const double proximity_tolerance) noexcept{
        const double dx{point1.at(0) - point2.at(0)};
        const double dy{point1.at(1) - point2.at(1)};
        const double dz{point1.at(2) - point2.at(2)};
        return sqrt(dx * dx + dy * dy + dz * dz) <= proximity_tolerance;
    }

    bool is_path_clear(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<double>& point_1, const std::vector<double>& point_2, const double step_size){

        // Extract coordinates of first point
        const double x1{point_1[0]};
        const double y1{point_1[1]};
        const double z1{point_1[2]};

        // Extract coordinates of second point
        const double x2{point_2[0]};
        const double y2{point_2[1]};
        const double z2{point_2[2]};
        
        // Calculate the components of the displacement
        const double dx{x2 - x1};
        const double dy{y2 - y1};
        const double dz{z2 - z1};

        // Calculate the edge length
        const double edge_length{sqrt(dx * dx + dy * dy + dz * dz)};

        // Guard against input points being at the same location
        // If an edge is zero length, then it's not a valid edge.
        if (edge_length < planning_config::FLOATING_POINT_COMPARISON_TOLERANCE){
            return false;
        }
        
        const double step_size_normalized{step_size / edge_length};

        // Check along the segment for collisions at intermediate points (between end points). 
        // Return false as soon as one is found. Return true only if none are found. 
        std::vector<double> point(3);
        for (double step_parameter{step_size_normalized}; step_parameter < 1.0; step_parameter += step_size_normalized){
            point[0] = x1 + step_parameter * dx;
            point[1] = y1 + step_parameter * dy;
            point[2] = z1 + step_parameter * dz;
            if (obstacles.is_collision(point)){
                return false;
            }

        }
        return true;

    } // is_path_clear

    std::vector<double> step_forward(const std::vector<double>& base_point, const std::vector<double>& target_point, const double step) noexcept{
        // Extract coordinates of first point
        const double x1{base_point[0]};
        const double y1{base_point[1]};
        const double z1{base_point[2]};

        // Extract coordinates of second point
        const double x2{target_point[0]};
        const double y2{target_point[1]};
        const double z2{target_point[2]};
        
        // Calculate the components of the displacement
        const double dx{x2 - x1};
        const double dy{y2 - y1};
        const double dz{z2 - z1};

        // Calculate the edge length
        const double edge_length{sqrt(dx * dx + dy * dy + dz * dz)};

        // Calculate unit vector components
        const double ux{dx / edge_length};
        const double uy{dy / edge_length};
        const double uz{dz / edge_length};

        const double x{x1 + step * ux};
        const double y{y1 + step * uy};
        const double z{z1 + step * uz};
        return std::vector<double>{x, y, z};

    }

    std::vector<std::vector<double>> shortcut(const obstacle_processor::ObstacleProcessor& obstacles, const std::vector<std::vector<double>>& path){
        
        size_t end_index {path.size() - 1};
        size_t start_index{0};
        size_t current_end_index{end_index};
        size_t current_start_index{start_index};
        std::vector<size_t> shortcut_indices;
        shortcut_indices.push_back(end_index);

        while (current_start_index < current_end_index){
            bool shortcut_found{false};
            while (current_start_index < current_end_index){
                if (is_path_clear(obstacles, path.at(current_start_index), path.at(current_end_index)) ){
                    shortcut_indices.push_back(current_start_index);
                    shortcut_found = true;
                    break;

                } else {
                    current_start_index++; 
                }   
            }
            if (shortcut_found){
                current_end_index = shortcut_indices.back();
                current_start_index = 0;


            } else {
                current_end_index--;
                shortcut_indices.push_back(current_end_index);
                current_start_index = 0;
            }
        }
        std::reverse(shortcut_indices.begin(), shortcut_indices.end());
        std::vector<std::vector<double>> shortcut_path;
        for (size_t index : shortcut_indices){
            shortcut_path.push_back(path.at(index));
        }
        return shortcut_path;



        
        
    }

    std::vector<std::vector<double>> discretize_path(const std::vector<std::vector<double>>& path, const double path_spacing){
        
        std::vector<std::vector<double>> processed_path;
        std::vector<double> point1(3);
        std::vector<double> point2(3); 

        double x1{};
        double x2{};
        double y1{};
        double y2{};
        double z1{};
        double z2{};
        double dx{};
        double dy{};
        double dz{};
        double segment_length{};
        int num_steps{};

        // For each base point up until the second to last, add the base 
        // point and all intermediate, interpolated points in the direction
        // of the next waypoint.  
        for (size_t i{0}; i < path.size() - 1; ++i){
            point1 = path.at(i);
            point2 = path.at(i+1);

            x1 = point1.at(0);
            x2 = point2.at(0);
            y1 = point1.at(1);
            y2 = point2.at(1);
            z1 = point1.at(2);
            z2 = point2.at(2);

            dx = x2 - x1;
            dy = y2 - y1;
            dz = z2 - z1;

            segment_length = sqrt(dx * dx + dy * dy + dz * dz);
           

            num_steps = static_cast<int>(segment_length / path_spacing);

            // Add all itermediate points to the processed path. Do not include
            // the next waypoint in the loop.
            // Assume segment length is never zero
            std::vector<double> intermediate_point; 
            for (int j{0}; j < num_steps; ++j){
                intermediate_point = {x1 + static_cast<double>(j) / static_cast<double>(num_steps) * dx,
                                      y1 + static_cast<double>(j) / static_cast<double>(num_steps) * dy, 
                                      z1 + static_cast<double>(j) / static_cast<double>(num_steps) * dz};

                processed_path.push_back(intermediate_point);
            } // iterate over all intermediate points of the segment
        } // iterate over all base points up to the second to last one
        
        processed_path.push_back(path.back());

        return processed_path;
    }

    

} // namespace local_planning