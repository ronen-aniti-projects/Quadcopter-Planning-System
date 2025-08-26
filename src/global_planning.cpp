#include "obstacle_processor.hpp"
#include "global_planning.hpp"
#include "utils.hpp"
#include <optional>
#include <iostream>
#include <cmath>
#include <queue>
#include <iostream>
#include <unordered_map> 
#include <unordered_set>
#include <algorithm>

namespace global_planning{

    FreeSpaceGraph::FreeSpaceGraph(const obstacle_processor::ObstacleProcessor& obstacles, const double resolution)
    : resolution_{resolution}, obstacles_{obstacles} {

        // Generate a 3D grid of equispaced points belonging to [xlow xhigh] x [ylow yhigh] x [zlow zhigh]
        for (double x{obstacles_.get_x_lim().at(0)}; x<= obstacles_.get_x_lim().at(1); x += resolution){
            for (double y{obstacles_.get_y_lim().at(0)}; y <= obstacles_.get_y_lim().at(1); y += resolution){
                for (double z{obstacles_.get_z_lim().at(0)}; z <= obstacles_.get_z_lim().at(1); z += resolution){
                    // Optimize this section in the next refactor:
                    // I can overload collision check to take double, thereby avoiding double heap
                    // allocation.
                    const std::vector<double> point{x, y, z};
                    if (!obstacles_.is_collision(point)){
                        nodes_.push_back(point);
                    }
                }
            }
        }

        // Build a KDTree with the nodes.
        nodes_kd_ = KDTree{nodes_};

        // Reize edges vector to match the number of nodes
        edges_.resize(nodes_.size());

        // Build an adjacency list (`edges_`) to describe the graph connecivity
        // for graph node i: [[...], [neighbor_index, distance_to_neighbor], [...]]
        
        // Iterate through graph node 0 to n
        // For each graph node:
        //   - Return neighbors based on KD radius lookup
        //   - For each neighbor: 
        //     - Collision check the line segment connecting the two nodes at discrete places with `is_collision`. 
        //     - If any discrete location is in collision, skip that neighbor, otherwise createa pair



        // Add a small tolerance to the radius used for neighbor lookups
        const double radius{resolution * 1.01};
        
        
        // --- Iterate over all nodes of free space
        for (size_t node_idx{0}; node_idx < nodes_.size(); ++node_idx){
            
            // An object holding the node (x, y, z) coordinates
            const std::vector<double>& node{nodes_.at(node_idx)};

            // An object holding a vector of neighbors (x, y, z) paired with neighbor indices (i)
            // Neighbors are found using a search radius based on global resolution
            const std::vector<std::pair<std::vector<double>, size_t>>& point_array_index{nodes_kd_.neighborhood(node, radius)};
            
            // Declare a vector to hold (idx, dist) for each of node i's neighbors.
            std::vector<std::pair<size_t, double>> valid_neighbors; 

            // Reserve memory equal to size of the KD neighborhood return query (a practical upper limit)
            valid_neighbors.reserve(point_array_index.size());

            // --- Iterate over all neighbors of the selected node
            for (const auto& [neighbor_node, neighbor_idx] : point_array_index){
                
                
                if (neighbor_idx == node_idx){
                    continue;
                }
                const double dist_between{helpers::distance_between_points(neighbor_node, node)};

                // Keep only collision free edges
                if (validate_edge(node, neighbor_node)){
                    valid_neighbors.emplace_back(neighbor_idx, dist_between);
                }
            } // end iterate over neighbors of i-th node

            // Move the valid neighbors vector into the larger edges vector
            edges_.at(node_idx) = std::move(valid_neighbors);

        } // end iterate over nodes
    } // end constructor
            
    bool FreeSpaceGraph::validate_edge(const std::vector<double> &point_1, const std::vector<double> &point_2, const double step_size) const{

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
            if (obstacles_.is_collision(point)){
                return false;
            }

        }
        return true;

    } // validate_edge



std::optional<std::vector<std::vector<double>>> FreeSpaceGraph::search(const std::vector<double>& start, const std::vector<double>& goal) const{

    // Type aliases for readability
    using QItem = std::pair<double, size_t>; // (q_score, node_idx)

    // Extract nearest nodes to query points for start and goal
    const auto [start_pt, start_idx] = nodes_kd_.nearest_pointIndex(start);
    const auto [goal_pt, goal_idx] = nodes_kd_.nearest_pointIndex(goal);

    // Declare the A* data structures
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> queue;
    std::unordered_set<size_t> visited_set;
    std::unordered_set<size_t> open_set;
    std::unordered_map<size_t, double> g_scores; // node_idx: g_score
    std::unordered_map<size_t, size_t> parents; // child: parent
    bool is_goal_found{false};

    // Initialize start g_score and parent (parent of itself)
    g_scores.insert_or_assign(start_idx, 0.0);
    parents.insert_or_assign(start_idx, start_idx);

    // Place start in open_set and in queue with f-score equal to Euclid. dist from start to goal 
    // Reminder: open_set is seen but not visited
    // Reminder: visited_set is visited (so also seen)
    // Reminder: queue can contain the same node but with different scores
    queue.emplace(helpers::distance_between_points(nodes_.at(start_idx), goal_pt), start_idx);
    open_set.insert(start_idx);

    // A* main loop
    while (!queue.empty()){

        // Access and pop the most promising node
        const auto [queue_score, current_idx] = queue.top();
        queue.pop(); 

        // Ask: Has it been visited? If so, skip it. 
        if (visited_set.count(current_idx)){
            continue;
        }

        // Mark node as visited immedietely after checking that it's not the goal
        visited_set.insert(current_idx);
        open_set.erase(current_idx);
        
        // Ask: Is this the goal? If so, BREAK.
        if (current_idx == goal_idx){
            is_goal_found = true;
            std::cout << "A* found the goal" << '\n';
            break;
        }

        // Iterate over each neighbor of current
        // For each: 
        // - Ask: Has it been visited? SKIP if yes. 
        // - Ask: Has it been seen? 
        //     If no, 
        //        add neighbor to g_score, parents, queue, open
        //     If yes,
        //        compute tentative g score for neighbor by taking current's g-score and adding distance between current and neighbor
        //        Ask: If tentative lower than what's already recorded for neighbor?
        //          If yes, 
        //              Update g_score, parents, queue, open for neighbor using tentative g_score
        //          If no, 
        //              SKIP (there's a better partial path already known)
        for (size_t neighbor_iter_idx{0}; neighbor_iter_idx < edges_.at(current_idx).size(); ++neighbor_iter_idx){
            const auto& [neighbor_idx, dist] = edges_.at(current_idx).at(neighbor_iter_idx);
            
            if (visited_set.count(neighbor_idx)){
                continue;
            }

            if (open_set.count(neighbor_idx)){ // neighbor has been seen
                const double tentative_g{g_scores[current_idx] + dist};
                if (tentative_g < g_scores[neighbor_idx]){ // If cheaper route
                    parents[neighbor_idx] = current_idx;
                    g_scores[neighbor_idx] = tentative_g;
                    const double q_score = tentative_g + helpers::distance_between_points(nodes_.at(neighbor_idx), nodes_.at(goal_idx));
                    queue.emplace(q_score, neighbor_idx);
                } else { // If not cheapter route
                    continue;
                }

            } else { // neighbor has not been seen
                g_scores[neighbor_idx] = g_scores[current_idx] + dist;
                parents[neighbor_idx] = current_idx;
                const double q_score = g_scores[neighbor_idx] + helpers::distance_between_points(nodes_.at(neighbor_idx), nodes_.at(goal_idx));
                queue.emplace(q_score, neighbor_idx);
                open_set.insert(neighbor_idx);
            }

        


        }  // Neighbor iteration

    } // A* main loop

    if (is_goal_found){
        const auto path{backtrack(parents, goal_idx, start_idx)};
        return path;
    }
    return std::nullopt;



}

std::vector<std::vector<double>> FreeSpaceGraph::backtrack(std::unordered_map<size_t, size_t> parents, size_t goal_idx, size_t start_idx) const{

    std::vector<std::vector<double>> path;
    path.push_back(nodes_.at(goal_idx));

    size_t current_parent_idx{parents[goal_idx]};
    while (current_parent_idx != start_idx){
        path.push_back(nodes_.at(current_parent_idx));
        current_parent_idx = parents[current_parent_idx];
    }
    path.push_back(nodes_.at(start_idx));
    
    std::reverse(path.begin(), path.end());

    return path;
}

} // namespace global_planning