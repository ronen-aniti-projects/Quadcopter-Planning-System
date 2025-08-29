#include "obstacle_processor.hpp"
#include "utils.hpp"
#include "KDTree.hpp"

#include <fstream>
#include <filesystem>
#include <stdexcept>
#include <sstream>
#include <string> 
#include <string_view>
#include <vector> 
#include <limits> 
#include <algorithm>
#include <iostream> 
#include <cmath>

namespace obstacle_processor{

    ObstacleProcessor::ObstacleProcessor(std::string_view filename){

        std::ifstream file{std::filesystem::path{filename}};
        if (!file.is_open()) {
            std::ostringstream oss;
            oss << "Unable to open file: " << filename;
            throw std::runtime_error(oss.str());
        }

        std::string line;
        while (std::getline(file, line)){
            std::istringstream ss(line);
            double x{};
            double y{};
            double z{};
            double hx{};
            double hy{};
            double hz{};
            char comma{};

            ss >> x >> comma >> y >> comma >> z >> comma >> hx >> comma >> hy >> comma >> hz;

            obstacle_ground_centers_.emplace_back(std::vector<double>{x, y});
            obstacle_ground_center_halfsizes_.emplace_back(std::vector<double>{hx, hy});
            obstacle_heights_.push_back(hz + hz);
            obstacle_z_halfsizes_.push_back(hz);
            
            obstacle_corners_.emplace_back(
                std::vector<std::vector<double>>{
                std::vector<double> {x - hx, y - hy, z - hz},
                std::vector<double> {x - hx, y + hy, z - hz},
                std::vector<double> {x + hx, y - hy, z - hz},
                std::vector<double> {x + hx, y + hy, z - hz},
                std::vector<double> {x - hx, y - hy, z + hz},
                std::vector<double> {x - hx, y + hy, z + hz},
                std::vector<double> {x + hx, y - hy, z + hz},
                std::vector<double> {x + hx, y + hy, z + hz}
            });

            obstacle_centers_3d_.emplace_back(std::vector<double>{x, y, z});

        }

        ground_centers_kd_ = KDTree{std::move(obstacle_ground_centers_)};


        double xmin =  std::numeric_limits<double>::infinity();
        double ymin =  std::numeric_limits<double>::infinity();
        double zmin =  std::numeric_limits<double>::infinity();
        double xmax = -std::numeric_limits<double>::infinity();
        double ymax = -std::numeric_limits<double>::infinity();
        double zmax = -std::numeric_limits<double>::infinity();

        for (const auto& corner : obstacle_corners_){
            for (const auto& point : corner){
                xmin = std::min(xmin, point[0]);
                ymin = std::min(ymin, point[1]);
                zmin = std::min(zmin, point[2]);
                xmax = std::max(xmax, point[0]);
                ymax = std::max(ymax, point[1]);
                zmax = std::max(zmax, point[2]);
            }
        }
        x_lim_ = {xmin, xmax};
        y_lim_ = {ymin, ymax};
        z_lim_ = {zmin, zmax};
    }

    bool ObstacleProcessor::is_collision(const std::vector<double>& query_point) const{

        // All three coordinates must be smaller than all maxes but larger than all mins.

        // Find nearest ground center obstacle
        // Assume: It will always have a nearest obstacle
        const std::vector<double> ground_center{query_point.at(0), query_point.at(1)};
        std::size_t nearest_idx{ground_centers_kd_.nearest_index(ground_center)};
        
        const std::vector<double>& center{obstacle_centers_3d_.at(nearest_idx)}; 
        const std::vector<double>& xy_halfsize{obstacle_ground_center_halfsizes_.at(nearest_idx)};
        
        const double center_x{center.at(0)};
        const double center_y{center.at(1)};
        const double center_z{center.at(2)};

        const double hx{xy_halfsize.at(0)};
        const double hy{xy_halfsize.at(1)};
        const double hz{obstacle_z_halfsizes_.at(nearest_idx)};
        
        const double x{query_point.at(0)};
        const double y{query_point.at(1)};
        const double z{query_point.at(2)};

        return (
            std::abs(x - center_x) <= hx &&
            std::abs(y - center_y) <= hy &&
            std::abs(z - center_z) <= hz
        );

    }

    double ObstacleProcessor::distance_from_obstacle(const std::vector<double>& query_point) const{
        const std::vector<double> ground_center{query_point.at(0), query_point.at(1)};
        
        // If the query point is in collision with an obstacle, then its distance to the nearest
        // obstacle is 0.0. The function returns this 0.0.
        if (is_collision(query_point)){
            return 0.0;
        }

        // If the query point is NOT in a collision with any obstacle, then return its Euclidean 
        // distance to the obtacle. 

        // Extract the nearest obstacle's geometry data
        std::size_t nearest_idx{ground_centers_kd_.nearest_index(ground_center)};
        const std::vector<double>& center{obstacle_centers_3d_.at(nearest_idx)}; 
        const std::vector<double>& xy_halfsize{obstacle_ground_center_halfsizes_.at(nearest_idx)};
        
        // Get the obstacle's center position
        const double center_x{center.at(0)};
        const double center_y{center.at(1)};
        const double center_z{center.at(2)};

        // Get the obstacle's halfsizes
        const double hx{xy_halfsize.at(0)};
        const double hy{xy_halfsize.at(1)};
        const double hz{obstacle_z_halfsizes_.at(nearest_idx)};
        
        // Extract the query point's coordinates
        const double x{query_point.at(0)};
        const double y{query_point.at(1)};
        const double z{query_point.at(2)};

        // Compute the components of the distance vector (dx, dy, dz)
        double dx{0.0};
        if (x < center_x - hx){
            dx = center_x - x - hx;
        } else if (x > center_x + hx){
            dx = x - center_x - hx;
        }

        double dy{0.0};
        if (y < center_y - hy){
            dy = center_y - y - hy;
        } else if (y > center_y + hy){
            dy = y - center_y - hy;
        }

        double dz{0.0};
        if (z < center_z - hz){
            dz = center_z - z - hz;
        } else if (z > center_z + hz){
            dz = z - center_z - hz;
        }

        // Return the Euclidean norm of the distance vector
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

} // obstacle_processor