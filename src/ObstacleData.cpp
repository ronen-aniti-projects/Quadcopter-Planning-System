#include "Types.h"
#include "ObstacleData.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <iostream>

ObstacleData::ObstacleData() {
    // Constructor definition

}

ObstacleData::~ObstacleData() {
}

void ObstacleData::readObstacleFile(const std::string& filename) {
    
    // 1. Open the obstacle data file
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    // 2. Extract the obstacle bounding box geometry (x,y,z,hx,hy,hz)
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream ss(line);

        float x = 0, y = 0, z = 0, hx = 0, hy = 0, hz = 0;
        char delimiter; // For explicitly reading and discarding commas

        // Parse the line with expected format: x,y,z,hx,hy,hz
        ss >> x >> delimiter >> y >> delimiter >> z >> delimiter 
           >> hx >> delimiter >> hy >> delimiter >> hz;

        // Store the parsed values into the respective vectors
        obstacleGroundCenters.push_back({x, y});
        obstacleGroundCenterHalfsizes.push_back({hx, hy});
        obstacleHeights.push_back(z + hz);
        obstacleZHalfsizes.push_back(hz);
        obstacleCenters3d.push_back({x, y, z});
        obstacleCorners.push_back({
            Point3D{x - hx, y - hy, z - hz},
            Point3D{x - hx, y + hy, z - hz},
            Point3D{x + hx, y - hy, z - hz},
            Point3D{x + hx, y + hy, z - hz},
            Point3D{x - hx, y - hy, z + hz},
            Point3D{x - hx, y + hy, z + hz},
            Point3D{x + hx, y - hy, z + hz},
            Point3D{x + hx, y + hy, z + hz}
        });
    }
    file.close();

    // 3. Build a KDTree for the obstacle ground centers for efficient queries
    //    later on.
    //    NOTE: The KDTree library requires points to be std::vector<std::vector<double>>
    //          Therefore, conversion from array float to vector double is necessary.
    pointVec points; 
    for (const auto& obs : obstacleGroundCenters){
        std::vector<double> p{static_cast<double>(obs[0]),
                              static_cast<double>(obs[1])};
        points.push_back(p);
    }
    kdTree = std::make_unique<KDTree>(points);


    // 4. Extract the bounds of the 3D environment
    float xmin = std::numeric_limits<float>::max();
    float ymin = std::numeric_limits<float>::max();
    float zmin = std::numeric_limits<float>::max();
    float xmax = std::numeric_limits<float>::lowest();
    float ymax = std::numeric_limits<float>::lowest();
    float zmax = std::numeric_limits<float>::lowest();
    for (const auto& corner : obstacleCorners){
        for(const auto& point : corner){
            xmin = std::min(xmin, point[0]);
            ymin = std::min(ymin, point[1]);
            zmin = std::min(zmin, point[2]);
            xmax = std::max(xmax, point[0]);
            ymax = std::max(ymax, point[1]);
            zmax = std::max(zmax, point[2]);
        }
    }
    xLim = {xmin, xmax};
    yLim = {ymin, ymax};
    zLim = {zmin, zmax};

}  


size_t ObstacleData::queryGroundCenter(const Point2D& groundCenterPoint) const {
    std::vector<double> queryPoint = {static_cast<double> (groundCenterPoint[0]), 
                                      static_cast<double> (groundCenterPoint[1])};

    size_t nearestNeighborIndex = kdTree->nearest_index(queryPoint);
    return nearestNeighborIndex;
}

bool ObstacleData::isCollisionDetected(const Point3D& queryPoint) const{
    size_t index = queryGroundCenter({queryPoint[0], queryPoint[1]});
    const auto& center = obstacleCenters3d[index];
    const auto& halfsize = obstacleGroundCenterHalfsizes[index];
    float zHalfsize = obstacleZHalfsizes[index]; 

    return (std::abs(queryPoint[0] - center[0]) <= halfsize[0]  &&
            std::abs(queryPoint[1] - center[1]) <= halfsize[1]  &&
            std::abs(queryPoint[2] - center[2]) <= zHalfsize);
}

float ObstacleData::distanceFromObstacle(const Point3D& queryPoint) const {

    size_t index = queryGroundCenter({queryPoint[0], queryPoint[1]});
    const auto& center = obstacleCenters3d[index];
    const auto& halfsize = obstacleGroundCenterHalfsizes[index];
    float zHalfsize = obstacleZHalfsizes[index];


    float dx = 0.0f;
    if (queryPoint[0] < center[0] - halfsize[0]) {
        dx = (center[0] - halfsize[0]) - queryPoint[0];
    } else if (queryPoint[0] > center[0] + halfsize[0]) {
        dx = queryPoint[0] - (center[0] + halfsize[0]);
    }

    float dy = 0.0f;
    if (queryPoint[1] < center[1] - halfsize[1]) {
        dy = (center[1] - halfsize[1]) - queryPoint[1];
    } else if (queryPoint[1] > center[1] + halfsize[1]) {
        dy = queryPoint[1] - (center[1] + halfsize[1]);
    }

    float dz = 0.0f;
    if (queryPoint[2] < center[2] - zHalfsize) {
        dz = (center[2] - zHalfsize) - queryPoint[2];
    } else if (queryPoint[2] > center[2] + zHalfsize) {
        dz = queryPoint[2] - (center[2] + zHalfsize);
    }

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
