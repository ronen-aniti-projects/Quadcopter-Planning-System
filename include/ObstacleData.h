#ifndef OBSTACLE_DATA_H
#define OBSTACLE_DATA_H

#include <array>
#include <string>
#include <vector>
#include <utility>
#include "include/KDTree.hpp"
#include "Types.h"

class ObstacleData {
public:
    // Constructor
    ObstacleData();

    // Destructor
    ~ObstacleData();

    // Methods
    void readObstacleFile(const std::string& filename); 
    size_t queryGroundCenter(const Point2D& queryPoint) const;
    bool isCollisionDetected(const Point3D& queryPoint) const; 
    float distanceFromObstacle(const Point3D& queryPoint) const; 

    // Properties
    Points2D obstacleGroundCenters; // 2D centers of obstacles
    Points2D obstacleGroundCenterHalfsizes;   // Half-sizes in 2D
    std::vector<float> obstacleHeights; // Heights of obstacles
    std::vector<float> obstacleZHalfsizes;  // Half-sizes in Z
    Corners obstacleCorners;    // 3D corners of obstacles
    Points3D obstacleCenters3d;    // 3D centers of obstacles
    Point2D  xLim; // Limits in x-axis
    Point2D  yLim; // Limits in y-axis
    Point2D  zLim; // Limits in z-axis

    // KDTree
    std::unique_ptr<KDTree> kdTree;

};

#endif // OBSTACLE_DATA_H
