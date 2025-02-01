#pragma once

#include "ObstacleData.h"
#include "Types.h"
#include <unordered_map>


class RRT{

public:
    RRT(Point3D start, Point3D goal, ObstacleData& obstacleData, 
        float stepSize=1.0f, int maxIterations=1000, float goalBias=0.1f);

    Points3D plan();




private:
    Point3D randomSample();
    Point3D findNearest(const Point3D& sample);
    Point3D moveTowards(const Point3D& from, const Point3D& to);
    
    bool isPathClear(const Point3D& start, const Point3D& end);

    std::unordered_map<Point3D, Point3D, std::hash<Point3D>> tree_;

    Point3D start_;
    Point3D goal_;
    ObstacleData& obstacleData_;
    float stepSize_;
    int maxIterations_; 
    float goalBias_; 

};