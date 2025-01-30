#pragma once

#include "ObstacleData.h"
#include "Types.h"
#include <unordered_map>


class RRT{

public:
    RRT(Point3D start, Point3D goal, ObstacleData& obstacleData, 
        float stepSize=1.0f, int maxIterations=1000);

    Points3D plan();
    void printPath(const Points3D& path); // Debug helper




private:
    Point3D randomSample();
    Point3D findNearest(const Point3D& sample);
    Point3D moveTowards(const Point3D& from, const Point3D& to); 
    bool isPathClear(const Point3D& start, const Point3D& end);

    std::unordered_map<Point3D, Point3D, std::hash<Point3D>> tree;

    Point3D start;
    Point3D goal;
    ObstacleData& obstacleData;
    float stepSize;
    int maxIterations; 

};