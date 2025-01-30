#include "LocalPlanning.h"
#include "Types.h"
#include "ObstacleData.h"
#include "KDTree.hpp"
#include <random>
#include <cmath>

RRT::RRT(Point3D start, Point3D goal, ObstacleData& obstacleData, float stepSize, int maxIterations) :
    start(start), goal(goal), obstacleData(obstacleData),
    stepSize(stepSize), maxIterations(maxIterations){
        tree[start] = {0,0,0};
    } 

Point3D RRT::randomSample(){}

Point3D RRT::findNearest(const Point3D& sample){}

void RRT::printPath(const Points3D& path){}