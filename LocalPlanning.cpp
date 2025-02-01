#include "LocalPlanning.h"
#include "Types.h"
#include "ObstacleData.h"
#include "KDTree.hpp"
#include <random>
#include <cmath>

RRT::RRT(Point3D start, Point3D goal, ObstacleData& obstacleData, float stepSize, int maxIterations, float goalBias) :
    start_(start), goal_(goal), obstacleData_(obstacleData),
    stepSize_(stepSize), maxIterations_(maxIterations), goalBias_(goalBias) {
        tree_[start] = {0,0,0};
    } 

Point3D RRT::randomSample(){
    
    // Generate random floats belonging to [0, 1]
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    
    // Sample the goal when the generated float belongs to [0, 1)
    if (dist(gen) < goalBias_){
        return goal_;
    }

    // Return a random point belonging inside the 3D map
    return {
        obstacleData_.xLim[0] + dist(gen) * (obstacleData_.xLim[1] - obstacleData_.xLim[0]),
        obstacleData_.yLim[0] + dist(gen) * (obstacleData_.yLim[1] - obstacleData_.yLim[0]),
        obstacleData_.zLim[0] + dist(gen) * (obstacleData_.zLim[1] - obstacleData_.zLim[0])
    };
}

Point3D RRT::findNearest(const Point3D& sample){

    // Convert sample from Point3D to KDTree format, vector<double>
    std::vector<double> kdSample = {
        static_cast<double>(sample[0]),
        static_cast<double>(sample[1]),
        static_cast<double>(sample[2])
    };

    // Build a KDTree from all graph points
    std::vector<std::vector<double>> points;
    for (const auto& point: tree_){
        points.push_back({
            static_cast<double>(point.first[0]),
            static_cast<double>(point.first[1]),
            static_cast<double>(point.first[2])
        });
    }
    KDTree kdTree(points);

    // Find the nearest point
    size_t nearestIndex = kdTree.nearest_index(kdSample);
    return {
        static_cast<float>(points[nearestIndex][0]),
        static_cast<float>(points[nearestIndex][1]),
        static_cast<float>(points[nearestIndex][2])
    };
}

// Move from the nearest node towards the sample
Point3D RRT::moveTowards(const Point3D& from, const Point3D& to){
    Point3D direction;
    float distance = 0.0f;

    // Calculate distance between points
    for (int i=0; i < 3; i++){
        direction[i] = to[i] - from[i];
        distance += direction[i] * direction[i];

    }
    distance = sqrt(distance);

    // If target is close, return target
    if (distance < stepSize_){
        return to;
    }

    // Otherwise move stepSize_ towards target
    Point3D newPoint;
    for (int i = 0; i < 3; i++){
        newPoint[i] = from[i] + (direction[i]/distance) * stepSize * i;
    }
    return newPoint;
}

bool RRT::isPathClear(const Point3D& start, const Point3D& end){
    Point3D direction;
    float distance = 0.0f;
    
}

