#include "LocalPlanning.h"
#include "Types.h"
#include "ObstacleData.h"
#include "KDTree.hpp"
#include <random>
#include <cmath>
#include <iostream>

RRT::RRT(Point3D start, Point3D goal, ObstacleData& obstacleData, float stepSize, int maxIterations, float goalBias) :
    start_(start), goal_(goal), obstacleData_(obstacleData),
    stepSize_(stepSize), maxIterations_(maxIterations), goalBias_(goalBias) {
        tree_[start] = start;
    } 

bool RRT::closeEnough(const Point3D& point1, const Point3D& point2){
    float epsilon = 1e-6f;
    if (std::fabs(point1[0] - point2[0]) < epsilon && std::fabs(point1[1] - point2[1]) < epsilon && std::fabs(point1[2] - point2[2]) < epsilon){
        return true;
    }
    return false;

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
        newPoint[i] = from[i] + (direction[i]/distance) * stepSize_;
    }
    return newPoint;
}

bool RRT::isPathClear(const Point3D& start, const Point3D& end){
    Point3D direction;
    float distance = 0.0f;
    
    // Calculate total distance
    for (int i = 0; i < 3; i++){
        direction[i] = end[i] - start[i];
        distance += direction[i] * direction[i];
    }
    distance = sqrt(distance);

    // Check intermediate points 
    int steps = static_cast<int> (distance / stepSize_);
    for (int i = 1; i <= steps; i++){
        Point3D checkPoint; 
        for (int j = 0; j < 3; j++){
            checkPoint[j] = start[j] + (direction[j] / distance) * stepSize_ * i;
        }
        if (obstacleData_.isCollisionDetected(checkPoint)){
            return false;
        }

    }
    return true;
    
}

std::vector<Point3D> RRT::plan(){ 
    for (int i = 0; i < maxIterations_; i++){

        // 1. Sample a random point
        Point3D sample = randomSample();

        // 2. Find nearest existing node
        Point3D nearest = findNearest(sample);

        // 3. Move towards sample by one step
        Point3D newNode = moveTowards(nearest, sample);

        // 4. Check if this path is collision-free
        if (isPathClear(nearest, newNode)){
            // Add newNode to the tree
            tree_[newNode] = nearest;

            // Terminate planning when newNode is close enough to goal
            float distToGoal = 0.0f;
            for (int j = 0; j < 3; j++){
                distToGoal += (newNode[j] - goal_[j]) * (newNode[j] - goal_[j]);
            }
            distToGoal = sqrt(distToGoal);
            if (distToGoal < stepSize_){
                tree_[goal_] = newNode;

                // Reconstruct the path
                Points3D path;
                Point3D current = goal_;
                while (true){
                    path.push_back(current);
                    current = tree_[current];
                    if (closeEnough(current, start_)){
                        path.push_back(start_);
                        std::reverse(path.begin(), path.end());  
                        return path;
                    }
                }
            }
        }
    
    }
           return {};
 
}

Points3D RRT::shortcutPath(const Points3D& inputPath){
    if (inputPath.empty()){
        return {};

    }
    int endIndex = inputPath.size() - 1;
    int startIndex = 0;
    int currentEndIndex = endIndex;
    int currentStartIndex = startIndex; 
    std::vector<int> shortcutIndices;
    shortcutIndices.push_back(endIndex);

    while(currentStartIndex < currentEndIndex){
        bool shortcutFound = false;
        while (currentStartIndex < currentEndIndex){
            if (isPathClear(inputPath[currentStartIndex], inputPath[currentEndIndex])){
                shortcutIndices.push_back(currentStartIndex);
                shortcutFound = true;
                break;
            } else {
                currentStartIndex++; 
            }
        }
        if (shortcutFound){
            currentEndIndex = shortcutIndices.back();
            currentStartIndex = 0;
        } else { 
            currentEndIndex--;
            shortcutIndices.push_back(currentEndIndex);
            currentStartIndex = 0;
        }
    }
    std::reverse(shortcutIndices.begin(), shortcutIndices.end());
    Points3D shorterPath;
    for (int idx : shortcutIndices){
        shorterPath.push_back(inputPath[idx]);
    }
    return shorterPath;


}

Points3D RRT::enhancedShortenedPath(const Points3D& shortcutPath, float res){
    Points3D enhancedPath; 
    
    for (size_t i = 0; i < shortcutPath.size() - 1; i++){
        const Point3D& start = shortcutPath[i];
        const Point3D& end = shortcutPath[i+1];

        Point3D direction; 
        float distance = 0.0f;
        for (int j = 0; j < 3; j++){
            direction[j] = end[j] - start[j];
            distance += direction[j] * direction[j];

        }
        distance = std::sqrt(distance);

        Point3D directionUnit;
        for (int j = 0; j < 3; j++){
            directionUnit[j] = direction[j] / distance;
        }

        int numPoints = static_cast<int>(distance / res);
        for (int j = 0; j < numPoints; j++){
            Point3D intermediate;
            for (int k=0; k<3; k++){
                intermediate[k] = start[k] + directionUnit[k] * j * res;
            }
            enhancedPath.push_back(intermediate);
        }
    }
    // Append the final waypoint
    enhancedPath.push_back(shortcutPath.back());
    return enhancedPath;
}