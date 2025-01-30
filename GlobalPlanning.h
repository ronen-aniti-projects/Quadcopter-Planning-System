#pragma once 

#include "include/KDTree.hpp"
#include "ObstacleData.h"
#include "Types.h"


class FreeSpaceGraph{
public:

    // Constructor
    FreeSpaceGraph();

    // Destructor
    ~FreeSpaceGraph();

    // Methods
    void generateGraph(const ObstacleData& obstacles, float resolution=25.0f);
    Points3D searchGraph(const Point3D& start, const Point3D& goal) const;
    
    // Getters
    const Points3D& getPoints() const {return points;}
    const AdjacencyList& getAdjacencyList() const {return adjacencyList;}

private:
    Points3D points;
    AdjacencyList adjacencyList;
    std::unique_ptr<KDTree> kdTree;

    // Helper functions
    point_t toKDPoint(const Point3D& p) const;
    Point3D fromKDPoint(const point_t& p) const;
    bool validateEdge(const ObstacleData& obs, 
                      const Point3D& p1, 
                      const Point3D& p2, 
                      float step = 5.0f) const; 





};
