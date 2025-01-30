#include "GlobalPlanning.h"
#include <iostream>
#include <queue>

FreeSpaceGraph::FreeSpaceGraph(){}

FreeSpaceGraph::~FreeSpaceGraph(){}


point_t FreeSpaceGraph::toKDPoint(const Point3D& p) const{

return {static_cast<double>(p[0]),
        static_cast<double>(p[1]),
        static_cast<double>(p[2])};
}

Point3D FreeSpaceGraph::fromKDPoint(const point_t& p) const{

return {static_cast<float>(p[0]),
        static_cast<float>(p[1]),
        static_cast<float>(p[2])};
}

void FreeSpaceGraph::generateGraph(const ObstacleData& obstacles, float resolution=25.0f){

    points.clear();
    adjacencyList.clear();

    // Generate a grid of non-obstacle points
    for (float x = obstacles.xLim[0]; x <= obstacles.xLim[1]; x += resolution){
        for (float y = obstacles.yLim[0]; y <= obstacles.yLim[1]; y += resolution){
            for (float z = obstacles.zLim[0]; z <= obstacles.zLim[1]; z += resolution){
                Point3D p{x, y, z};
                if (!obstacles.isCollisionDetected(p)){
                    points.push_back(p);
                }
            }
        }
    }
    

    // Build KDTree for efficient queries (edge validation)
    pointVec kdPoints;
    for (const auto& p : points){
        kdPoints.push_back(toKDPoint(p));
    }
    kdTree = std::make_unique<KDTree>(kdPoints);

    // Build adjacency list
    const float connectionRadius = resolution * 1.1f;
    for (const auto& p : points){
        std::vector<std::pair<Point3D, float>> neighbors; // point -> vec of [point, dist]
        point_t query = toKDPoint(p);
        auto neighborIndices = kdTree->neighborhood_indices(query, connectionRadius);

        for (size_t idx : neighborIndices){
            Point3D neighbor = points[idx];
            if (p != neighbor && validateEdge(obstacles, p, neighbor)){
                float dx = p[0] - neighbor[0];
                float dy = p[1] - neighbor[1];
                float dz = p[2] - neighbor[2];
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                neighbors.emplace_back(neighbor, dist);        
            }
        }
        adjacencyList[p] = neighbors;
    }
}


Points3D FreeSpaceGraph::searchGraph(const Point3D& start, const Point3D& goal) const{
    if(points.empty()) {
        throw std::runtime_error("Graph not generated. Call generateGraph() first.");
    }

    // Find nearest nodes to the query start/goal
    size_t startIdx = kdTree->nearest_index(toKDPoint(start));
    size_t goalIdx = kdTree->nearest_index(toKDPoint(goal));
    Point3D startNode = points[startIdx];
    Point3D goalNode = points[goalIdx];

    // A* data structures
    using Node = std::pair<float, Point3D>; // (f-score, point)
    std::priority_queue<Node, std::vector<Node>, std::greater<>> openSet;
    std::unordered_map<Point3D, Point3D> cameFrom;
    std::unordered_map<Point3D, float> gScore;

    // Reminder: Node gScore is cost to traverse there from start
    //           Node fScore is that traversal cost plus a best-guess
    //            of the additional cost to reach the goal.
    //           Nodes with lower recorded fScore are explored
    //            first via the priority queue structure. 
    //           For a node to be explored, it must be in the openSet.
    //           For a node to be in the openSet, it must have been current or
    //            neighbor to a current at one point during the A* loop.

    // Initialize scores
    for (const auto& p : points){
        gScore[p] = std::numeric_limits<float>::max();
    }
    gScore[startNode] = 0.0f;

    // Heuristic function (Euclidean distance)
    auto heuristic = [&](const Point3D& a, const Point3D& b){
        float dx = a[0] - b[0];
        float dy = a[1] - b[1];
        float dz = a[2] - b[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    openSet.emplace(heuristic(startNode, goalNode), startNode);

    // A* main loop
    while (!openSet.empty()){
        auto [currentF, current] = openSet.top();
        openSet.pop();

        if (current == goalNode){
            // Reconstruct path
            Points3D path;
            Point3D node = goalNode;
            while(cameFrom.find(node) != cameFrom.end()){
                path.push_back(node);
                node = cameFrom[node];

            }
            path.push_back(startNode);
            std::reverse(path.begin(), path.end());
            return path;

        }

        // Look through neighbors to current
        // Reminder: The adjacency list has the current->neighbor cost recorded
        for (const auto& [neighbor, cost] : adjacencyList.at(current)){
            // Add the current->neighbor cost to the path cost to compute g
            float tentativeG = gScore[current] + cost;
            // Only proceed if this sum is less than the neighbor's existing g
            if (tentativeG < gScore[neighbor]){
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;
                // f is 
                float fScore = tentativeG + heuristic(neighbor, goalNode);
                openSet.emplace(fScore, neighbor);
            }
        }
    }
    return {}; // No path found



    
    
}

bool FreeSpaceGraph::validateEdge(const ObstacleData& obs, 
                      const Point3D& p1, 
                      const Point3D& p2, 
                      float step) const {
    
    Point3D displacement = {p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]};
    float distance = std::sqrt(displacement[0]*displacement[0] + 
                           displacement[1]*displacement[1] + 
                           displacement[2]*displacement[2]);
    Point3D unitVector = {displacement[0]/distance, 
                          displacement[1]/distance,
                          displacement[2]/distance};
    
    int steps = distance / step;
    // Assume p1 and p2 are collision free. Only check intermediate points.
    for (int i=1; i <= steps; ++i){
        Point3D checkPoint = {
            p1[0] + unitVector[0] * step * i,
            p1[1] + unitVector[1] * step * i,
            p1[2] + unitVector[2] * step * i 
        };
        if (obs.isCollisionDetected(checkPoint)){
            return false;
        }
    }
    return true;



}

