#include <iostream>
#include <fstream>
#include <random>
#include "ObstacleData.h"
#include "GlobalPlanning.h"
#include "LocalPlanning.h"
#include <chrono>


void testReadObstacleData(ObstacleData& obstacleData){
    
    // Verify that the readObstacleData method extracts the correct bounds 
    // of the obstacle data set. 


    obstacleData.readObstacleFile("obstacle-data.csv");

    Point2D xLim = obstacleData.xLim;
    Point2D yLim = obstacleData.yLim;
    Point2D zLim = obstacleData.zLim;
    
    if (std::abs(xLim[0] - (-315.239) ) <= 0.01){
        std::cout << "Test pass: Xmin OK" << std::endl;
    }
    if (std::abs(yLim[0] - (-444.232) ) <= 0.01){
        std::cout << "Test pass: Ymin OK" << std::endl;
    }
    if (std::abs(zLim[0] - (-0.396395) ) <= 0.01){
        std::cout << "Test pass: Zmin OK" << std::endl;
    }

    if (std::abs(xLim[1] - 604.761 ) <= 0.01){
        std::cout << "Test pass: Xmax OK" << std::endl;
    }
    if (std::abs(yLim[1] - 475.768 ) <= 0.01){
        std::cout << "Test pass: Ymax OK" << std::endl;
    }
    if (std::abs(zLim[1] - 212.0 ) <= 0.01){
        std::cout << "Test pass: Zmax OK" << std::endl;
    }

    // Inspect the bounds of the environment
    std::cout << "Bounds for inspection: " << std::endl;
    std::cout << "xLim: [" << xLim[0] << ", " << xLim[1] << "]" << std::endl;
    std::cout << "yLim: [" << yLim[0] << ", " << yLim[1] << "]" << std::endl;
    std::cout << "zLim: [" << zLim[0] << ", " << zLim[1] << "]" << std::endl;
}

void testIsCollisionDetected(ObstacleData& obstacleData){


    // Verify that the isCollisionDetected method returns the expected result
    // for a sequence of hard-coded test cases. 

    Point3D p1 = {50.0f, 50.0f, 20.0f}; // Expected: False
    Point3D p2 = {51.0f, 50.0f, 21.0f}; // Expected: False
    Point3D p3 = {50.0f, 90.0f, 19.0f}; // Expected: True

    if (!obstacleData.isCollisionDetected(p1)){
        std::cout << "Test pass" << std::endl;

    }

    if (!obstacleData.isCollisionDetected(p2)){
        std::cout << "Test pass" << std::endl;

    }

    if (obstacleData.isCollisionDetected(p3)){
        std::cout << "Test pass" << std::endl;
    }
}


void testDistanceFromObstacle(ObstacleData& obstacleData){


    // Verify the distanceFromObstacle method returns the expected result
    // for a sequence of hard-coded test cases.

    Point3D p1 = {50.0f, 90.0f, 19.0f}; // Expected: 0.0 (Inside obstacle bounding box)
    Point3D p2 = {51.0f, 93.0f, 21.0f}; // Expected: 0.0 (Inside obstacle bounding box)
    Point3D p3 = {52.0f, 95.0f, 23.0f}; // Expected: 0.0 (Inside obstacle bounding box)

    if (obstacleData.distanceFromObstacle(p1) == 0.0){
        std::cout << "Test pass" << std::endl;
    }

    if (obstacleData.distanceFromObstacle(p2) == 0.0){
        std::cout << "Test pass" << std::endl;
    }

    if (obstacleData.distanceFromObstacle(p3) == 0.0){
        std::cout << "Test pass" << std::endl;
    }

}

void saveGraphToFile(const FreeSpaceGraph& graph) {
    std::ofstream file("graph.json");
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing graph data." << std::endl;
        return;
    }
    file << "{\n";
    for (const auto& [point, neighbors] : graph.getAdjacencyList()) {
        file << "  \"(" << point[0] << ", " << point[1] << ", " << point[2] << ")\": [";
        for (const auto& [neighbor, _] : neighbors) {
            file << "( " << neighbor[0] << ", " << neighbor[1] << ", " << neighbor[2] << " ), ";
        }
        file << "],\n";
    }
    file << "}";
    file.close();
    std::cout << "Graph saved to graph.json\n";
}

void testGenerateGraph(FreeSpaceGraph& graph, ObstacleData& obstacles){
    
    float resolution = 25.0f;
    graph.generateGraph(obstacles, resolution);
    saveGraphToFile(graph);

}


void testAStarPerformance(FreeSpaceGraph& graph, ObstacleData& obstacles) {
    // Objective: Estimate the typical execution time for A* search 
    // for waypoints spaced between 600 m and 700 m apart.

    // Generate random uniform distributions 
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> xDist(obstacles.xLim[0], obstacles.xLim[1]);
    std::uniform_real_distribution<float> yDist(obstacles.yLim[0], obstacles.yLim[1]);
    std::uniform_real_distribution<float> zDist(obstacles.zLim[0], obstacles.zLim[1]);
    
    int validCount = 0;
    double totalTime = 0.0;
    const int numTests = 50;
    const float minSeparation = 600.0f;
    const float maxSeparation = 700.0f;
    // Precompute squared distances to avoid computing sqrt repeatedly.
    const float minSeparationSq = minSeparation * minSeparation;
    const float maxSeparationSq = maxSeparation * maxSeparation;

    while (validCount < numTests) {
        // Generate a valid start point (one that is not in collision)
        Point3D start;
        while (true) {
            start = { xDist(gen), yDist(gen), zDist(gen) };
            if (!obstacles.isCollisionDetected(start)) {
                break; // Valid start found.
            }
        }

        // Generate a valid goal point:
        // The goal must not be in collision and the squared distance from start 
        // must be between minSeparationSq and maxSeparationSq.
        Point3D goal;
        float dx, dy, dz, distanceSq;
        while (true) {
            goal = { xDist(gen), yDist(gen), zDist(gen) };
            if (obstacles.isCollisionDetected(goal)) {
                continue; // Skip if the goal is in collision.
            }
            dx = goal[0] - start[0];
            dy = goal[1] - start[1];
            dz = goal[2] - start[2];
            distanceSq = dx * dx + dy * dy + dz * dz;
            if (distanceSq >= minSeparationSq && distanceSq <= maxSeparationSq) {
                break; // Valid goal found.
            }
        }
        
        // Time the A* search
        auto startTime = std::chrono::high_resolution_clock::now();
        graph.searchGraph(start, goal);
        auto endTime = std::chrono::high_resolution_clock::now();
        totalTime += std::chrono::duration<double>(endTime - startTime).count();
        validCount++;
    }
    std::cout << "Average A* search time over " << numTests 
              << " cases: " << (totalTime / numTests) << " seconds" << std::endl;
}

void testRRT(ObstacleData& obstacleData) {
    // For testing, choose start and goal positions that lie within free space.
    // You may need to adjust these values based on your obstacle data.
    Point3D start = {0.0f, 0.0f, 10.0f};
    Point3D goal  = {0.0f, 50.0f, 200.0f};

    // Create an RRT instance. Adjust step size, iterations, and goal bias as desired.
    RRT rrt(start, goal, obstacleData, 1.0f, 10000, 0.25f);

    // Run the planner.
    Points3D path = rrt.plan();

    // Check if a path was found.
    if (path.empty()) {
        std::cout << "RRT failed to find a path." << std::endl;
        return;
    } else {
        std::cout << "RRT found a path with " << path.size() << " nodes." << std::endl;
    }

    // Export the computed path to a JSON file.
    std::ofstream file("rrt_path.json");
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing RRT path data." << std::endl;
        return;
    }
    
    file << "[\n";
    for (size_t i = 0; i < path.size(); ++i) {
        file << "  { \"x\": " << path[i][0]
             << ", \"y\": " << path[i][1]
             << ", \"z\": " << path[i][2] << " }";
        if (i < path.size() - 1) {
            file << ",";
        }
        file << "\n";
    }
    file << "]\n";
    file.close();
    std::cout << "RRT path saved to rrt_path.json" << std::endl;
}


int main() {



    std::cout << "Validating the Obstacle processing module..." << std::endl;
    ObstacleData obstacleData;

    std::cout << "Validating the readObstacleData method of the ObstacleData class..." << std::endl;    
    testReadObstacleData(obstacleData); 

    std::cout << "Validating the isCollisionDetected method of the ObstacleData class... " << std::endl; 
    testIsCollisionDetected(obstacleData);

    std::cout << "Validating the distanceFromObstacle method of the ObstacleData class..." << std::endl;
    //testDistanceFromObstacle(obstacleData);

    std::cout << "Validating the global planning module..." << std::endl;
    //FreeSpaceGraph graph;
    
    std::cout << "Validating the generateGraph method of the FreeSpaceGraph class" << std::endl;
    //testGenerateGraph(graph, obstacleData);

    std::cout << "Validating the searchGraph method of the FreeSpace graph class" << std::endl;
    //testAStarPerformance(graph, obstacleData);

    // Test the RRT implementation
    std::cout << "Testing RRT planning..." << std::endl;
    testRRT(obstacleData);




    return 0;
}


