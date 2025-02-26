#include <iostream>
#include <fstream>
#include <random>
#include "ObstacleData.h"
#include "GlobalPlanning.h"
#include "LocalPlanning.h"
#include <chrono>
#include <Eigen/Dense>
#include "TrajectoryPlanning.h"


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
    Point3D start = {0.0f,0.0f,0.0f};
    Point3D goal  = {-400.0f, 400.0f, 100.0f};

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

    // Post-process the path.
    Points3D shortcutPath = rrt.shortcutPath(path);
    Points3D enhancedPath = rrt.enhancedShortenedPath(shortcutPath);

    // Verify that all points in enhancedPath are collision free,
    // and print the distance from the obstacle for any collisions.
    bool allCollisionFree = true;
    for (const auto& pt : enhancedPath) {
        if (obstacleData.isCollisionDetected(pt)) {
            float distance = obstacleData.distanceFromObstacle(pt);
            std::cerr << "Collision detected at point: ("
                      << pt[0] << ", " << pt[1] << ", " << pt[2]
                      << ") with distance " << distance << " from the obstacle." << std::endl;
            allCollisionFree = false;
        }
    }
    if (allCollisionFree) {
        std::cout << "All points in enhancedPath are collision free." << std::endl;
    } else {
        std::cerr << "Some points in enhancedPath are in collision." << std::endl;
    }

    // Export the computed original RRT path to a JSON file.
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



// Monte Carlo test for RRT: selects random start/goal pairs that are 45-50 m apart,
// tracks running statistics, and reports when the running mean converges.
void testRRTMonteCarlo(ObstacleData& obstacleData) {
    // Create random distributions for x, y, and z based on the environment bounds.
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> xDist(obstacleData.xLim[0], obstacleData.xLim[1]);
    std::uniform_real_distribution<float> yDist(obstacleData.yLim[0], obstacleData.yLim[1]);
    std::uniform_real_distribution<float> zDist(obstacleData.zLim[0], obstacleData.zLim[1]);

    const int numTests = 1000; // maximum number of Monte Carlo iterations
    int validCount = 0;
    std::vector<double> times; // store execution times for valid tests
    const float minSep = 45.0f, maxSep = 50.0f;
    float minSepSq = minSep * minSep;
    float maxSepSq = maxSep * maxSep;

    // Convergence parameters:
    const int minTrialsBeforeConvergence = 100;
    const int consecutiveRequired = 10; // number of consecutive trials with little change
    int consecutiveCount = 0;
    const double toleranceFraction = 0.01; // 1% of the running mean
    double previousRunningMean = 0.0;

    while (validCount < numTests) {
        // Generate a collision-free start point.
        Point3D start;
        while (true) {
            start = { xDist(gen), yDist(gen), zDist(gen) };
            if (!obstacleData.isCollisionDetected(start))
                break;
        }

        // Generate a collision-free goal point that is between 45 and 50 m from start.
        Point3D goal;
        while (true) {
            goal = { xDist(gen), yDist(gen), zDist(gen) };
            if (obstacleData.isCollisionDetected(goal))
                continue; // Skip if in collision.
            float dx = goal[0] - start[0];
            float dy = goal[1] - start[1];
            float dz = goal[2] - start[2];
            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq >= minSepSq && distSq <= maxSepSq)
                break;
        }

        // Create an RRT instance with the generated start and goal.
        // Adjust RRT parameters as needed.
        RRT rrt(start, goal, obstacleData, 5.0f, 1000, 0.33f);
        auto t0 = std::chrono::high_resolution_clock::now();
        Points3D path = rrt.plan();
        auto t1 = std::chrono::high_resolution_clock::now();

        // Only count the test if a valid path was found.
        if (path.empty()) {
            std::cout << "Test " << validCount 
                      << " failed: no path found between start and goal." << std::endl;
            continue;
        }
        double timeSec = std::chrono::duration<double>(t1 - t0).count();
        times.push_back(timeSec);
        validCount++;
        std::cout << "Test " << validCount << ": path found in " << timeSec 
                  << " seconds with " << path.size() << " nodes." << std::endl;

        // Update the running mean.
        double sum = std::accumulate(times.begin(), times.end(), 0.0);
        double runningMean = sum / times.size();

        // Check for convergence only after a minimum number of trials.
        if (validCount >= minTrialsBeforeConvergence) {
            double tolerance = toleranceFraction * runningMean;
            double diff = std::abs(runningMean - previousRunningMean);
            if (diff < tolerance) {
                consecutiveCount++;
            } else {
                consecutiveCount = 0;
            }
            previousRunningMean = runningMean;

            if (consecutiveCount >= consecutiveRequired) {
                std::cout << "Convergence reached after " << validCount 
                          << " valid tests. Running mean planning time = " << runningMean 
                          << " seconds." << std::endl;
                break; // Optionally break out early if convergence is reached.
            }
        }
    }

    // Compute overall statistics from the collected times.
    double sum = std::accumulate(times.begin(), times.end(), 0.0);
    double mean = sum / times.size();

    double variance = 0.0;
    for (double t : times) {
        variance += (t - mean) * (t - mean);
    }
    variance /= times.size();
    double stddev = std::sqrt(variance);

    std::cout << "Final average RRT planning time over " << validCount 
              << " tests: " << mean << " seconds." << std::endl;
    std::cout << "Variance: " << variance << " sec^2, StdDev: " << stddev << " seconds." << std::endl;
}




void testTrajectoryPlanner() {
    std::cout << "Running trajectory planner test..." << std::endl;
    
    // Define a set of test waypoints (same as your Python example)
    Points3D testWaypoints = {
        { -100.0f, -50.0f, 200.0f },
        { -95.95f, -49.01f, 190.91f },
        { -91.90f, -48.02f, 181.82f },
        { -87.85f, -47.02f, 172.73f },
        { -83.80f, -46.03f, 163.64f },
        { -75.75f, -44.06f, 145.55f },
        { -71.29f, -41.46f, 136.98f },
        { -66.83f, -38.87f, 128.41f },
        { -62.37f, -36.28f, 119.85f },
        { -57.91f, -33.68f, 111.28f },
        { -53.45f, -31.09f, 102.71f },
        { -48.99f, -28.50f, 94.14f },
        { -44.54f, -25.90f, 85.58f },
        { -40.08f, -23.31f, 77.01f },
        { -35.62f, -20.72f, 68.44f },
        { -31.16f, -18.12f, 59.88f },
        { -26.70f, -15.53f, 51.31f },
        { -22.24f, -12.94f, 42.74f },
        { -17.78f, -10.34f, 34.17f },
        { -13.33f, -7.75f, 25.61f },
        { -8.87f, -5.16f, 17.04f },
        { 0.0f, 0.0f, 0.0f }
    };
    
    double maxSpeed = 5.0;
    TrajectoryPlanner planner(testWaypoints, maxSpeed);
    TrajectorySolution sol = planner.solveTrajectory();

    std::cout << "Trajectory Planner Coefficients:" << std::endl;
    std::cout << "X coefficients:\n" << sol.xCoeffs << std::endl << std::endl;
    std::cout << "Y coefficients:\n" << sol.yCoeffs << std::endl << std::endl;
    std::cout << "Z coefficients:\n" << sol.zCoeffs << std::endl << std::endl;
    
}

int main() {



    std::cout << "Validating the Obstacle processing module..." << std::endl;
    ObstacleData obstacleData;

    std::cout << "Validating the readObstacleData method of the ObstacleData class..." << std::endl;    
    testReadObstacleData(obstacleData); 

    std::cout << "Validating the isCollisionDetected method of the ObstacleData class... " << std::endl; 
    testIsCollisionDetected(obstacleData);

    std::cout << "Validating the distanceFromObstacle method of the ObstacleData class..." << std::endl;
    testDistanceFromObstacle(obstacleData);

    std::cout << "Validating the global planning module..." << std::endl;
    FreeSpaceGraph graph;
    
    std::cout << "Validating the generateGraph method of the FreeSpaceGraph class" << std::endl;
    testGenerateGraph(graph, obstacleData);

    std::cout << "Validating the searchGraph method of the FreeSpace graph class" << std::endl;
    testAStarPerformance(graph, obstacleData);

    // Test the RRT implementation
    std::cout << "Testing RRT planning..." << std::endl;
    testRRT(obstacleData);

    // Run the Monte Carlo RRT test.
    std::cout << "Running Monte Carlo RRT tests..." << std::endl;
    testRRTMonteCarlo(obstacleData);

    // Run the trajectory planner and inspect the generated polynomial coefficients 
    std::cout << "Running trajectory planner test..." << std::endl;
    testTrajectoryPlanner();

    // 



    return 0;
}


