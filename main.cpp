#include <iostream>
#include "ObstacleData.h"
#include "GlobalPlanning.h"

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

void testGenerateGraph(FreeSpaceGraph& graph, ObstacleData& obstacles){
    
    float resolution = 25.0f;
    graph.generateGraph(obstacles, resolution);

    // Save the graph construction to a file. 
    // TODO: Implement code to save the adjacencyList to a csv file. 
    // I can visually inspect this against the obstacles in Python/Matplotlib
    // for correctness.

}

void testSearchGraph(FreeSpaceGraph& graph, ObstacleData& obstacles){

    // Generate 100 random VALID start/goal configs. 
    // Generate 20 random INVALID start/goal configs. 
    // Run the searchGraph method on each
    // Save the results to csv with rows:
    //    start, goal, path (or none)
    // Visually inspect each for correctness in Python/Matplotlib
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
    testSearchGraph(graph, obstacleData);

    std::cout << "Validating the local planning module..." << std::endl;
    RRT rrt;





    return 0;
}


