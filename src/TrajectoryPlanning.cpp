#include "TrajectoryPlanning.h"
#include <cmath>

// Constructor
TrajectoryPlanner::TrajectoryPlanner(const Points3D& waypoints, double maxSpeed) : waypoints(waypoints), maxSpeed(maxSpeed) {
    
    // Set the desired speed to be fraction of the max speed
    desiredSpeed = maxSpeed / 5.0;
    
    // Convert the waypoints from the custom type to the Eigen type
    numWaypoints = static_cast<int>(waypoints.size());
    eigenWaypoints.resize(numWaypoints, 3);
    for (int i=0; i< numWaypoints; i++){
        eigenWaypoints(i, 0) = static_cast<double>(waypoints[i][0]);
        eigenWaypoints(i, 1) = static_cast<double>(waypoints[i][1]);
        eigenWaypoints(i, 2) = static_cast<double>(waypoints[i][2]);

    }

    // Compute the number of segments
    numSegments = numWaypoints - 1;

    // Compute number of unknown coefficients
    numCoeffs = 8 * numSegments;

    // Compute the elapsed times for each segment
    elapsedTimes = this->computeElapsedTimes();

    // Compute start times for each segment
    startTimes = this->computeStartTimes();  


}

// derivativePowers:
// Computes the falling factorial i*(i-1)*(i-2)*....(i-derivativeOrder+1 multiplied by the normalized time and returns the result as an eight element vector.  
Eigen::VectorXd TrajectoryPlanner::derivativePowers(double normalizedTime, int derivativeOrder){
    Eigen::VectorXd result = Eigen::VectorXd::Zero(8); // column vector
    for (int i=derivativeOrder; i<8; i++){
        double coeff = 1.0;
        for (int j = i - derivativeOrder + 1; j <= i; j++){
            coeff *= j;

        }
        result(i) = coeff * std::pow(normalizedTime, i-derivativeOrder);
        
    }
    return result;
}

//elapsedTime:
// Computes time allotment for each segment based on Euclidean distance between consecutive eigenWaypoints. 
std::vector<double> TrajectoryPlanner::computeElapsedTimes(){
    int n = eigenWaypoints.rows();
    elapsedTimes.clear();
    for (int i=0; i < n-1; i++){
        double distance = (eigenWaypoints.row(i+1) - eigenWaypoints.row(i)).norm();
        elapsedTimes.push_back(distance / desiredSpeed);
    }
    return elapsedTimes;


}

//startTime:
std::vector<double> TrajectoryPlanner::computeStartTimes(){
    double t = 0.0;
    for (double elapsedTime : elapsedTimes){
        startTimes.push_back(t);
        t += elapsedTime;
    }
    return startTimes;

}

// findSegmentIndex
int TrajectoryPlanner::findSegmentIndex(double globalTime){
    for (int i=0; i < startTimes.size() - 1; i++){
        if (startTimes[i] <= globalTime && globalTime < startTimes[i+1]){
            return i;
        }
    }
    return startTimes.size() - 1;
}

// normalizeTime
double TrajectoryPlanner::normalizeTime(double globalTime){
    int segmentIndex = findSegmentIndex(globalTime);
    double localTime = globalTime - startTimes[segmentIndex];
    double normalizedTime = 2.0 * localTime / elapsedTimes[segmentIndex] - 1.0;
    return normalizedTime;
}

// getCoeffsForSegment
Eigen::VectorXd TrajectoryPlanner::getCoeffsForSegment(int segmentIndex, const Eigen::VectorXd& allCoeffs){
    const int coeffsPerSegment = 8;
    int startIndex = segmentIndex * coeffsPerSegment;
    return allCoeffs.segment(startIndex, coeffsPerSegment);

}

// assembleBoundaryMatrix
Eigen::MatrixXd TrajectoryPlanner::assembleBoundaryMatrix(){
    Eigen::MatrixXd ABar1 = Eigen::MatrixXd::Zero(6, numCoeffs);

    // Beginning of first segment
    double tFirst = elapsedTimes[0];
    double scaleFactor1 = 2.0 / tFirst; // A result of the chain rule on differentiating the normalized time variable
    ABar1.block(0, 0, 1, 8) = derivativePowers(-1.0, 1).transpose() * scaleFactor1;
    ABar1.block(1, 0, 1, 8) = derivativePowers(-1.0, 2).transpose() * std::pow(scaleFactor1, 2);
    ABar1.block(2, 0, 1, 8) = derivativePowers(-1.0, 3).transpose() * std::pow(scaleFactor1, 3);

    // End of final segment
    double tFinal = elapsedTimes[numSegments-1];
    double scaleFactor2 = 2.0 / tFinal;
    int segFinal = numSegments - 1;
    int colStart = segFinal * 8; 
    ABar1.block(3, colStart, 1, 8) = derivativePowers(1.0, 1).transpose() * scaleFactor2;
    ABar1.block(4, colStart, 1, 8) = derivativePowers(1.0, 2).transpose() * std::pow(scaleFactor2, 2);
    ABar1.block(5, colStart, 1, 8) = derivativePowers(1.0, 3).transpose() * std::pow(scaleFactor2, 3);
    
    return ABar1;
}

// assembleWaypointMatrix
Eigen::MatrixXd TrajectoryPlanner::assembleWaypointMatrix() {
    Eigen::MatrixXd ABar2 = Eigen::MatrixXd::Zero(numWaypoints, numCoeffs);
    for (int i=0; i < numSegments; i++){
        int colStart = 8 * i; 
        ABar2.block(i, colStart, 1, 8) = derivativePowers(-1.0, 0).transpose();

    }
    int colStart = (numSegments - 1) * 8;
    ABar2.block(numWaypoints-1, colStart, 1, 8) = derivativePowers(1.0, 0).transpose(); 

    return ABar2;
}

//assembleContinuityMatrix
Eigen::MatrixXd TrajectoryPlanner::assembleContinuityMatrix(){
    int numRows = 7 * (numWaypoints - 2); // Number of intermediate waypoints * the number of derivatives being forced to be continuous at the waypoint transition points (in this case, 7: derivatives 0 to 6 (position...pop))
    int numCols = numCoeffs; 
    Eigen::MatrixXd ABar3 = Eigen::MatrixXd::Zero(numRows, numCols);

    int rowIndex = 0;
    for (int i = 0; i < numWaypoints - 2; i++){
        double tLeft = elapsedTimes[i];
        double tRight = elapsedTimes[i+1];
        // For derivatives 0 to 6
        for (int order = 0; order <= 6; order++){
            // Right most position on the left segment
            Eigen::VectorXd leftVec = derivativePowers(1.0, order) * std::pow(2.0 / tLeft, order);
            //  Left most position on the right segment
            Eigen::VectorXd rightVec = -1.0 * derivativePowers(-1.0, order) * std::pow(2.0 / tRight, order);
            Eigen::VectorXd concat(16);
            concat << leftVec, rightVec;
            ABar3.block(rowIndex, 8*i, 1, 16) = concat.transpose();
            rowIndex++; 
        }
    }
    return ABar3;
}

//solveSystemOnAxis
Eigen::VectorXd TrajectoryPlanner::solveSystemOnAxis(int axis){

    Eigen::MatrixXd A1 = assembleBoundaryMatrix();
    Eigen::MatrixXd A2 = assembleWaypointMatrix(); 
    Eigen::MatrixXd A3 = assembleContinuityMatrix(); 

    int totalRows = A1.rows() + A2.rows() + A3.rows(); 
    int totalCols = A1.cols(); 
    Eigen::MatrixXd AAug(totalRows, totalCols);
    AAug << A1, A2, A3;

    // Build the right hand side vector
    Eigen::VectorXd b = Eigen::VectorXd::Zero(totalRows);
    //Set the waypoint constraints
    // Skip the start/end boundary constraints (they are 0s)
    for (int i=0; i < numWaypoints; i++){
        b(A1.rows() + i) = eigenWaypoints(i, axis);
    }
    // The continuity constraints remain zero.

    // Solve the system with Eigen
    Eigen::VectorXd solution = AAug.fullPivLu().solve(b);
    return solution;
}


//solve
TrajectorySolution TrajectoryPlanner::solveTrajectory(){
    TrajectorySolution sol;
    sol.xCoeffs = solveSystemOnAxis(0);
    sol.yCoeffs = solveSystemOnAxis(1);
    sol.zCoeffs = solveSystemOnAxis(2);
    return sol; 
}