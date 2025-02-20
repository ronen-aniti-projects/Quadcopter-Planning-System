// TrajectoryPlanner Class


// Methods
// derivativePowers
// elapsedTime
// startTime
// findSegmentTime
// normalizeTime
// getCoeffsForSegment
// assembleBoundaryMatrix
// assembleContinuityMatrix
// buildSystemOnAxis
// solve

// Data
//data memebers go here
#pragma once
#include <Eigen/Dense>
#include <vector>

struct TrajectorySolution {
    Eigen::VectorXd xCoeffs;
    Eigen::VectorXd yCoeffs;
    Eigen::VectorXd zCoeffs;
};

class TrajectoryPlanner {
public:

    TrajectoryPlanner(double maxSpeed);

    TrajectorySolution solve(const Points3D& waypoints);

    Eigen::VectorXd derivativePowers(double normalizedTime, int derivativeOrder)
}
