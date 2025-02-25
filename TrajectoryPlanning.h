#pragma once
#include <Eigen/Dense>
#include <vector>
#include "Types.h"

struct TrajectorySolution {
    Eigen::VectorXd xCoeffs;
    Eigen::VectorXd yCoeffs;
    Eigen::VectorXd zCoeffs;
};

class TrajectoryPlanner {
public:

    TrajectoryPlanner(const Points3D& waypoints, double maxSpeed);

    TrajectorySolution solveTrajectory();

    Eigen::VectorXd derivativePowers(double normalizedTime, int derivativeOrder);

    std::vector<double> computeElapsedTimes();

    std::vector<double> computeStartTimes();

    int findSegmentIndex(double globalTime);

    double normalizeTime(double globalTime);

    Eigen::VectorXd getCoeffsForSegment(int segmentIndex, const Eigen::VectorXd& allCoeffs);

    Eigen::MatrixXd assembleBoundaryMatrix();

    Eigen::MatrixXd assembleContinuityMatrix();

    Eigen::MatrixXd assembleWaypointMatrix();

    Eigen::VectorXd solveSystemOnAxis(int axis);

    double maxSpeed;
    double desiredSpeed;
    int numSegments;
    int numCoeffs;
    int numWaypoints;
    const Points3D waypoints;
    Eigen::MatrixXd eigenWaypoints;
    std::vector<double> elapsedTimes; 
    std::vector<double> startTimes;

};
