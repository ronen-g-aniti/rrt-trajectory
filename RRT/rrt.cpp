#include "rrt.h"
#include <random>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace nanoflann;

RRT::RRT(ObstacleParser& environment, Vector3d startPos, Vector3d goalPos, double goalBias, double maxSteerAngleRate,
    double timeStep, double timeInterval, double speed, int maxIterations, double goalTolerance)
    : environment(environment), startPos(startPos), goalPos(goalPos), goalBias(goalBias),
    maxSteerAngle(maxSteerAngleRate* timeStep), timeStep(timeStep), speed(speed),
    maxIterations(maxIterations), goalTolerance(goalTolerance), timeInterval(timeInterval),
    integrationSteps(static_cast<int>(timeInterval / timeStep)), goalIsFound(false) {

    // The drone's attitude will be pointing towards the goal position, initially
    Vector3d delta = goalPos - startPos;
    Vector3d startOrientation = delta.normalized();

    // Define the starting state (position + orientation + time)
    VectorXd startState(7); // x, y, z, vx, vy, vz, t
    startState << startPos, startOrientation, 0.0; // 0.0 is the initial time

    // Add the starting state to the list of states
    states.push_back(startState);
    edges[0] = -1; // The starting state has no parent

    // Initialize the KD-Tree
    cloud.points.push_back({ startState[0], startState[1], startState[2] });
    kdTree = new myKDTree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdTree->buildIndex();

}

vector<VectorXd> RRT::run() {

    for (int i = 0; i < maxIterations; i++) {
        
        Vector3d samplePoint = sampleWithBias();
        int nearestIndex = findNearestState(samplePoint);
        VectorXd nearestState = states[nearestIndex];
        VectorXd newState = integrateForward(nearestState, samplePoint);

        if (goalIsFound) {
        
            states.push_back(newState);
            int newIndex = states.size() - 1;
            edges[newIndex] = nearestIndex;
            return constructPath(newIndex);
        } 

        states.push_back(newState);
        int newIndex = states.size() - 1;
        edges[newIndex] = nearestIndex;

        // Add the new state to the KD-Tree
        cloud.points.push_back({ newState[0], newState[1], newState[2] });
        kdTree->buildIndex();
    } 
    cerr << "Path not found!" << endl;
    return {};

}

Vector3d RRT::sampleWithBias() { 
    
    // Random number generators
    // Static variables are used to avoid reseeding the random number generator
    static random_device rd;
    static mt19937 gen(rd());
    static uniform_real_distribution<> dis(0.0, 1.0);
    static uniform_real_distribution<> disX(environment.getBounds()[0], environment.getBounds()[1]);
    static uniform_real_distribution<> disY(environment.getBounds()[2], environment.getBounds()[3]);
    static uniform_real_distribution<> disZ(environment.getBounds()[4], environment.getBounds()[5]);

    // With probability goalBias, return the goal position
    if (dis(gen) < goalBias) { 
		return goalPos;
    }
    else {
		return Vector3d(disX(gen), disY(gen), disZ(gen));
	}
}

int RRT::findNearestState(const Vector3d& queryPos) {

    const double queryPoint[3] = { queryPos[0], queryPos[1], queryPos[2] };
    size_t retIndex; // Index of the nearest neighbor
    double outDistSqr; // Squared distance to the nearest neighbor
    nanoflann::KNNResultSet<double> resultSet(1); // K=1, meaning we only want the nearest neighbor
    resultSet.init(&retIndex, &outDistSqr);
    kdTree->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParameters());
    return static_cast<int>(retIndex);
}

VectorXd RRT::integrateForward(const VectorXd& nearestState, const Vector3d& samplePoint) { 


    VectorXd currentState = nearestState; 
    for (int i = 0; i < integrationSteps; i++) { 
    
        VectorXd priorState = currentState; 
        currentState = updateState(currentState, samplePoint); 

        //Debugging
        cout << "Current state: " << currentState.head<6>().transpose() << endl;

        if ((goalPos - currentState.head<3>()).norm() < goalTolerance) {
			goalIsFound = true;
            cout << "Goal found!" << endl;
			return currentState; 
		}
        if (isCollision(currentState.head<3>()) || !isInsideEnvironment(currentState.head<3>())) {
			return priorState;
		}
    }
    return currentState;
} 

VectorXd RRT::updateState(const VectorXd& nearestState, const Vector3d& samplePoint) { 

    Vector3d priorAttitude = nearestState.segment<3>(3);
    Vector3d priorPosition = nearestState.head<3>();
    double priorTime = nearestState[6];
    Vector3d u1 = (samplePoint - priorPosition).normalized(); // Points from the current position to the sample point
    Vector3d axisVector = priorAttitude.cross(u1); // Cross the prior attitude with the direction to the sample point to get the axis of rotation
    
    Matrix3d rotationMatrix;

    if (axisVector.norm() <= 0.01) {
        rotationMatrix = Matrix3d::Identity(); // Prevents a zero axis vector from causing a NaN rotation
    }
    else {
        Vector3d u2 = axisVector.normalized(); // Normalized axis of rotation
        double theta = acos(priorAttitude.dot(u1)); // Angle between prior attitude and direction to sample point serves as the angle of rotation
        double steerAngle = std::clamp(theta, -maxSteerAngle, maxSteerAngle); // Clamp the steering angle to prevent large angular changes (we want flyable paths)
        Matrix3d K;
        K << 0, -u2.z(), u2.y(),
			u2.z(), 0, -u2.x(),
			-u2.y(), u2.x(), 0; // Skew-symmetric matrix for the cross product with the axis of rotation
		rotationMatrix = Matrix3d::Identity() + sin(steerAngle) * K + (1 - cos(steerAngle)) * K * K; // Rodrigues' rotation formula

        Vector3d newAttitude = rotationMatrix * priorAttitude; // Rotate the prior attitude to get the new attitude
        Vector3d newPosition = priorPosition + speed * timeStep * newAttitude; // Move the drone in the direction of the new attitude
        double newTime = priorTime + timeStep; // Increment the time
        VectorXd newState(7); // x, y, z, vx, vy, vz, t
        newState << newPosition, newAttitude, newTime;
        return newState;
    }
}

bool RRT::isCollision(const Vector3d& point) {
    for (const auto& obstacle : environment.getObstacles()) {
        if (obstacle.isCollision(point.cast<float>())) {
            return true;
        }
    }
    return false;
}

bool RRT::isInsideEnvironment(const Vector3d& point) {
    const auto& bounds = environment.getBounds();
    return (point.x() >= bounds[0] && point.x() <= bounds[1] &&
        		point.y() >= bounds[2] && point.y() <= bounds[3] &&
        		point.z() >= bounds[4] && point.z() <= bounds[5]);
}

vector<VectorXd> RRT::constructPath(int newIndex) {
    vector<int> path;
    path.push_back(newIndex);
    int parent = edges[newIndex];
    while (parent != -1) {
        path.push_back(parent);
        parent = edges[parent];

    }
    reverse(path.begin(), path.end());

    vector<VectorXd> pathAsStates;
    for (int idx : path) {
        pathAsStates.push_back(states[idx]);

    }
    return pathAsStates;
}
