#include "rrt.h"
#include <cmath>
#include <iostream>
#include <random>
#include <algorithm>
#include <numeric>

// Constructor
RRT::RRT(ObstacleParser& environment, std::vector<double> startPos, std::vector<double> goalPos,
	double goalBias, double maxSteeringAngleRate, double timeStep, double timeInterval,
	double speed, int maxIterations, double goalTolerance) :
	environment(environment), startPos(startPos), goalPos(goalPos), goalBias(goalBias),
	maxSteerAngle(maxSteeringAngleRate* timeStep), timeStep(timeStep), speed(speed),
	maxIterations(maxIterations), goalTolerance(goalTolerance), timeInterval(timeInterval),
	integrationSteps(static_cast<int>(timeInterval / timeStep)), goalIsFound(false) {

	// Calculate initial attitude vector from start position to goal position
	std::vector<double> initialAttitude(3);
	for (int i = 0; i < 3; ++i) {
		initialAttitude[i] = goalPos[i] - startPos[i];
	}
	double norm = std::sqrt(std::inner_product(initialAttitude.begin(), initialAttitude.end(), initialAttitude.begin(), 0.0));
	std::transform(initialAttitude.begin(), initialAttitude.end(), initialAttitude.begin(), [norm](double val) { return val / norm; });

	// Initial state: position = startPos, attitude = initialAttitude, time = 0
	std::vector<double> startState = { startPos[0], startPos[1], startPos[2], initialAttitude[0], initialAttitude[1], initialAttitude[2], 0.0 };
	states.push_back(startState);
	edges[0] = -1;
	cloud.points.push_back({ startState[0], startState[1], startState[2] });
	kdTree = new myKDTree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
	kdTree->buildIndex();
}

// Sample a point in the environment
std::vector<double> RRT::sampleWithBias() {
	static std::random_device rd;
	static std::mt19937 gen(rd());
	static std::uniform_real_distribution<> dis(0.0, 1.0);
	static std::uniform_real_distribution<> disX(environment.getBounds()[0], environment.getBounds()[1]);
	static std::uniform_real_distribution<> disY(environment.getBounds()[2], environment.getBounds()[3]);
	static std::uniform_real_distribution<> disZ(environment.getBounds()[4], environment.getBounds()[5]);

	if (dis(gen) < goalBias) {
		return goalPos;
	}
	else {
		return { disX(gen), disY(gen), disZ(gen) };
	}
}

// Find the nearest state to the query point
int RRT::findNearestState(const std::vector<double>& queryPos) {
	const double queryPoint[3] = { queryPos[0], queryPos[1], queryPos[2] };
	size_t retIndex;
	double outDistSqr;
	nanoflann::KNNResultSet<double> resultSet(1);
	resultSet.init(&retIndex, &outDistSqr);
	kdTree->findNeighbors(resultSet, &queryPoint[0], nanoflann::SearchParameters());
	
	return static_cast<int>(retIndex);

	
}

// Integrate the state forward in time
std::vector<double> RRT::integrateForward(const std::vector<double>& nearestState, const std::vector<double>& samplePoint) {
	std::cout << "Integrating forward from " << nearestState[0] << ", " << nearestState[1] << ", " << nearestState[2] << "\n";
	std::vector<double> currentState = nearestState;
	for (int i = 0; i < integrationSteps; i++) {
		std::vector<double> priorState = currentState;
		currentState = updateState(currentState, samplePoint);

		if (std::sqrt(std::pow(goalPos[0] - currentState[0], 2) +
			std::pow(goalPos[1] - currentState[1], 2) +
			std::pow(goalPos[2] - currentState[2], 2)) < goalTolerance) {
			goalIsFound = true;
			std::cout << "Goal found!" << std::endl;
			return currentState;
		}
		if (isCollision({ currentState[0], currentState[1], currentState[2] }) ||
			!isInsideEnvironment({ currentState[0], currentState[1], currentState[2] })) {

			return priorState;
		}
	}
	return currentState;
}

// Update the state of the drone
std::vector<double> RRT::updateState(const std::vector<double>& nearestState, const std::vector<double>& samplePoint) {
	std::vector<double> priorAttitude(nearestState.begin() + 3, nearestState.begin() + 6);
	std::vector<double> priorPosition(nearestState.begin(), nearestState.begin() + 3);
	double priorTime = nearestState[6];

	std::vector<double> u1(3);
	for (int i = 0; i < 3; ++i) {
		u1[i] = samplePoint[i] - priorPosition[i];
	}
	double u1_norm = std::sqrt(std::inner_product(u1.begin(), u1.end(), u1.begin(), 0.0));
	std::transform(u1.begin(), u1.end(), u1.begin(), [u1_norm](double val) { return val / u1_norm; });

	std::vector<double> axisVector(3);
	axisVector[0] = priorAttitude[1] * u1[2] - priorAttitude[2] * u1[1];
	axisVector[1] = priorAttitude[2] * u1[0] - priorAttitude[0] * u1[2];
	axisVector[2] = priorAttitude[0] * u1[1] - priorAttitude[1] * u1[0];

	double axisVector_norm = std::sqrt(std::inner_product(axisVector.begin(), axisVector.end(), axisVector.begin(), 0.0));
	if (axisVector_norm <= 0.01) {
		axisVector = { 0.0, 0.0, 0.0 };
	}
	else {
		std::transform(axisVector.begin(), axisVector.end(), axisVector.begin(), [axisVector_norm](double val) { return val / axisVector_norm; });
	}

	double theta = std::acos(std::inner_product(priorAttitude.begin(), priorAttitude.end(), u1.begin(), 0.0));
	double steerAngle = std::clamp(theta, -maxSteerAngle, maxSteerAngle);

	std::vector<std::vector<double>> K = {
		{0, -axisVector[2], axisVector[1]},
		{axisVector[2], 0, -axisVector[0]},
		{-axisVector[1], axisVector[0], 0}
	};

	std::vector<std::vector<double>> rotationMatrix = {
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1}
	};

	if (axisVector_norm > 0.01) {
		double sinTheta = std::sin(steerAngle);
		double cosTheta = std::cos(steerAngle);
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				rotationMatrix[i][j] += sinTheta * K[i][j];
				for (int k = 0; k < 3; ++k) {
					rotationMatrix[i][j] += (1 - cosTheta) * K[i][k] * K[k][j];
				}
			}
		}
	}

	std::vector<double> newAttitude(3);
	for (int i = 0; i < 3; ++i) {
		newAttitude[i] = 0;
		for (int j = 0; j < 3; ++j) {
			newAttitude[i] += rotationMatrix[i][j] * priorAttitude[j];
		}
	}

	std::vector<double> newPosition(3);
	for (int i = 0; i < 3; ++i) {
		newPosition[i] = priorPosition[i] + speed * timeStep * newAttitude[i];
	}

	double newTime = priorTime + timeStep;
	std::vector<double> newState = { newPosition[0], newPosition[1], newPosition[2], newAttitude[0], newAttitude[1], newAttitude[2], newTime };
	return newState;
}

std::vector<std::vector<double>> RRT::run() {
	for (int i = 0; i < maxIterations; i++) {
		std::vector<double> samplePoint = sampleWithBias();
		int nearestIndex = findNearestState(samplePoint);
		std::vector<double> nearestState = states[nearestIndex];

		std::vector<double> newState = integrateForward(nearestState, samplePoint);
		if (goalIsFound) {
			states.push_back(newState);
			int newIndex = states.size() - 1;
			edges[newIndex] = nearestIndex;
			return constructPath(newIndex);
		}

		states.push_back(newState);
		int newIndex = states.size() - 1;
		edges[newIndex] = nearestIndex;

		cloud.points.push_back({ newState[0], newState[1], newState[2] });
		kdTree->buildIndex();
	}
	std::cerr << "Path not found!" << std::endl;
	return {};
}

std::vector<std::vector<double>> RRT::constructPath(int newIndex) {
	std::vector<int> path;
	path.push_back(newIndex);
	int parent = edges[newIndex];
	while (parent != -1) {
		path.push_back(parent);
		parent = edges[parent];
	}
	std::reverse(path.begin(), path.end());

	std::vector<std::vector<double>> pathAsStates;
	for (int idx : path) {
		pathAsStates.push_back(states[idx]);
	}
	return pathAsStates;
}

bool RRT::isCollision(const std::vector<double>& point) {
	for (const auto& obstacle : environment.getObstacles()) {
		if (obstacle.isCollision(point)) {
			return true;
		}
	}
	return false;
}

bool RRT::isInsideEnvironment(const std::vector<double>& point) {
	const auto& bounds = environment.getBounds();
	return (point[0] >= bounds[0] && point[0] <= bounds[1] &&
		point[1] >= bounds[2] && point[1] <= bounds[3] &&
		point[2] >= bounds[4] && point[2] <= bounds[5]);
}
