#pragma once

#include <vector>
#include <map>
#include <nanoflann.hpp>
#include "obstacles.h"
#include <cmath>

#ifndef M_PI 
#define M_PI 3.14159265358979323846
#endif

struct PointCloud {
	struct Point {
		double x, y, z;
	};
	std::vector<Point> points;
	inline size_t kdtree_get_point_count() const { return points.size(); }
	inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
		if (dim == 0) return points[idx].x;
		else if (dim == 1) return points[idx].y;
		else return points[idx].z;
	}
	template <class BBOX> bool kdtree_get_bbox(BBOX&) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
	nanoflann::L2_Simple_Adaptor<double, PointCloud>,
	PointCloud,
	3
> myKDTree;

class RRT {

public:
	RRT(ObstacleParser& environment, std::vector<double> startPos, std::vector<double> goalPos,
		double goalBias = 0.20, double maxSteeringAngleRate = M_PI / 24, double timeStep = 0.1,
		double timeInterval = 2.0, double speed = 2.0, int maxIterations = 10000, double goalTolerance = 1.0);

	std::vector<std::vector<double>> run();
	std::vector<std::vector<double>> getStates() const { return states; }

private:
	ObstacleParser& environment;
	std::vector<double> startPos;
	std::vector<double> goalPos;
	double goalBias;
	double maxSteerAngle;
	double timeStep;
	double speed;
	int maxIterations;
	double goalTolerance;
	double timeInterval;
	int integrationSteps;
	bool goalIsFound;
	std::vector<std::vector<double>> states;
	std::map<int, int> edges;
	PointCloud cloud;
	myKDTree* kdTree;

	std::vector<double> sampleWithBias();
	int findNearestState(const std::vector<double>& queryPos);
	std::vector<double> integrateForward(const std::vector<double>& nearestState, const std::vector<double>& samplePoint);
	std::vector<double> updateState(const std::vector<double>& nearestState, const std::vector<double>& samplePoint);
	bool isCollision(const std::vector<double>& point);
	bool isInsideEnvironment(const std::vector<double>& point);
	std::vector<std::vector<double>> constructPath(int newIndex);
};
