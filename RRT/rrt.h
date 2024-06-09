#pragma once

#include <vector>
#include <map>
#include <Eigen/Dense>
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
	RRT(ObstacleParser& environment, Eigen::Vector3d startPos, Eigen::Vector3d goalPos,
		double goalBias = 0.5, double maxSteeringAngleRate = M_PI / 24, double timeStep = 0.1,
		double timeInterval = 5.0, double speed = 2.0, int maxIterations = 10000, double goalTolerance = 1.0);

	std::vector<Eigen::VectorXd> run(); 

private: 
	ObstacleParser& environment;
	Eigen::Vector3d startPos;
	Eigen::Vector3d goalPos;
	double goalBias;
	double maxSteerAngle;
	double timeStep;
	double speed;
	int maxIterations;
	double goalTolerance;
	double timeInterval;
	int integrationSteps;
	bool goalIsFound;
	bool steering;
	std::vector<Eigen::VectorXd> states;
	std::map<int, int> edges;
	PointCloud cloud;
	myKDTree* kdTree;

	Eigen::Vector3d sampleWithBias();
	int findNearestState(const Eigen::Vector3d& queryPos);
	Eigen::VectorXd integrateForward(const Eigen::VectorXd& nearestState, const Eigen::Vector3d& samplePoint);
	Eigen::VectorXd updateState(const Eigen::VectorXd& nearestState, const Eigen::Vector3d& samplePoint);
	bool isCollision(const Eigen::Vector3d& point);
	bool isInsideEnvironment(const Eigen::Vector3d& point);
	std::vector<Eigen::VectorXd> constructPath(int newIndex);
};