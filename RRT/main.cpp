#include "rrt.h"
#include "obstacles.h"
#include <iostream>
#include <Eigen/Dense>
#include <fstream>

int main() {
    // Define environment and obstacles
    ObstacleParser environment("obstacles.csv");

    // Define start and goal positions
    Eigen::Vector3d startPos(0.0, 0.0, 0.0);
    Eigen::Vector3d goalPos(10.0, 10.0, 10.0);

    // Create RRT object
    RRT rrt(environment, startPos, goalPos);

    // Run RRT
    std::vector<Eigen::VectorXd> path = rrt.run();

    // Print the path
    std::cout << "Path:" << std::endl;
    for (const auto& state : path) {
        std::cout << state.transpose() << std::endl;
    }

    return 0;
}