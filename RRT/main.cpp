#include "rrt.h"
#include "obstacles.h"
#include <iostream>
#include <fstream>
#include <vector>

// Function to save trajectory to a CSV file
void saveTrajectory(const std::vector<std::vector<double>>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    // Write the header
    file << "x,y,z,vx,vy,vz,t\n";

    // Write the data
    for (const auto& state : path) {
        for (size_t i = 0; i < state.size(); ++i) {
            file << state[i];
            if (i < state.size() - 1) {
                file << ",";
            }
        }
        file << "\n";
    }

    file.close();
    std::cout << "Trajectory saved to " << filename << std::endl;
}

void saveStates(const std::vector<std::vector<double>>& states, const std::string& filename) {
	std::ofstream file(filename);
    if (!file.is_open()) {
		std::cerr << "Error opening file: " << filename << std::endl;
		return;
	
    }

    // Write the header
    file << "x,y,z,vx,vy,vz,t\n";

    // Write the data
    for (const auto& state : states) {
        for (size_t i = 0; i < state.size(); ++i) {
			file << state[i];
            if (i < state.size() - 1) {
				file << ",";
			}
		}
		file << "\n";
	
    }

    file.close();
    std::cout << "States saved to " << filename << std::endl;
}

int main() {
    // Define environment and obstacles
    ObstacleParser environment("obstacles.csv");

    // Define start and goal positions
    std::vector<double> startPos = { 0.0, 0.0, 0.0 };
    std::vector<double> goalPos = { 110.0, -10, 150.0 };

    // Create RRT object
    RRT rrt(environment, startPos, goalPos);

    // Run RRT
    std::vector<std::vector<double>> path = rrt.run();

    // Print the path
    std::cout << "Path:" << std::endl;
    for (const auto& state : path) {
        for (double val : state) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }

    // Save the trajectory to a CSV file
    saveTrajectory(path, "trajectory.csv");

    // Save the states to a CSV file
    saveStates(rrt.getStates(), "states.csv");

    return 0;
}