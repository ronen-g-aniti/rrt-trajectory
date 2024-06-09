#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>

class Obstacle {
public:
    Obstacle(float posX, float posY, float posZ, float halfsizeX, float halfsizeY, float halfsizeZ);

    float isCollision(const Eigen::Vector3f& point) const;
    float getMinX() const;
    float getMaxX() const;
    float getMinY() const;
    float getMaxY() const;
    float getMinZ() const;
    float getMaxZ() const;

private:
    float posX, posY, posZ;
    float halfSizeX, halfSizeY, halfSizeZ;
};

class ObstacleParser {
public:
    ObstacleParser(const std::string& filename);

    std::vector<Obstacle> getObstacles() const;
    const std::vector<float>& getBounds() const;

private:
    void parseCSV(const std::string& filename);
    void computeBounds();

    std::vector<Obstacle> obstacles;
    std::vector<float> bounds; // six element vector: minX, maxX, minY, maxY, minZ, maxZ
};
