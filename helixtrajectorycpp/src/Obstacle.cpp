#include "Obstacle.h"

#include <vector>

namespace helixtrajectory {

    Obstacle::Obstacle(double safetyDistance, const std::vector<ObstaclePoint>& points)
        : safetyDistance(safetyDistance), points(points) {
    }
}