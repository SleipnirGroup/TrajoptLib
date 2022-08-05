#include "Obstacle.h"

namespace helixtrajectory {

    Obstacle::Obstacle(const double safetyDistance, const std::vector<ObstaclePoint>& points)
        : safetyDistance(safetyDistance), points(points) {
    }
}