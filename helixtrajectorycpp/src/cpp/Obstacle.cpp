#include "Obstacle.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "ObstaclePoint.h"

namespace helixtrajectory {

    Obstacle::Obstacle(double safetyDistance, const std::vector<ObstaclePoint>& points)
        : safetyDistance(safetyDistance), points(points) {
    }

    std::ostream& operator<<(std::ostream& stream, const Obstacle& obstacle) {
        return stream << "{\"safety_distance\": " << obstacle.safetyDistance
                << ", \"points\": " << obstacle.points
                << "}";
    }
}