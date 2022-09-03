#include "Obstacle.h"

#include <iostream>
#include <vector>

#include <casadi/casadi.hpp>

#include "ObstaclePoint.h"

namespace helixtrajectory {

    Obstacle::Obstacle(double safetyDistance, bool applyToAllSegments, const std::vector<ObstaclePoint>& points)
        : safetyDistance(safetyDistance), applyToAllSegments(applyToAllSegments), points(points) {
    }

    std::ostream& operator<<(std::ostream& stream, const Obstacle& obstacle) {
        stream << std::boolalpha;
        return stream << "{\"safety_distance\": " << obstacle.safetyDistance
                << ", \"apply_to_all_segments\": " << obstacle.applyToAllSegments
                << ", \"points\": " << obstacle.points
                << "}";
    }
}