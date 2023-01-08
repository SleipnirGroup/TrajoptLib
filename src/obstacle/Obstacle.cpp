// Copyright (c) TrajoptLib contributors

#include "obstacle/Obstacle.h"

#include <iostream>
#include <vector>

#include "obstacle/ObstaclePoint.h"

namespace trajopt {

Obstacle::Obstacle(double safetyDistance,
                   const std::vector<ObstaclePoint>& points)
    : safetyDistance(safetyDistance), points(points) {}

std::ostream& operator<<(std::ostream& stream, const Obstacle& obstacle) {
  stream << std::boolalpha;
  return stream << "{\"safety_distance\": "
                << obstacle.safetyDistance
                // << ", \"points\": " << obstacle.points
                << "}";
}
}  // namespace trajopt
