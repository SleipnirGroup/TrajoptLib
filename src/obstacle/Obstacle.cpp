// Copyright (c) TrajoptLib contributors

#include "obstacle/Obstacle.h"

#include <vector>

#include "obstacle/ObstaclePoint.h"

namespace trajopt {

Obstacle::Obstacle(double safetyDistance, std::vector<ObstaclePoint> points)
    : safetyDistance(safetyDistance), points(std::move(points)) {}

}  // namespace trajopt
