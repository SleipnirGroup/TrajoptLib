// Copyright (c) TrajoptLib contributors

#include "constraint/ObstacleConstraint.h"

#include "obstacle/Obstacle.h"

namespace trajopt {

ObstacleConstraint::ObstacleConstraint(const Obstacle& obstacle)
    : obstacle(obstacle) {}
}  // namespace trajopt
