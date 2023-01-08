// Copyright (c) TrajoptLib contributors

#include "constraint/ObstacleConstraint.h"

#include "obstacle/Obstacle.h"

namespace trajopt {

ObstacleConstraint::ObstacleConstraint(Obstacle obstacle)
    : obstacle(std::move(obstacle)) {}
}  // namespace trajopt
