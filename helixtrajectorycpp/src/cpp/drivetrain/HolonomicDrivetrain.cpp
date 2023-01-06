// Copyright (c) TrajoptLib contributors

#include "drivetrain/HolonomicDrivetrain.h"

#include "drivetrain/Drivetrain.h"
#include "obstacle/Obstacle.h"

namespace helixtrajectory {

HolonomicDrivetrain::HolonomicDrivetrain(double mass, double momentOfInertia)
    : Drivetrain(mass, momentOfInertia) {}
}  // namespace helixtrajectory
