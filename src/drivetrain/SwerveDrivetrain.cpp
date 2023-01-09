// Copyright (c) TrajoptLib contributors

#include "drivetrain/SwerveDrivetrain.h"

#include <cmath>
#include <cstddef>
#include <string>

#include "IncompatibleTrajectoryException.h"
#include "drivetrain/HolonomicDrivetrain.h"
#include "obstacle/Obstacle.h"

namespace trajopt {

SwerveDrivetrain::SwerveDrivetrain(double mass, double momentOfInertia,
                                   std::vector<SwerveModule> modules)
    : HolonomicDrivetrain(mass, momentOfInertia), modules(std::move(modules)) {}

}  // namespace trajopt
