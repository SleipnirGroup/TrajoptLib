#include "drivetrain/SwerveDrivetrain.h"

#include <iostream>
#include <cmath>
#include <vector>

#include <casadi/casadi.hpp>

#include "drivetrain/HolonomicDrivetrain.h"
#include "drivetrain/SwerveDrivetrain.h"
#include "obstacle/Obstacle.h"

namespace helixtrajectory {

    SwerveDrivetrain::SwerveDrivetrain(double mass, double momentOfInertia, const std::vector<SwerveModule>& modules) :
        HolonomicDrivetrain(mass, momentOfInertia), modules(modules) {
    }

    std::ostream& operator<<(std::ostream& stream, const SwerveDrivetrain& swerveDrivetrain) {
        return stream << "{\"mass\": " << swerveDrivetrain.mass
               << ", \"moment_of_inertia\": " << swerveDrivetrain.momentOfInertia
               << ", \"modules\": " << swerveDrivetrain.modules
               << "}";
    }
}