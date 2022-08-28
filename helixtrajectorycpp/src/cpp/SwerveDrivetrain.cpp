#include "SwerveDrivetrain.h"

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <casadi/casadi.hpp>

#include "HolonomicDrivetrain.h"
#include "Obstacle.h"
#include "SwerveDrivetrain.h"

namespace helixtrajectory {

    SwerveDrivetrain::SwerveDrivetrain(double mass, double momentOfInertia, const std::vector<SwerveModule>& modules, const Obstacle& bumpers) :
        HolonomicDrivetrain(mass, momentOfInertia, bumpers), modules(modules) {
    }

    std::ostream& operator<<(std::ostream& stream, const SwerveDrivetrain& swerveDrivetrain) {
        return stream << "{\"mass\": " << swerveDrivetrain.mass
               << ", \"moment_of_inertia\": " << swerveDrivetrain.momentOfInertia
               << ", \"bumpers\": " << swerveDrivetrain.bumpers
               << ", \"modules\": " << swerveDrivetrain.modules
               << "}";
    }
}