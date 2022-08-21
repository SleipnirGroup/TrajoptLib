#include "HolonomicDrivetrain.h"

#include "Drivetrain.h"
#include "Obstacle.h"

namespace helixtrajectory {

    HolonomicDrivetrain::HolonomicDrivetrain(double mass, double momentOfInertia, const Obstacle& bumpers)
        : Drivetrain(mass, momentOfInertia, bumpers) {
    }

    HolonomicDrivetrain::~HolonomicDrivetrain() {
    }
}