#include "Drivetrain.h"

#include <casadi/casadi.hpp>

namespace helixtrajectory {

    Drivetrain::Drivetrain(double mass, double momentOfInertia, const Obstacle& bumpers)
        : mass(mass), momentOfInertia(momentOfInertia), bumpers(bumpers) {
    }

    Drivetrain::~Drivetrain() {
    }
}