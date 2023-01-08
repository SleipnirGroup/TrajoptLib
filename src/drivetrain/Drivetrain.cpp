// Copyright (c) TrajoptLib contributors

#include "drivetrain/Drivetrain.h"

namespace trajopt {

Drivetrain::Drivetrain(double mass, double momentOfInertia)
    : mass(mass), momentOfInertia(momentOfInertia) {}
}  // namespace trajopt
