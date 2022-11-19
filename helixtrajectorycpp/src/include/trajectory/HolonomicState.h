#pragma once

#include <iostream>

#include "trajectory/State.h"

namespace helixtrajectory {

class HolonomicState : public State {
public:
    double velocityX, velocityY, angularVelocity, accelerationX, accelerationY, angularAcceleration;

    HolonomicState(double x, double y, double heading, double velocityX, double velocityY, double angularVelocity,
            double accelerationX, double accelerationY, double angularAcceleration);

    friend std::ostream& operator<<(std::ostream& stream, const HolonomicState& state);
};
}