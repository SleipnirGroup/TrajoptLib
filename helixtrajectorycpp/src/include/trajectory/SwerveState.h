#pragma once

#include <iostream>
#include <vector>

#include "trajectory/HolonomicState.h"

namespace helixtrajectory {

class SwerveState : public HolonomicState {
public:
    std::vector<double> forceX;
    std::vector<double> forceY;

    SwerveState(double x, double y, double heading,
                double velocityX, double velocityY, double angularVelocity,
                double accelerationX, double accelerationY, double angularAcceleration,
                std::vector<double> forceX, std::vector<double> forceY);

    friend std::ostream& operator<<(std::ostream& stream, const SwerveState& state);
};
}