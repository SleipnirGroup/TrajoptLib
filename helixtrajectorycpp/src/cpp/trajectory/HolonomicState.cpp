#include "trajectory/HolonomicState.h"

#include <iostream>

namespace helixtrajectory {

HolonomicState::HolonomicState(double x, double y, double heading,
        double velocityX, double velocityY, double angularVelocity,
        double accelerationX, double accelerationY, double angularAcceleration)
        : State(x, y, heading),
        velocityX(velocityX), velocityY(velocityY), angularVelocity(angularVelocity),
        accelerationX(accelerationX), accelerationY(accelerationY), angularAcceleration(angularAcceleration) {
}

std::ostream& operator<<(std::ostream& stream, const HolonomicState& state) {
    return stream << "{\"x\": " << state.x
            << ", \"y\": " << state.y
            << ", \"heading\": " << state.heading
            << ", \"velocity_x\": " << state.velocityX
            << ", \"velocity_y\": " << state.velocityY
            << ", \"angular_velocity\": " << state.angularVelocity
            << ", \"acceleration_x\": " << state.accelerationX
            << ", \"acceleration_y\": " << state.accelerationY
            << ", \"angular_acceleration\": " << state.angularAcceleration
            << "}";
}
}