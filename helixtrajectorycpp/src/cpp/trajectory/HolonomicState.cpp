// #include "trajectory/HolonomicState.h"

// #include <iostream>
// #include <fmt/format.h>

// namespace helixtrajectory {

// HolonomicState::HolonomicState(double x, double y, double heading,
//         double velocityX, double velocityY, double angularVelocity,
//         double accelerationX, double accelerationY, double angularAcceleration)
//         : State(x, y, heading),
//         velocityX(velocityX), velocityY(velocityY), angularVelocity(angularVelocity),
//         accelerationX(accelerationX), accelerationY(accelerationY), angularAcceleration(angularAcceleration) {
// }

// std::ostream& operator<<(std::ostream& stream, const HolonomicState& state) {
//     return stream << fmt::format(
//             "{{\"x\": {}, \"y\": {}, \"heading\": {}, \"velocity_x\": {}, \"velocity_y\": {}, \"angular_velocity\": {}, \"acceleration_x\": {}, \"acceleration_y\": {}, \"angular_acceleration\": {}}}",
//             state.x, state.y, state.heading,
//             state.velocityX, state.velocityY, state.angularVelocity,
//             state.accelerationX, state.accelerationY, state.angularAcceleration);
// }
// }