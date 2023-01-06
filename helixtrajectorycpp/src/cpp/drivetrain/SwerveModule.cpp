// Copyright (c) TrajoptLib contributors

#include "drivetrain/SwerveModule.h"

#include <iostream>

namespace helixtrajectory {

SwerveModule::SwerveModule(double x, double y, double wheelRadius,
                           double wheelMaxAngularVelocity,
                           double wheelMaxTorque)
    : x(x),
      y(y),
      wheelRadius(wheelRadius),
      wheelMaxAngularVelocity(wheelMaxAngularVelocity),
      wheelMaxTorque(wheelMaxTorque) {}

std::ostream& operator<<(std::ostream& stream, const SwerveModule& module) {
  return stream << "{\"x\": " << module.x << ", \"y\": " << module.y
                << ", \"wheel_radius\": " << module.wheelRadius
                << ", \"wheel_max_angular_velocity\": "
                << module.wheelMaxAngularVelocity
                << ", \"wheel_max_torque\": " << module.wheelMaxTorque << "}";
}
}  // namespace helixtrajectory
