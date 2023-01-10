// Copyright (c) TrajoptLib contributors

#include "drivetrain/SwerveModule.h"

namespace trajopt {

SwerveModule::SwerveModule(double x, double y, double wheelRadius,
                           double wheelMaxAngularVelocity,
                           double wheelMaxTorque)
    : x(x),
      y(y),
      wheelRadius(wheelRadius),
      wheelMaxAngularVelocity(wheelMaxAngularVelocity),
      wheelMaxTorque(wheelMaxTorque) {}

}  // namespace trajopt
