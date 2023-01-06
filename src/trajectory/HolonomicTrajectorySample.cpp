// Copyright (c) TrajoptLib contributors

#include "trajectory/HolonomicTrajectorySample.h"

#include <iostream>

namespace trajopt {

HolonomicTrajectorySample::HolonomicTrajectorySample(double timestamp, double x,
                                                     double y, double heading,
                                                     double velocityX,
                                                     double velocityY,
                                                     double angularVelocity)
    : timestamp(timestamp),
      x(x),
      y(y),
      heading(heading),
      velocityX(velocityX),
      velocityY(velocityY),
      angularVelocity(angularVelocity) {}

}  // namespace trajopt
