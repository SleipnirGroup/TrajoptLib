// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * @brief An initial guess of a possible state the robot may be in during the
 * trajectory.
 */
struct TRAJOPT_DLLEXPORT InitialGuessPoint {
  /// The initial guess of the x-coordinate of the robot.
  double x;

  /// The initial guess of the y-coordinate of the robot.
  double y;

  /// The initial guess of the heading of the robot.
  double heading;
};

}  // namespace trajopt
