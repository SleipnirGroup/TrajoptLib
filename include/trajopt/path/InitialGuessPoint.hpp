// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.hpp"

namespace trajopt {

/**
 * @brief An initial guess of a possible state the robot may be in during the
 * trajectory.
 */
struct TRAJOPT_DLLEXPORT InitialGuessPoint {
  /// The initial guess of the x-coordinate (meters) of the robot.
  double x;

  /// The initial guess of the y-coordinate (meters) of the robot.
  double y;

  /// The initial guess of the heading (radians) of the robot.
  double heading;
};

}  // namespace trajopt
