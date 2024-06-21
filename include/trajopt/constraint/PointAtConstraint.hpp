// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies a point on the field at which the robot should point.
 */
struct TRAJOPT_DLLEXPORT PointAtConstraint {
  /// Field point (meters).
  Translation2d fieldPoint;

  /// The allowed robot heading tolerance (radians), must be positive.
  double headingTolerance;
};

}  // namespace trajopt
