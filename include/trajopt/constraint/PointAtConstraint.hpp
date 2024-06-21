// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies a point on the field at which the robot should point.
 */
struct TRAJOPT_DLLEXPORT PointAtConstraint {
  /// field point x (meters)
  double fieldPointX;
  /// field point y (meters)
  double fieldPointY;
  /// the allowed robot heading tolerance (radians), must be positive
  double headingTolerance;
};

}  // namespace trajopt
