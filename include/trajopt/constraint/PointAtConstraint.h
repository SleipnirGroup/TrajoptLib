// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Specifies a point on the field at which the robot should point.
 */
struct PointAtConstraint {
  /// field point x
  double fieldPointX;
  /// field point y
  double fieldPointY;
  /// the allowed robot heading tolerance, must be positive
  double headingTolerance;
  /// robot heading offset
  double headingOffset;
};

}  // namespace trajopt
