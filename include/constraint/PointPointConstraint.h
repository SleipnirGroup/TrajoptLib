// Copyright (c) TrajoptLib contributors

#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

/**
 * @brief Specifies the required distance between a point on the robot's frame and a point on the field.
 */
struct PointPointConstraint {
  /// robot point x
  double robotPointX;
  /// robot point y
  double robotPointY;
  /// field point x
  double fieldPointX;
  /// field point y
  double fieldPointY;
  /// the required distance between the point and point, must be positive
  IntervalSet1d distance;
};
}  // namespace trajopt
