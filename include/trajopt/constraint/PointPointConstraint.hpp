// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies the required distance between a point on the robot's frame
 * and a point on the field.
 */
struct TRAJOPT_DLLEXPORT PointPointConstraint {
  /// robot point x (meters)
  double robotPointX;
  /// robot point y (meters)
  double robotPointY;
  /// field point x (meters)
  double fieldPointX;
  /// field point y (meters)
  double fieldPointY;
  /// the required distance (meters) between the point and point, must be
  /// positive
  IntervalSet1d distance;
};

}  // namespace trajopt
