// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies the required distance between a point on the robot's frame
 * and a point on the field.
 */
struct TRAJOPT_DLLEXPORT PointPointConstraint {
  /// Robot point (meters).
  Translation2d robotPoint;

  /// Field point (meters).
  Translation2d fieldPoint;

  /// The required distance (meters) between the robot point and field point.
  /// Must be positive.
  IntervalSet1d distance;
};

}  // namespace trajopt
