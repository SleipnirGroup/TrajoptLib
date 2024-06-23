// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies the required minimum distance between a point on the robot's
 * frame and a line segment on the field.
 */
struct TRAJOPT_DLLEXPORT PointLineConstraint {
  /// Robot point (meters).
  Translation2d robotPoint;

  /// Field line start (meters).
  Translation2d fieldLineStart;

  /// Field line end (meters).
  Translation2d fieldLineEnd;

  /// The required minimum distance (meters) between the point and line. Must be
  /// positive.
  IntervalSet1d distance;
};

}  // namespace trajopt
