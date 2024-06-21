// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/geometry/Translation2.hpp"
#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies the required minimum distance between a line segment on the
 * robot's frame and a point on the field.
 */
struct TRAJOPT_DLLEXPORT LinePointConstraint {
  /// Robot line start (meters).
  Translation2d robotLineStart;

  /// Robot line end (meters).
  Translation2d robotLineEnd;

  /// Field point (meters).
  Translation2d fieldPoint;

  /// The allowed distances (meters) between the line segment and point.
  IntervalSet1d distance;
};

}  // namespace trajopt
