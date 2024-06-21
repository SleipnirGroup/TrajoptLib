// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Specifies the required minimum distance between a line segment on the
 * robot's frame and a point on the field.
 */
struct TRAJOPT_DLLEXPORT LinePointConstraint {
  /// robot line start x (meters)
  double robotLineStartX;
  /// robot line start y (meters)
  double robotLineStartY;
  /// robot line end x (meters)
  double robotLineEndX;
  /// robot line end y (meters)
  double robotLineEndY;
  /// field point x (meters)
  double fieldPointX;
  /// field point y (meters)
  double fieldPointY;
  /// the allowed distances (meters) between the line segment and point
  IntervalSet1d distance;
};

}  // namespace trajopt
