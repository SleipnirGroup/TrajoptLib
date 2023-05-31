// Copyright (c) TrajoptLib contributors

#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

/**
 * @brief Specifies the required minimum distance between a line segment on the robot's frame and a point on the field.
 */
struct LinePointConstraint {
  /// robot line start x
  double robotLineStartX;
  /// robot line start y
  double robotLineStartY;
  /// robot line end x
  double robotLineEndX;
  /// robot line end y
  double robotLineEndY;
  /// field point x
  double fieldPointX;
  /// field point y
  double fieldPointY;
  /// the required minimum distance between the line and point, must be positive
  IntervalSet1d distance;
};
}  // namespace trajopt
