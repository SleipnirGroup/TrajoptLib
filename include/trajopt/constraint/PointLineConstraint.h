// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Specifies the required minimum distance between a point on the robot's
 * frame and a line segment on the field.
 */
struct PointLineConstraint {
  /// robot point x (meters)
  double robotPointX;
  /// robot point y (meters)
  double robotPointY;
  /// field line start x (meters)
  double fieldLineStartX;
  /// field line start y (meters)
  double fieldLineStartY;
  /// field line end x (meters)
  double fieldLineEndX;
  /// field line end y (meters)
  double fieldLineEndY;
  /// the required minimum distance (meters) between the point and line, must be
  /// positive
  IntervalSet1d distance;
};

}  // namespace trajopt
