// Copyright (c) TrajoptLib contributors

#pragma once

#include <nlohmann/json.hpp>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Specifies the required minimum distance between a point on the robot's
 * frame and a line segment on the field.
 */
struct PointLineConstraint {
  /// robot point x
  double robotPointX;
  /// robot point y
  double robotPointY;
  /// field line start x
  double fieldLineStartX;
  /// field line start y
  double fieldLineStartY;
  /// field line end x
  double fieldLineEndX;
  /// field line end y
  double fieldLineEndY;
  /// the required minimum distance between the point and line, must be positive
  IntervalSet1d distance;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(PointLineConstraint, robotPointX,
                                   robotPointY, fieldLineStartX,
                                   fieldLineStartY, fieldLineEndX,
                                   fieldLineEndY, distance)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::PointLineConstraint)
