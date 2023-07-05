// Copyright (c) TrajoptLib contributors

#pragma once

#include <nlohmann/json.hpp>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Specifies the required distance between a point on the robot's frame
 * and a point on the field.
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

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    PointPointConstraint,
    robotPointX,
    robotPointY,
    fieldPointX,
    fieldPointY,
    distance)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::PointPointConstraint)
