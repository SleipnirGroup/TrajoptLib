// Copyright (c) TrajoptLib contributors

#pragma once

#include <nlohmann/json.hpp>

#include "trajopt/set/IntervalSet1d.h"

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

/**
 * Formatter for PointPointConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::PointPointConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted PointPointConstraint.
   *
   * @param constraint PointPointConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::PointPointConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "point point constraint");
  }
};
