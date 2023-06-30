// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.h"

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
}  // namespace trajopt

/**
 * Formatter for PointLineConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::PointLineConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted PointLineConstraint.
   *
   * @param constraint PointLineConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::PointLineConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "point line constraint");
  }
};
