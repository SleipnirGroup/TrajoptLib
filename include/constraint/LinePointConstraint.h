// Copyright (c) TrajoptLib contributors

#pragma once

#include "set/IntervalSet1d.h"

namespace trajopt {

/**
 * Specifies the required minimum distance between a line segment on the
 * robot's frame and a point on the field.
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

/**
 * Formatter for LinePointConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::LinePointConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted LinePointConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint PointLineConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::LinePointConstraint& constraint,
              fmt::format_context& ctx) {
    return fmt::format_to(ctx.out(), "line point constraint");
  }
};
