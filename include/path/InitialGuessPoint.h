// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>

#include "SymbolExports.h"

namespace trajopt {

/**
 * @brief An initial guess of a possible state the robot may be in during the
 * trajectory.
 */
struct TRAJOPT_DLLEXPORT InitialGuessPoint {
  /// The initial guess of the x-coordinate of the robot.
  double x;

  /// The initial guess of the y-coordinate of the robot.
  double y;

  /// The initial guess of the heading of the robot.
  double heading;
};

}  // namespace trajopt

/**
 * Formatter for InitialGuessPoint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::InitialGuessPoint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted InitialGuessPoint.
   *
   * @tparam FormatContext Format string context type.
   * @param initialGuessPoint InitialGuessPoint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::InitialGuessPoint& initialGuessPoint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "Initial Guess Point");
  }
};
