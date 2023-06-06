// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "set/RectangularSet2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Linear 2D set.
 */
struct TRAJOPT_DLLEXPORT LinearSet2d {
  /// FIXME What does this do?
  double theta;

  /**
   * FIXME What does this do?
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * FIXME What does this do?
   *
   * @param theta FIXME What does this do?
   * @param rBound FIXME What does this do?
   */
  static RectangularSet2d RBoundToRectangular(double theta,
                                              const IntervalSet1d& rBound);
};
}  // namespace trajopt

/**
 * Formatter for LinearSet2d.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::LinearSet2d> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted LinearSet2d.
   *
   * @tparam FormatContext Format string context type.
   * @param linearSet LinearSet2d instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::LinearSet2d& linearSet, fmt::format_context& ctx) {
    return fmt::format_to(ctx.out(), "polar line: θ = {}", linearSet.theta);
  }
};
