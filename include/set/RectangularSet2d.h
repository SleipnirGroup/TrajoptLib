// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Rectangular 2D set.
 */
struct TRAJOPT_DLLEXPORT RectangularSet2d {
  /// The x boundary.
  IntervalSet1d xBound;

  /// The y boundary.
  IntervalSet1d yBound;

  /**
   * Construct a RectangularSet2d from polar coordinates.
   *
   * @param r The distance.
   * @param theta The heading.
   */
  static RectangularSet2d PolarExactSet2d(double r, double theta);

  /**
   * Construct a RectangularSet2d spanning R².
   */
  static RectangularSet2d R2();

  /**
   * Returns an error if the given vector isn't in the region.
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * @brief Check if this planar bound is valid. A planar bound is valid when
   * the bounds on a0 and a1 are valid, and additionally for planar bounds, a0
   * is contained within the interval [0, inf] and a1 is contained within the
   * interval [-pi, pi].
   *
   * @return true if and only if this planar bound is valid
   */
  bool IsValid() const noexcept;
};
}  // namespace trajopt

/**
 * Formatter for RectangularSet2d.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::RectangularSet2d> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted RectangularSet2d.
   *
   * @tparam FormatContext Format string context type.
   * @param rectangularSet RectangularSet2d instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::RectangularSet2d& rectangularSet,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "x {}, y {}", rectangularSet.xBound,
                          rectangularSet.yBound);
  }
};
