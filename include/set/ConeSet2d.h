// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Conical 2D set.
 */
struct TRAJOPT_DLLEXPORT ConeSet2d {
  /// The heading bounds of the cone.
  IntervalSet1d thetaBound;

  /**
   * Returns an error if the given coordinate is outside the cone.
   *
   * @param xComp The x coordinate.
   * @param yComp The y coordinate.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVector(
      double xComp, double yComp,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept;
};
}  // namespace trajopt

/**
 * Formatter for ConeSet2d.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::ConeSet2d> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted ConeSet2d.
   *
   * @param coneSet ConeSet2d instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::ConeSet2d& coneSet,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "cone: θ = {}", coneSet.thetaBound);
  }
};
