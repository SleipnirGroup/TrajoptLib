// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

/**
 * Differential Tangential Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT DifferentialTangentialVelocityConstraint {
  /// Velocity bound.
  IntervalSet1d velocityBound;
};
}  // namespace trajopt

/**
 * Formatter for VelocityConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::DifferentialTangentialVelocityConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted VelocityConstraint.
   *
   * @param constraint VelocityConstraint instance.
   * @param ctx Format string context.
   */
  auto format(
      const trajopt::DifferentialTangentialVelocityConstraint& constraint,
      fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "diff velocity magnitude {}",
                          constraint.velocityBound);
  }
};
