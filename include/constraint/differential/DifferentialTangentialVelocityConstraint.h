// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

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
   * @tparam FormatContext Format string context type.
   * @param constraint VelocityConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(
      const trajopt::DifferentialTangentialVelocityConstraint& constraint,
      FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "diff velocity magnitude {}",
                          constraint.velocityBound);
  }
};
