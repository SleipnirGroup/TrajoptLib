// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

/**
 * Angular velocity constraint.
 */
struct TRAJOPT_DLLEXPORT AngularVelocityConstraint {
  /// The angular velocity bounds.
  IntervalSet1d angularVelocityBound;

  /**
   * Returns an error if the angular velocity is outside the bounds.
   *
   * @param angularVelocity The angular velocity.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckAngularVelocity(
      double angularVelocity,
      const SolutionTolerances& tolerances) const noexcept;
};

}  // namespace trajopt

/**
 * Formatter for AngularVelocityConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::AngularVelocityConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted AngularVelocityConstraint.
   *
   * @param constraint AngularVelocityConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::AngularVelocityConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "ω {}", constraint.angularVelocityBound);
  }
};
