// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT HolonomicVelocityConstraint {
  /// Velocity bound.
  Set2d velocityBound;

  /// Coordinate system.
  CoordinateSystem coordinateSystem;

  /**
   * Returns an error if the given velocity doesn't satisfy the constraint.
   *
   * @param velocityX The velocity's x component.
   * @param velocityY The velocity's y component.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVelocity(
      double velocityX, double velocityY,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

/**
 * Formatter for VelocityConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicVelocityConstraint> {
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
  auto format(const trajopt::HolonomicVelocityConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "velocity {}", constraint.velocityBound);
  }
};
