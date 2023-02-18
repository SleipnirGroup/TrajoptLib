// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/AngularVelocityConstraint.h"
#include "constraint/VelocityConstraint.h"
#include "solution/SolutionChecking.h"
#include "util/AppendVariant.h"

namespace trajopt {

// In the future this will be used
// using HolonomicConstraint = decltype(_append_variant(
//     Constraint{}, VelocityConstraint{}, AngularVelocityConstraint{}));

using HolonomicConstraint =
    std::variant<VelocityConstraint, AngularVelocityConstraint>;

/**
 * Returns an error if the given state doesn't satisfy the constraint.
 *
 * @param x The x coordinate.
 * @param y The y coordinate.
 * @param heading The heading.
 * @param velocityX The velocity's x component.
 * @param velocityY The velocity's y component.
 * @param angularVelocity The angular velocity.
 * @param accelerationX The acceleration's x component.
 * @param accelerationY The acceleration's y component.
 * @param angularAcceleration The angular acceleration.
 * @param tolerances The tolerances considered to satisfy the constraint.
 */
std::optional<SolutionError> CheckState(
    const HolonomicConstraint& constraint, double x, double y, double heading,
    double velocityX, double velocityY, double angularVelocity,
    double accelerationX, double accelerationY, double angularAcceleration,
    const SolutionTolerances& tolerances) noexcept;

}  // namespace trajopt

/**
 * Formatter for HolonomicConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HolonomicConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HolonomicConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint HolonomicConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::HolonomicConstraint& constraint,
              FormatContext& ctx) {
    using namespace trajopt;
    if (std::holds_alternative<VelocityConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<VelocityConstraint>(constraint));
    } else if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<AngularVelocityConstraint>(constraint));
    } else {
      return ctx.out();
    }
  }
};
