// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/AngularVelocityConstraint.h"
#include "constraint/VelocityConstraint.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

using HolonomicConstraintVariant =
    std::variant<VelocityConstraint, AngularVelocityConstraint>;

/**
 * Holonomic constraint.
 */
class TRAJOPT_DLLEXPORT HolonomicConstraint {
 public:
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
  std::optional<SolutionError> CheckHolonomicState(
      double x, double y, double heading, double velocityX, double velocityY,
      double angularVelocity, double accelerationX, double accelerationY,
      double angularAcceleration,
      const SolutionTolerances& tolerances) const noexcept;

  /**
   * Returns true if there's a velocity constraint.
   */
  bool IsVelocityConstraint() const noexcept;

  /**
   * Returns true if there's an angular velocity constraint.
   */
  bool IsAngularVelocityConstraint() const noexcept;

  /**
   * Returns the velocity constraint.
   */
  const VelocityConstraint& GetVelocityConstraint() const;

  /**
   * Returns the velocity constraint.
   */
  VelocityConstraint& GetVelocityConstraint();

  /**
   * Returns the angular velocity constraint.
   */
  const AngularVelocityConstraint& GetAngularVelocityConstraint() const;

  /**
   * Returns the angular velocity constraint.
   */
  AngularVelocityConstraint& GetAngularVelocityConstraint();

  /**
   * Construct a HolonomicConstraint from a velocity constraint.
   *
   * @param velocityConstraint The velocity constraint.
   */
  HolonomicConstraint(  // NOLINT
      const VelocityConstraint& velocityConstraint);

  /**
   * Construct a HolonomicConstraint from an angular velocity constraint.
   *
   * @param angularVelocityConstraint The angular velocity constraint.
   */
  HolonomicConstraint(  // NOLINT
      const AngularVelocityConstraint& angularVelocityConstraint);

 private:
  HolonomicConstraintVariant constraint;
};
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
    if (constraint.IsVelocityConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetVelocityConstraint());
    } else if (constraint.IsAngularVelocityConstraint()) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            constraint.GetAngularVelocityConstraint());
    } else {
      return ctx.out();
    }
  }
};
