// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Angular velocity constraint.
 */
class TRAJOPT_DLLEXPORT AngularVelocityConstraint {
 public:
  /// The angular velocity bounds.
  IntervalSet1d angularVelocityBound;

  /**
   * Construct a AngularVelocityConstraint.
   *
   * @param angularVelocityBound The angular velocity bounds.
   */
  explicit AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);

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
   * @tparam FormatContext Format string context type.
   * @param constraint AngularVelocityConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::AngularVelocityConstraint& constraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "ω {}", constraint.angularVelocityBound);
  }
};
