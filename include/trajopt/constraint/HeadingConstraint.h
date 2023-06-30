// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound.
  IntervalSet1d headingBound;

  /**
   * Returns an error if the given heading isn't in the heading region.
   *
   * @param theta The heading.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckHeading(
      double theta, const SolutionTolerances& tolerances) const noexcept;
};

}  // namespace trajopt

/**
 * Formatter for HeadingConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::HeadingConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted HeadingConstraint.
   *
   * @param constraint HeadingConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::HeadingConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "heading {}", constraint.headingBound);
  }
};
