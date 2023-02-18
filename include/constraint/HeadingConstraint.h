// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

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
   * @tparam FormatContext Format string context type.
   * @param constraint HeadingConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::HeadingConstraint& constraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "heading {}", constraint.headingBound);
  }
};
