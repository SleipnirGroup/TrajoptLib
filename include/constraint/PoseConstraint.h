// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * Pose constraint.
 */
class TRAJOPT_DLLEXPORT PoseConstraint : public TranslationConstraint,
                                         public HeadingConstraint {
 public:
  /**
   * Construct a PoseConstraint.
   *
   * @param translationBound The trnaslation bounds.
   * @param headingBound The heading bounds.
   */
  PoseConstraint(const Set2d& translationBound,
                 const IntervalSet1d& headingBound);

  /**
   * Returns an error if the given pose is outside the region.
   *
   * @param x The x coordinate.
   * @param y The y coordinate.
   * @param heading The heading.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckPose(
      double x, double y, double heading,
      const SolutionTolerances& tolerances) const noexcept;
};
}  // namespace trajopt

/**
 * Formatter for PoseConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::PoseConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted PoseConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint PoseConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::PoseConstraint& constraint, FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "pose constraint");
  }
};
