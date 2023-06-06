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
 * Differential centripetal acceleration constraint
 */
struct TRAJOPT_DLLEXPORT DifferentialCentripetalAccelerationConstraint {
  /// Acceleration bound.
  IntervalSet1d accelerationBound;
};
}  // namespace trajopt

/**
 * Formatter for DifferentialCentripetalAccelerationConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::DifferentialCentripetalAccelerationConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted DifferentialCentripetalAccelerationConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint DifferentialCentripetalAccelerationConstraint instance.
   * @param ctx Format string context.
   */
  auto format(
      const trajopt::DifferentialCentripetalAccelerationConstraint& constraint,
      fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "diff centrip acceleration bound");
  }
};
