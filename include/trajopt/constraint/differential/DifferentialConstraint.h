// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/constraint/differential/DifferentialCentripetalAccelerationConstraint.h"
#include "trajopt/constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/AppendVariant.h"

namespace trajopt {

using DifferentialConstraint =
    decltype(_append_variant(Constraint{}, AngularVelocityConstraint{},
                             DifferentialTangentialVelocityConstraint{},
                             DifferentialCentripetalAccelerationConstraint{}));
}  // namespace trajopt

/**
 * Formatter for HolonomicConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::DifferentialConstraint> {
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
   * @param constraint HolonomicConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::DifferentialConstraint& constraint,
              fmt::format_context& ctx) const {
    using namespace trajopt;
    if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
      return fmt::format_to(ctx.out(), "constraint: {}",
                            std::get<AngularVelocityConstraint>(constraint));
    } else if (std::holds_alternative<DifferentialTangentialVelocityConstraint>(
                   constraint)) {
      return fmt::format_to(
          ctx.out(), "constraint: {}",
          std::get<DifferentialTangentialVelocityConstraint>(constraint));
    } else if (std::holds_alternative<
                   DifferentialCentripetalAccelerationConstraint>(constraint)) {
      return fmt::format_to(
          ctx.out(), "constraint: {}",
          std::get<DifferentialCentripetalAccelerationConstraint>(constraint));
    } else {
      return ctx.out();
    }
  }
};
