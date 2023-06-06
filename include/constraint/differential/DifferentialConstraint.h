// Copyright (c) TrajoptLib contributors

#pragma once

#include <variant>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/AngularVelocityConstraint.h"
#include "constraint/Constraint.h"
#include "constraint/differential/DifferentialCentripetalAccelerationConstraint.h"
#include "constraint/differential/DifferentialTangentialVelocityConstraint.h"
#include "solution/SolutionChecking.h"
#include "util/AppendVariant.h"

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
   * @tparam FormatContext Format string context type.
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
