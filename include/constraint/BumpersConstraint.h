// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>

#include "SymbolExports.h"
#include "obstacle/Bumpers.h"

namespace trajopt {

/**
 * Bumpers constraint.
 */
struct TRAJOPT_DLLEXPORT BumpersConstraint {
  /// The obstacle.
  Bumpers bumpers;
};

}  // namespace trajopt

/**
 * Formatter for BumpersConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::BumpersConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted BumpersConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint BumpersConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::BumpersConstraint& constraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "bumpers constraint");
  }
};
