// Copyright (c) TrajoptLib contributors

#pragma once

#include <fmt/format.h>

#include "SymbolExports.h"
#include "obstacle/Obstacle.h"

namespace trajopt {

/**
 * Obstacle constraint.
 */
struct TRAJOPT_DLLEXPORT ObstacleConstraint {
  /// The obstacle.
  Obstacle obstacle;
};

}  // namespace trajopt

/**
 * Formatter for ObstacleConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::ObstacleConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted ObstacleConstraint.
   *
   * @tparam FormatContext Format string context type.
   * @param constraint ObstacleConstraint instance.
   * @param ctx Format string context.
   */
  template <typename FormatContext>
  auto format(const trajopt::ObstacleConstraint& constraint,
              FormatContext& ctx) {
    return fmt::format_to(ctx.out(), "obstacle constraint");
  }
};
