// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include <fmt/format.h>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

/**
 * Translation constraint.
 */
struct TRAJOPT_DLLEXPORT TranslationConstraint {
  /// Translation bound.
  Set2d translationBound;

  /**
   * Returns an error if the given position doesn't satisfy the constraint.
   *
   * @param x The position's x component.
   * @param y The position's y component.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckTranslation(
      double x, double y, const SolutionTolerances& tolerances) const noexcept;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TranslationConstraint,
    translationBound)

}  // namespace trajopt

/**
 * Formatter for TranslationConstraint.
 */
//! @cond Doxygen_Suppress
template <>
struct fmt::formatter<trajopt::TranslationConstraint> {
  //! @endcond
  /**
   * Format string parser.
   *
   * @param ctx Format string context.
   */
  constexpr auto parse(fmt::format_parse_context& ctx) { return ctx.begin(); }

  /**
   * Writes out a formatted TranslationConstraint.
   *
   * @param constraint TranslationConstraint instance.
   * @param ctx Format string context.
   */
  auto format(const trajopt::TranslationConstraint& constraint,
              fmt::format_context& ctx) const {
    return fmt::format_to(ctx.out(), "translation {}",
                          constraint.translationBound);
  }
};
