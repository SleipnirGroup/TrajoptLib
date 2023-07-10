// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"

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

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TranslationConstraint, translationBound)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::TranslationConstraint)
