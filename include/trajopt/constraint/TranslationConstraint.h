// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

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

}  // namespace trajopt
