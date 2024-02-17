// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

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
