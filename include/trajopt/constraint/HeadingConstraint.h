// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/ConeSet2d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound.
  ConeSet2d headingBound;

  /**
   * Returns an error if the given heading isn't in the heading region.
   *
   * @param theta The heading.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckHeading(
      double thetacos, double thetasin, const SolutionTolerances& tolerances) const noexcept;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(HeadingConstraint, headingBound)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::HeadingConstraint)
