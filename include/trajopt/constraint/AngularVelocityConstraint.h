// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Angular velocity constraint.
 */
struct TRAJOPT_DLLEXPORT AngularVelocityConstraint {
  /// The angular velocity bounds.
  IntervalSet1d angularVelocityBound;

  /**
   * Returns an error if the angular velocity is outside the bounds.
   *
   * @param angularVelocity The angular velocity.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckAngularVelocity(
      double angularVelocity,
      const SolutionTolerances& tolerances) const noexcept;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(AngularVelocityConstraint,
                                   angularVelocityBound)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::AngularVelocityConstraint)
