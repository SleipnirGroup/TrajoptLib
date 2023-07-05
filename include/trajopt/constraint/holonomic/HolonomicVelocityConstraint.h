// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <nlohmann/json.hpp>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/SolutionChecking.h"
#include "trajopt/util/JsonFmtFormatter.h"

namespace trajopt {

/**
 * Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT HolonomicVelocityConstraint {
  /// Velocity bound.
  Set2d velocityBound;

  /// Coordinate system.
  CoordinateSystem coordinateSystem;

  /**
   * Returns an error if the given velocity doesn't satisfy the constraint.
   *
   * @param velocityX The velocity's x component.
   * @param velocityY The velocity's y component.
   * @param tolerances The tolerances considered to satisfy the constraint.
   */
  std::optional<SolutionError> CheckVelocity(
      double velocityX, double velocityY,
      const SolutionTolerances& tolerances) const noexcept;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    HolonomicVelocityConstraint,
    velocityBound,
    coordinateSystem)

}  // namespace trajopt

_JSON_FMT_FORMATTER(trajopt::HolonomicVelocityConstraint)
