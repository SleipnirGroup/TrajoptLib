// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/set/Set2d.h"
#include "trajopt/solution/SolutionChecking.h"

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

}  // namespace trajopt
