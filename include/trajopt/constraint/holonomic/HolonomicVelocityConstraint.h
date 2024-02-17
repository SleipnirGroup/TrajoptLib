// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/constraint/Constraint.h"
#include "trajopt/set/Set2d.h"

namespace trajopt {

/**
 * Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT HolonomicVelocityConstraint {
  /// Velocity bound.
  Set2d velocityBound;

  /// Coordinate system.
  CoordinateSystem coordinateSystem;
};

}  // namespace trajopt
