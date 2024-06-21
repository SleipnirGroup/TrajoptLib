// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/constraint/Constraint.hpp"
#include "trajopt/set/Set2d.hpp"
#include "trajopt/util/SymbolExports.hpp"

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
