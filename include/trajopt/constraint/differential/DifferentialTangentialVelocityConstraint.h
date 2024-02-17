// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Differential Tangential Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT DifferentialTangentialVelocityConstraint {
  /// Velocity bound.
  IntervalSet1d velocityBound;
};

}  // namespace trajopt
