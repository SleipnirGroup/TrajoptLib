// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Differential Tangential Velocity constraint.
 */
struct TRAJOPT_DLLEXPORT DifferentialTangentialVelocityConstraint {
  /// Velocity bound.
  IntervalSet1d velocityBound;
};

}  // namespace trajopt
