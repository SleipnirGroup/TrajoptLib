// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Angular velocity constraint.
 */
struct TRAJOPT_DLLEXPORT AngularVelocityConstraint {
  /// The angular velocity bounds (rad/s)
  IntervalSet1d angularVelocityBound;
};

}  // namespace trajopt
