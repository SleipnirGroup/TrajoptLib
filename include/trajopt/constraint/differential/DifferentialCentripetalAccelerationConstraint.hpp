// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Differential centripetal acceleration constraint
 */
struct TRAJOPT_DLLEXPORT DifferentialCentripetalAccelerationConstraint {
  /// Acceleration bound.
  IntervalSet1d accelerationBound;
};

}  // namespace trajopt
