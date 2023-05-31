// Copyright (c) TrajoptLib contributors

#pragma once

#include <optional>

#include <fmt/format.h>

#include "SymbolExports.h"
#include "constraint/Constraint.h"
#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

/**
 * .
 */
struct TRAJOPT_DLLEXPORT DifferentialCentripetalAccelerationConstraint {
  /// Acceleration bound.
  IntervalSet1d accelerationBound;
};
}  // namespace trajopt
