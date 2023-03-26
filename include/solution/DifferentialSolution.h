// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "solution/Solution.h"

namespace trajopt {

/**
 * The holonomic trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT DifferentialSolution : Solution {
  /// The x velocities.
  std::vector<double> vl;

  /// The y velocities.
  std::vector<double> vr;

  /// The angular velocities.
  std::vector<double> omega;

  /// The x accelerations.
  std::vector<double> ax;

  /// The y accelerations.
  std::vector<double> ay;

  /// The angular accelerations.
  std::vector<double> alpha;
};

}  // namespace trajopt
