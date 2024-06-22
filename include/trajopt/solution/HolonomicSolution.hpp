// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/solution/Solution.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * The holonomic trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT HolonomicSolution : public Solution {
  /// The x velocities.
  std::vector<double> vx;

  /// The y velocities.
  std::vector<double> vy;

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
