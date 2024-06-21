// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/solution/Solution.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * The holonomic trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT DifferentialSolution : public Solution {
  /// The x velocities.
  std::vector<double> vL;
  /// The y velocities.
  std::vector<double> vR;
  /// the torque of the left driverail wheels
  std::vector<double> tauL;
  /// the torque of the right driverail wheels
  std::vector<double> tauR;
};

}  // namespace trajopt
