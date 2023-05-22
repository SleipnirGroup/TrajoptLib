// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "solution/Solution.h"

namespace trajopt {

/**
 * The holonomic trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT DifferentialSolution : public Solution {
  /// The x velocities.
  std::vector<double> vL;
  /// The y velocities.
  std::vector<double> vR;
  std::vector<double> tauL;
  std::vector<double> tauR;
};

}  // namespace trajopt
