// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/solution/HolonomicSolution.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * The swerve drive trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT SwerveSolution : HolonomicSolution {
  /// The x forces for each module.
  std::vector<std::vector<double>> moduleFX;

  /// The y forces for each module.
  std::vector<std::vector<double>> moduleFY;
};

}  // namespace trajopt
