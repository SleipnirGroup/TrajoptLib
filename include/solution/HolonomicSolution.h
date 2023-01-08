// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"
#include "solution/Solution.h"

namespace trajopt {
struct TRAJOPT_DLLEXPORT HolonomicSolution : Solution {
  std::vector<double> vx, vy, omega, ax, ay, alpha;
};
}  // namespace trajopt
