// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "solution/Solution.h"

namespace helixtrajectory {
struct HolonomicSolution : Solution {
  std::vector<double> vx, vy, omega, ax, ay, alpha;
};
}  // namespace helixtrajectory
