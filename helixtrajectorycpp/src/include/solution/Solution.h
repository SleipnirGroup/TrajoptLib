// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

namespace helixtrajectory {

struct Solution {
  std::vector<double> dt, x, y, theta;
};
}  // namespace helixtrajectory
