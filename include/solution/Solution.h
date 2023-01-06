// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "SymbolExports.h"

namespace trajopt {

struct TRAJOPT_DLLEXPORT Solution {
  std::vector<double> dt, x, y, theta;
};
}  // namespace trajopt
