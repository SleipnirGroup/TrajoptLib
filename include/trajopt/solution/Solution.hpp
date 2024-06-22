// Copyright (c) TrajoptLib contributors

#pragma once

#include <vector>

#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * The trajectory optimization solution.
 */
struct TRAJOPT_DLLEXPORT Solution {
  /// Times between samples.
  std::vector<double> dt;

  /// X positions.
  std::vector<double> x;

  /// Y positions.
  std::vector<double> y;

  /// Heading cosine.
  std::vector<double> thetacos;
  /// Heading sine.
  std::vector<double> thetasin;
};

}  // namespace trajopt
