// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Manifold 2D set.
 */
struct TRAJOPT_DLLEXPORT ManifoldIntervalSet2d {
  /// The angle at the midpoint of the range.
  double middle;

  /// Half of the width of the valid range.
  double tolerance;
};

}  // namespace trajopt