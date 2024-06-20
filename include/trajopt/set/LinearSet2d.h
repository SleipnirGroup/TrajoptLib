// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * Linear 2D set.
 */
struct TRAJOPT_DLLEXPORT LinearSet2d {
  /// The direction in which to constrain a 2D vector.
  double theta;
};

}  // namespace trajopt
