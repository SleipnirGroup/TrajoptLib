// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/ManifoldIntervalSet2d.h"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound (radians)
  ManifoldIntervalSet2d headingBound;
};

}  // namespace trajopt
