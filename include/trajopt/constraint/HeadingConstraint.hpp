// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.hpp"
#include "trajopt/set/ManifoldIntervalSet2d.hpp"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound (radians)
  ManifoldIntervalSet2d headingBound;
};

}  // namespace trajopt
