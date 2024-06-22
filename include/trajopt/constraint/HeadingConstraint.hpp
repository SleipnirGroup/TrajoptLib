// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/ManifoldIntervalSet2d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound (radians)
  ManifoldIntervalSet2d headingBound;
};

}  // namespace trajopt
