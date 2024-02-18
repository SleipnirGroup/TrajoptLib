// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound.
  IntervalSet1d headingBound;
};

}  // namespace trajopt
