// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/ConeSet2d.h"

namespace trajopt {

/**
 * Heading constraint.
 */
struct TRAJOPT_DLLEXPORT HeadingConstraint {
  /// The heading bound.
  ConeSet2d headingBound;
};

}  // namespace trajopt
