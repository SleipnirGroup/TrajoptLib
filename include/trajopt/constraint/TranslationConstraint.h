// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/Set2d.h"

namespace trajopt {

/**
 * Translation constraint.
 */
struct TRAJOPT_DLLEXPORT TranslationConstraint {
  /// Translation bound.
  Set2d translationBound;
};

}  // namespace trajopt
