// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/set/Set2d.hpp"
#include "trajopt/util/SymbolExports.hpp"

namespace trajopt {

/**
 * Translation constraint.
 */
struct TRAJOPT_DLLEXPORT TranslationConstraint {
  /// Translation bound.
  Set2d translationBound;
};

}  // namespace trajopt
