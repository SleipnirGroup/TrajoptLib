// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"

namespace trajopt {

/**
 * Manifold 2D set.
 */
struct TRAJOPT_DLLEXPORT ManifoldIntervalSet2d {
  /// The heading bounds of the cone.
  double middle;

  double tolerance;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept;
};

}  // namespace trajopt
