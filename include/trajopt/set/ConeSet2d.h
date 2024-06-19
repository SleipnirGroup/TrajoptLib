// Copyright (c) TrajoptLib contributors

#pragma once

#include <numbers>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Conical 2D set.
 */
struct TRAJOPT_DLLEXPORT ConeSet2d {
  /// The heading bounds of the cone.
  IntervalSet1d thetaBound;

  /**
   * Returns true if the set is valid.
   */
  bool IsValid() const noexcept {
    return thetaBound.Range() > 0.0 && thetaBound.Range() <= std::numbers::pi;
  }
};

}  // namespace trajopt
