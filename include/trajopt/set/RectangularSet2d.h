// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"

namespace trajopt {

/**
 * Rectangular 2D set.
 */
struct TRAJOPT_DLLEXPORT RectangularSet2d {
  /// The x boundary.
  IntervalSet1d xBound;

  /// The y boundary.
  IntervalSet1d yBound;

  /**
   * Construct a RectangularSet2d from polar coordinates.
   *
   * @param r The distance.
   * @param theta The heading.
   */
  static RectangularSet2d PolarExactSet2d(double r, double theta) {
    return RectangularSet2d{r * std::cos(theta), r * std::sin(theta)};
  }

  /**
   * Construct a RectangularSet2d spanning R².
   */
  static RectangularSet2d R2() {
    return RectangularSet2d{IntervalSet1d::R1(), IntervalSet1d::R1()};
  }

  /**
   * @brief Check if this planar bound is valid. A planar bound is valid when
   * the bounds on a0 and a1 are valid, and additionally for planar bounds, a0
   * is contained within the interval [0, inf] and a1 is contained within the
   * interval [-pi, pi].
   *
   * @return true if and only if this planar bound is valid
   */
  bool IsValid() const noexcept { return xBound.IsValid() && yBound.IsValid(); }
};

}  // namespace trajopt
