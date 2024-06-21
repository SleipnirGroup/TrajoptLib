// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>

#include "trajopt/set/IntervalSet1d.hpp"
#include "trajopt/util/SymbolExports.hpp"

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
   * @param r The distance. Must be greater than zero.
   * @param theta The heading.
   */
  static RectangularSet2d PolarExactSet2d(double r, double theta) {
    return RectangularSet2d{r * std::cos(theta), r * std::sin(theta)};
  }

  /**
   * Construct a RectangularSet2d spanning R².
   */
  static constexpr RectangularSet2d R2() {
    return RectangularSet2d{IntervalSet1d::R1(), IntervalSet1d::R1()};
  }
};

}  // namespace trajopt
