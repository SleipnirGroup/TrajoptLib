// Copyright (c) TrajoptLib contributors

#pragma once

#include "trajopt/SymbolExports.h"
#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/RectangularSet2d.h"

namespace trajopt {

/**
 * Linear 2D set.
 */
struct TRAJOPT_DLLEXPORT LinearSet2d {
  /// FIXME What does this do?
  double theta;

  /**
   * FIXME What does this do?
   *
   * @param theta FIXME What does this do?
   * @param rBound FIXME What does this do?
   */
  static RectangularSet2d RBoundToRectangular(double theta,
                                              const IntervalSet1d& rBound);
};

}  // namespace trajopt
