// Copyright (c) TrajoptLib contributors

#pragma once

#include <cmath>

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
                                              const IntervalSet1d& rBound) {
    double sinTheta = std::sin(theta);
    double cosTheta = std::cos(theta);
    if (sinTheta > std::abs(cosTheta)) {  // y > |x|, up cone
      double lowerVectorY = rBound.lower * sinTheta;
      double upperVectorY = rBound.upper * sinTheta;
      return RectangularSet2d{IntervalSet1d::R1(),
                              {lowerVectorY, upperVectorY}};
    } else if (sinTheta < -std::abs(cosTheta)) {  // y < -|x|, down cone
      double lowerVectorY = rBound.upper * sinTheta;
      double upperVectorY = rBound.lower * sinTheta;
      return RectangularSet2d{IntervalSet1d::R1(),
                              {lowerVectorY, upperVectorY}};
    } else if (cosTheta >= std::abs(sinTheta)) {  // x ≥ |y|, right cone
      double lowerVectorX = rBound.lower * cosTheta;
      double upperVectorX = rBound.upper * cosTheta;
      return RectangularSet2d{{lowerVectorX, upperVectorX},
                              IntervalSet1d::R1()};
    } else /*if (cosTheta <= -std::abs(sinTheta))*/ {  // x ≤ -|y|, left cone
      double lowerVectorX = rBound.upper * cosTheta;
      double upperVectorX = rBound.lower * cosTheta;
      return RectangularSet2d{{lowerVectorX, upperVectorX},
                              IntervalSet1d::R1()};
    }
  }
};

}  // namespace trajopt
