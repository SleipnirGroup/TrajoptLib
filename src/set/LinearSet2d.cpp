// Copyright (c) TrajoptLib contributors

#include "trajopt/set/LinearSet2d.h"

#include <cmath>
#include <optional>

#include <fmt/format.h>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/set/RectangularSet2d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> LinearSet2d::CheckVector(
    double xComp, double yComp,
    const SolutionTolerances& tolerances) const noexcept {
  if (std::abs(xComp * std::sin(theta) - yComp * std::cos(theta)) >
      tolerances.errorMargin) {
    double rComp = std::hypot(xComp, yComp);
    double thetaComp = std::atan2(yComp, xComp);
    return SolutionError{fmt::format("(r, θ) = ({}, {})", rComp, thetaComp)};
  }
  return std::nullopt;
}

RectangularSet2d LinearSet2d::RBoundToRectangular(double theta,
                                                  const IntervalSet1d& rBound) {
  double sinTheta = std::sin(theta);
  double cosTheta = std::cos(theta);
  if (sinTheta > std::abs(cosTheta)) {  // y > |x|, up cone
    double lowerVectorY = rBound.lower * sinTheta;
    double upperVectorY = rBound.upper * sinTheta;
    return RectangularSet2d{IntervalSet1d::R1(), {lowerVectorY, upperVectorY}};
  } else if (sinTheta < -std::abs(cosTheta)) {  // y < -|x|, down cone
    double lowerVectorY = rBound.upper * sinTheta;
    double upperVectorY = rBound.lower * sinTheta;
    return RectangularSet2d{IntervalSet1d::R1(), {lowerVectorY, upperVectorY}};
  } else if (cosTheta >= std::abs(sinTheta)) {  // x ≥ |y|, right cone
    double lowerVectorX = rBound.lower * cosTheta;
    double upperVectorX = rBound.upper * cosTheta;
    return RectangularSet2d{{lowerVectorX, upperVectorX}, IntervalSet1d::R1()};
  } else /*if (cosTheta <= -std::abs(sinTheta))*/ {  // x ≤ -|y|, left cone
    double lowerVectorX = rBound.upper * cosTheta;
    double upperVectorX = rBound.lower * cosTheta;
    return RectangularSet2d{{lowerVectorX, upperVectorX}, IntervalSet1d::R1()};
  }
}
}  // namespace trajopt
