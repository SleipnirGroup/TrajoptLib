// Copyright (c) TrajoptLib contributors

#include "trajopt/set/ConeSet2d.h"

#include <cmath>
#include <numbers>
#include <optional>

#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> ConeSet2d::CheckVector(
    double xComp, double yComp,
    const SolutionTolerances& tolerances) const noexcept {
  if (!(xComp * std::sin(thetaBound.upper) >=
            yComp * std::cos(thetaBound.upper) &&
        xComp * std::sin(thetaBound.lower) <=
            yComp * std::cos(thetaBound.lower))) {
    double rComp = std::hypot(xComp, yComp);
    double thetaComp = std::atan2(yComp, xComp);
    return SolutionError{fmt::format("(r, θ) = ({}, {})", rComp, thetaComp)};
  }
  return std::nullopt;
}

bool ConeSet2d::IsValid() const noexcept {
  return thetaBound.Range() > 0.0 && thetaBound.Range() <= std::numbers::pi;
}
}  // namespace trajopt
