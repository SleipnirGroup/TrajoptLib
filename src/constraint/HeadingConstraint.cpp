// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/HeadingConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "trajopt/set/ConeSet2d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> HeadingConstraint::CheckHeading(
    double thetacos, double thetasin, const SolutionTolerances& tolerances) const noexcept {
  auto check = headingBound.CheckVector(thetacos, thetasin, tolerances);
  if (check.has_value()) {
    double theta = std::atan2(thetasin, thetacos);
    return SolutionError{fmt::format("θ = {}: {}", theta, check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
