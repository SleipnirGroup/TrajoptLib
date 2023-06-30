// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/HeadingConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> HeadingConstraint::CheckHeading(
    double theta, const SolutionTolerances& tolerances) const noexcept {
  auto check = headingBound.CheckScalar(theta, tolerances);
  if (check.has_value()) {
    return SolutionError{fmt::format("θ = {}: {}", theta, check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
