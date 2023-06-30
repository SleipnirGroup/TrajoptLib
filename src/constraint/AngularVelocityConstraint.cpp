// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/AngularVelocityConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "trajopt/set/IntervalSet1d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> AngularVelocityConstraint::CheckAngularVelocity(
    double angularVelocity,
    const SolutionTolerances& tolerances) const noexcept {
  auto check = angularVelocityBound.CheckScalar(angularVelocity, tolerances);
  if (check.has_value()) {
    return SolutionError{fmt::format("ω {}", check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
