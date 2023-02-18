// Copyright (c) TrajoptLib contributors

#include "constraint/AngularVelocityConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

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
