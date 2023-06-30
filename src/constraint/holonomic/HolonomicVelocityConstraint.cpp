// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "trajopt/set/Set2d.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> HolonomicVelocityConstraint::CheckVelocity(
    double velocityX, double velocityY,
    const SolutionTolerances& tolerances) const noexcept {
  auto check = CheckVector(velocityBound, velocityX, velocityY, tolerances);
  if (check.has_value()) {
    return SolutionError{fmt::format("velocity {}", check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
