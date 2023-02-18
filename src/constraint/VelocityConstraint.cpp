// Copyright (c) TrajoptLib contributors

#include "constraint/VelocityConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> VelocityConstraint::CheckVelocity(
    double velocityX, double velocityY,
    const SolutionTolerances& tolerances) const noexcept {
  auto check = CheckVector(velocityBound, velocityX, velocityY, tolerances);
  if (check.has_value()) {
    return SolutionError{fmt::format("velocity {}", check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
