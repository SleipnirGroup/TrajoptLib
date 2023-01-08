// Copyright (c) TrajoptLib contributors

#include "constraint/VelocityConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

VelocityConstraint::VelocityConstraint(const Set2d& velocityBound,
                                       CoordinateSystem coordinateSystem)
    : velocityBound(velocityBound), coordinateSystem(coordinateSystem) {}

std::optional<SolutionError> VelocityConstraint::CheckVelocity(
    double velocityX, double velocityY,
    const SolutionTolerances& tolerances) const noexcept {
  auto check = velocityBound.CheckVector(velocityX, velocityY, tolerances);
  if (check.has_value()) {
    return SolutionError{fmt::format("velocity {}", check->errorMessage)};
  }
  return std::nullopt;
}
}  // namespace trajopt
