// Copyright (c) TrajoptLib contributors

#include "constraint/HolonomicConstraint.h"

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "constraint/AngularVelocityConstraint.h"
#include "constraint/VelocityConstraint.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckState(
    const HolonomicConstraint& constraint, double x, double y, double heading,
    double velocityX, double velocityY, double angularVelocity,
    double accelerationX, double accelerationY, double angularAcceleration,
    const SolutionTolerances& tolerances) noexcept {
  if (std::holds_alternative<VelocityConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<VelocityConstraint>(constraint)
            .CheckVelocity(velocityX, velocityY, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "*this", check->errorMessage)};
    }
  } else if (std::holds_alternative<AngularVelocityConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<AngularVelocityConstraint>(constraint)
            .CheckAngularVelocity(angularVelocity, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "*this", check->errorMessage)};
    }
  }
  return std::nullopt;
}

}  // namespace trajopt
