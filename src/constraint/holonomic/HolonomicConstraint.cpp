// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/holonomic/HolonomicConstraint.h"

#include <optional>
#include <string>
#include <variant>

#include <fmt/format.h>

#include "trajopt/constraint/AngularVelocityConstraint.h"
#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/LinePointConstraint.h"
#include "trajopt/constraint/PointLineConstraint.h"
#include "trajopt/constraint/PointPointConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/constraint/holonomic/HolonomicVelocityConstraint.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckState(
    const HolonomicConstraint& constraint, double x, double y, double heading,
    double velocityX, double velocityY, double angularVelocity,
    double accelerationX, double accelerationY, double angularAcceleration,
    const SolutionTolerances& tolerances) noexcept {
  if (std::holds_alternative<HolonomicVelocityConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<HolonomicVelocityConstraint>(constraint)
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
