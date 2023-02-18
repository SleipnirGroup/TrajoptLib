// Copyright (c) TrajoptLib contributors

#include "constraint/Constraint.h"

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "constraint/HeadingConstraint.h"
#include "constraint/ObstacleConstraint.h"
#include "constraint/PoseConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckState(
    const Constraint& constraint,
    double x, double y, double heading,
    const SolutionTolerances& tolerances) noexcept {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<TranslationConstraint>(constraint).CheckTranslation(x, y, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "GetTranslationConstraint()",
                      check->errorMessage)};  // <<< causes error: "Cannot
                                              // format a const argument."
    }
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<HeadingConstraint>(constraint).CheckHeading(heading, tolerances);
    if (check.has_value()) {
      return SolutionError{fmt::format(
          "({}) violated: {}", "GetHeadingConstraint()", check->errorMessage)};
    }
  } else if (std::holds_alternative<PoseConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<PoseConstraint>(constraint).CheckPose(x, y, heading, tolerances);
    if (check.has_value()) {
      return SolutionError{fmt::format(
          "({}) violated: {}", "GetPoseConstraint()", check->errorMessage)};
    }
  }
  return std::nullopt;
}

}  // namespace trajopt
