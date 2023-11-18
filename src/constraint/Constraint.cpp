// Copyright (c) TrajoptLib contributors

#include "trajopt/constraint/Constraint.h"

#include <optional>
#include <variant>

#include <fmt/format.h>

#include "trajopt/constraint/HeadingConstraint.h"
#include "trajopt/constraint/TranslationConstraint.h"
#include "trajopt/solution/SolutionChecking.h"

namespace trajopt {

std::optional<SolutionError> CheckState(
    const Constraint& constraint, double x, double y, double headingcos, double headingsin,
    const SolutionTolerances& tolerances) noexcept {
  if (std::holds_alternative<TranslationConstraint>(constraint)) {
    std::optional<SolutionError> check =
        std::get<TranslationConstraint>(constraint)
            .CheckTranslation(x, y, tolerances);
    if (check.has_value()) {
      return SolutionError{
          fmt::format("({}) violated: {}", "GetTranslationConstraint()",
                      check->errorMessage)};  // <<< causes error: "Cannot
                                              // format a const argument."
    }
  } else if (std::holds_alternative<HeadingConstraint>(constraint)) {
    std::optional<SolutionError> check = std::get<HeadingConstraint>(constraint)
                                             .CheckHeading(headingcos, headingsin, tolerances);
    if (check.has_value()) {
      return SolutionError{fmt::format(
          "({}) violated: {}", "GetHeadingConstraint()", check->errorMessage)};
    }
  }
  return std::nullopt;
}

}  // namespace trajopt
