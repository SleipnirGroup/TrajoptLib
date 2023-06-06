// Copyright (c) TrajoptLib contributors

#include "constraint/differential/DifferentialCentripetalAccelerationConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace trajopt {

// std::optional<SolutionError>
// DifferentialCentripetalAccelerationConstraint::CheckHeading(
//     double theta, const SolutionTolerances& tolerances) const noexcept {
//   auto check = headingBound.CheckScalar(theta, tolerances);
//   if (check.has_value()) {
//     return SolutionError{fmt::format("θ = {}: {}", theta,
//     check->errorMessage)};
//   }
//   return std::nullopt;
// }
}  // namespace trajopt
