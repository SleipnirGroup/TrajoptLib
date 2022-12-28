#include "constraint/HeadingConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

HeadingConstraint::HeadingConstraint(const IntervalSet1d& headingBound)
        : headingBound(headingBound) {
}

std::optional<SolutionError> HeadingConstraint::CheckHeading(double heading,
        const SolutionTolerances& tolerances) const noexcept {
    auto check = headingBound.CheckScalar(heading, tolerances);
    if (check.has_value()) {
        return SolutionError(fmt::format("θ = {}: {}", heading, check->errorMessage));
    }
    return std::nullopt;
}
}