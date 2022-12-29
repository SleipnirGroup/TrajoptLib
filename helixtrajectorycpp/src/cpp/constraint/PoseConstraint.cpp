#include "constraint/PoseConstraint.h"

#include <optional>

#include <fmt/format.h>

#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"

namespace helixtrajectory {

PoseConstraint::PoseConstraint(const Set2d& translationBound,
        const IntervalSet1d& headingBound)
        : TranslationConstraint(translationBound), HeadingConstraint(headingBound) {
}

std::optional<SolutionError> PoseConstraint::CheckPose(double x, double y, double theta, const SolutionTolerances& tolerances) {
    Check
    auto translationCheck = translationBound.CheckVector(x, y, tolerances);
    if (check.has_value()) {
        return SolutionError(fmt::format("(x, y) = ({}, {}), {}", x, y, check->errorMessage));
    }
    return std::nullopt;
}
}