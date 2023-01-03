#include "constraint/PoseConstraint.h"

#include <optional>

#include "constraint/HeadingConstraint.h"
#include "constraint/TranslationConstraint.h"
#include "set/IntervalSet1d.h"
#include "set/Set2d.h"

namespace helixtrajectory {

PoseConstraint::PoseConstraint(const Set2d& translationBound,
        const IntervalSet1d& headingBound)
        : TranslationConstraint(translationBound), HeadingConstraint(headingBound) {
}

std::optional<SolutionError> PoseConstraint::CheckPose(double x, double y,
        double heading, const SolutionTolerances& tolerances) const noexcept {
    auto translationCheck = CheckTranslation(x, y, tolerances);
    if (translationCheck.has_value()) {
        return translationCheck.value();
    }
    auto headingCheck = CheckHeading(heading, tolerances);
    if (headingCheck.has_value()) {
        return headingCheck.value();
    }
    return std::nullopt;
}
}