#include "constraint/TranslationConstraint.h"

#include <optional>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

TranslationConstraint::TranslationConstraint(const Set2d& translationBound)
        : translationBound(translationBound) {
}

std::optional<SolutionError> TranslationConstraint::CheckTranslation(double x, double y, const SolutionTolerances& tolerances) const noexcept {
    return std::nullopt;
}
}