#pragma once

#include <optional>

#include "set/Set2d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class TranslationConstraint {
public:
    Set2d translationBound;

    TranslationConstraint(const Set2d& translationBound);

    std::optional<SolutionError> CheckTranslation(double x, double y, const SolutionTolerances& tolerances) const noexcept;
};
}