#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class HeadingConstraint {
public:
    IntervalSet1d headingBound;

    HeadingConstraint(const IntervalSet1d& headingBound);

    std::optional<SolutionError> CheckHeading(double heading,
            const SolutionTolerances& tolerances) const noexcept;
};
}