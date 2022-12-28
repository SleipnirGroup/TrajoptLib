#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class AngularVelocityConstraint {
public:
    IntervalSet1d angularVelocityBound;

    AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);

    std::optional<SolutionError> CheckAngularVelocity(double angularVelocity,
            const SolutionTolerances& tolerances) const noexcept;
};
}