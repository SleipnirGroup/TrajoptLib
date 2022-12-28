#pragma once

#include <optional>

#include "set/IntervalSet1d.h"
#include "solution/SolutionChecking.h"

namespace helixtrajectory {

class ConeSet2d {
public:
    IntervalSet1d thetaBound;

    ConeSet2d(const IntervalSet1d& thetaBound);

    std::optional<SolutionError> CheckVector(double xComp, double yComp, const SolutionTolerances& tolerances) const noexcept;

    bool IsValid() const noexcept;
};
}