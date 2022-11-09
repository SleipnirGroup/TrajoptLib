#pragma once

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    class PositionConstraint {
    public:
        IntervalSet1d headingBound;

        HeadingConstraint(const IntervalSet1d& headingBound);
    };
}