#pragma once

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    class HeadingConstraint {
    public:
        IntervalSet1d headingBound;

        HeadingConstraint(const IntervalSet1d& headingBound);
    };
}