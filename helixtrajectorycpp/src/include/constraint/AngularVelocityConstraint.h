#pragma once

#include "set/IntervalSet1d.h"

namespace helixtrajectory {

    class AngularVelocityConstraint {
    public:
        IntervalSet1d angularVelocityBound;

        AngularVelocityConstraint(const IntervalSet1d& angularVelocityBound);
    };
}