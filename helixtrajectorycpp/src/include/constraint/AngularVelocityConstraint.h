#pragma once

#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class AngularVelocityConstraint {
    public:
        ScalarBound angularVelocityBound;

        AngularVelocityConstraint(const ScalarBound& angularVelocityBound);
    };
}