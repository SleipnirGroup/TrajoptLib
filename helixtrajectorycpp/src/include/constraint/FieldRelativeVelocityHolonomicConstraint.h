#pragma once

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class FieldRelativeVelocityHolonomicConstraint {
    public:
        PlanarBound fieldRelativeHolonomicVelocityBound;

        HolonomicVelocityConstraint(const PlanarBound& fieldRelativeVelocityBound);
    };
}