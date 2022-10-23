#pragma once

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class HolonomicVelocityConstraint {
    public:
        ScalarBound angularVelocityBound;
        PlanarBound fieldRelativeVelocityBound;
        PlanarBound robotRelativeVelocityBound;

        HolonomicVelocityConstraint(const ScalarBound& angularVelocityBound = ScalarBound(),
                const PlanarBound& fieldRelativeVelocityBound = PlanarBound(),
                const PlanarBound& robotRelativeVelocityBound = PlanarBound());
    };
}