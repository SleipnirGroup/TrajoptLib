#pragma once

#include "PlanarBound.h"
#include "ScalarBound.h"

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