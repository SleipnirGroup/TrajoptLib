#pragma once

#include "PlanarBound.h"
#include "ScalarBound.h"

namespace helixtrajectory {

    class HolonomicAccelerationConstraint {
    public:
        ScalarBound angularAccelerationBound;
        PlanarBound fieldRelativeAccelerationBound;
        PlanarBound robotRelativeAccelerationBound;
        PlanarBound robotVelocityRelativeAccelerationBound;

        HolonomicAccelerationConstraint(const ScalarBound& angularVelocityBound = ScalarBound(),
                const PlanarBound& fieldRelativeAccelerationBound = PlanarBound(),
                const PlanarBound& robotRelativeAccelerationBound = PlanarBound(),
                const PlanarBound& robotVelocityRelativeAccelerationBound = PlanarBound());
    };
}