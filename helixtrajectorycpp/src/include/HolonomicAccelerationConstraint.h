#pragma once

#include "PlanarBound.h"
#include "ScalarBound.h"

namespace helixtrajectory {

    class HolonomicAccelerationConstraint {
    public:
        ScalarBound angularAccelerationBound;
        PlanarBound fieldRelativeAccelerationBound;
        PlanarBound robotRelativeAccelerationBound;

        HolonomicAccelerationConstraint(const ScalarBound& angularVelocityBound = ScalarBound(),
                const PlanarBound& fieldRelativeAccelerationBound = PlanarBound(),
                const PlanarBound& robotRelativeAccelerationBound = PlanarBound());
    };
}