#pragma once

#include "constraint/PlanarBound.h"
#include "constraint/ScalarBound.h"

namespace helixtrajectory {

    class HolonomicAccelerationConstraint {
    public:
        ScalarBound angularAccelerationBound;
        PlanarBound fieldRelativeAccelerationBound;
        PlanarBound robotRelativeAccelerationBound;

        HolonomicAccelerationConstraint(const ScalarBound& angularAccelerationBound = ScalarBound(),
                const PlanarBound& fieldRelativeAccelerationBound = PlanarBound(),
                const PlanarBound& robotRelativeAccelerationBound = PlanarBound());
    };
}